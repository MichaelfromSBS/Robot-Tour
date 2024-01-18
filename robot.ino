#include <LiquidCrystal_I2C.h>
#include <Wire.h>

#define PIN_MTR1_ENCA    2
#define PIN_MTR2_ENCA    3
#define PIN_PB_START     4
#define PIN_MTR1_DIR_FWD 5
#define PIN_MTR1_DIR_REV 6
#define PIN_MTR2_DIR_FWD 7
#define PIN_MTR2_DIR_REV 8
#define PIN_MTR1_PWM     9
#define PIN_MTR2_PWM     10

#define PULSES_PER_MM         2.257
#define ENCODER_COUNTS_90_DEG 315 // Set to the number of encoder pulses to make a 90 degree turn
#define SPEED_MIN             120 // Minimum speed (pulses/second) use at the end of individual moves

LiquidCrystal_I2C display { 0x27, 20, 4 };

unsigned long usLast = 0;
unsigned long usScanLong = 0;
int usLongResetCount = 0;
unsigned long usScanAvg = 0;
unsigned long timerusScan = 0;
int scanCount = 0;

unsigned long timerRunTime = 0;
int speedFwd = 100;
int speedTurn = 100;
bool flagTimeRun = false;

unsigned long msTimerPrint = 0;

unsigned long timerPBStartOn = 0;
unsigned long timerPBStartOff = 0;

#define MAX_COMMANDS 60 // Maximum number of motion commands allowed

//======================================================================================
// Command object is used to hold a list of commands to be executed
//======================================================================================
class CommandQueue {
private:
    int start;
    int end;
    int list[MAX_COMMANDS];
    int p1[MAX_COMMANDS];
    int flagFirstScan;

public:
    int current() { return list[start]; }
    int getParameter1() { return p1[start]; }
    int empty() { return (start == end) ? 127 : 0; }

    int firstScan()
    {
        int flag = flagFirstScan;
        flagFirstScan = 0;
        return flag;
    }

    void add(int cmd, int par1 = 0)
    {
        list[end] = cmd;
        p1[end] = par1;
        end++;
        if (end >= MAX_COMMANDS)
            end = 0;
    }

    int next()
    {
        flagFirstScan = 127;
        if (empty())
            return 0;
        start++;
        if (start >= MAX_COMMANDS)
            start = 0;
        return list[start];
    }
};

CommandQueue cmdQueue {};

#define VEHICLE_START_WAIT     1 // Wait for the start button to be pressed
#define VEHICLE_START          2 // First motion command after button press
#define VEHICLE_FORWARD        10
#define VEHICLE_TURN_RIGHT     40
#define VEHICLE_TURN_LEFT      50
#define VEHICLE_SET_MOVE_SPEED 101
#define VEHICLE_SET_TURN_SPEED 102
#define VEHICLE_SET_ACCEL      105
#define VEHICLE_FINISHED       900  // Must be at the end of the command list
#define VEHICLE_ABORT          2000 // Used to abort the current movement list and stop the robot

void loadCommandQueue()
{
    cmdQueue = {};
    cmdQueue.add(VEHICLE_START_WAIT); // do not change this line - waits for start pushbutton
    cmdQueue.add(VEHICLE_START);      // do not change this line

    // Define robot movement speeds
    // Speed is encoder pulses per second.
    // There is a maximum speed.  Testing will be required to learn this speed.
    //    SETTING THE SPEEDS ABOVE THE MOTOR'S MAXIMUM SPEED WILL CAUSE STRANGE RESULTS
    cmdQueue.add(VEHICLE_SET_MOVE_SPEED, 500); // Speed used for forward movements
    cmdQueue.add(VEHICLE_SET_TURN_SPEED, 300); // Speed used for left or right turns
    cmdQueue.add(VEHICLE_SET_ACCEL, 400);      // smaller is softer   larger is quicker and less accurate moves

    // Example list of robot movements
    // This block is modified for each tournament
    cmdQueue.add(VEHICLE_FORWARD, 500);
    cmdQueue.add(VEHICLE_TURN_LEFT);
    cmdQueue.add(VEHICLE_FORWARD, 500);
    cmdQueue.add(VEHICLE_TURN_LEFT);
    cmdQueue.add(VEHICLE_FORWARD, 500);
    cmdQueue.add(VEHICLE_TURN_LEFT);
    cmdQueue.add(VEHICLE_FORWARD, 500);
    cmdQueue.add(VEHICLE_TURN_LEFT);

    // This MUST be the last command.
    cmdQueue.add(VEHICLE_FINISHED);
}

//======================================================================================
// Motion object (like a library) that calculates the acceleration used for motor speed
// control.
//
// Distance is encoder pulses
// Speed is encoder pulses per second
// Accel is encoder pulses per second^2
//======================================================================================
class MotionLogic {
public:
    bool isForward() { return forward; }
    bool isStopped() { return stopped; }

    void incrEncoder() { ++countEncoder; }

    void enableDebug() { debug = true; }
    bool getDebug() { return debug; }

    void setParams(long accel, int spdMin, int pPWM)
    {
        accelRate = accel;
        decelRate = accel;
        speedMinimum = spdMin;
        pinPWM = pPWM;
    }

    void setAccel(long accel)
    {
        accelRate = accel;
        decelRate = accel;
    }

    void startMove(int pos, int spd)
    {
        forward = spd > 0;

        speedTarget = abs(spd);
        positionTarget = abs(pos);
        if (debug) {
            Serial.print(F("Motion - speed       = "));
            Serial.println(speedTarget);
            Serial.print(F("Motion - position    = "));
            Serial.println(positionTarget);
        }

        float tAccel = (float)speedTarget / (float)accelRate;
        float tDecel = (float)speedTarget / (float)decelRate;
        float distAccel = (float)speedTarget / 2.0 * tAccel;
        float distDecel = (float)speedTarget / 2.0 * tDecel;
        float distAtSpeed = (float)positionTarget - distAccel - distDecel;
        if (distAtSpeed < 0.0) { // current written as accel and decel same
            // Serial.println("Motion - Short move logic");
            distAccel = (float)positionTarget / 2.0;
            distDecel = (float)positionTarget / 2.0;
            distAtSpeed = 0.0;

            tAccel = sqrt(distAccel * 2.0 / (float)accelRate);
            tDecel = sqrt(distDecel * 2.0 / (float)decelRate);
        }
        float tAtSpeed = distAtSpeed / (float)speedTarget;

        // times are in microseconds
        timeAtSpeed = tAccel * 1000000;
        timeDecel = timeAtSpeed + tAtSpeed * 1000000;

        pwmLoopI = 0;
        pwmLoopP = 0;
        countEncoder = 0;
        countEncoderLast = 0;
        posProfile = 0;
        speedAtDecel = -10000;
        timeRunning = 0;
        timeRunningLast = 0;
        timerUpdate = 0;
        stopped = false;
        counterStopped = 0;
        running = true;
    }

    void stop()
    {
        forward = true;
        analogWrite(pinPWM, 0);
        running = false;
        posProfile = 0;
        speedProfile = 0;
        speedTarget = 0;
    }

    void updateMotion(long usecElapsed)
    {
        int flagUpdate = 0;
        int speedActual;

        timerUpdate += usecElapsed;
        if (timerUpdate >= 30000) {
            long delta = countEncoder - countEncoderLast;
            speedActual = delta * 1000000L / timerUpdate;
            countEncoderLast = countEncoder;
            timerUpdate = 0;
            flagUpdate = 1;
            if (!running && speedActual < 4) {
                if (!stopped)
                    ++counterStopped;
                if (counterStopped > 2)
                    stopped = true;
            }
        }

        if (!running) {
            forward = true;
            pwmLoopP = 0;
            pwmLoopI = 0;
            return;
        }

        if (countEncoder >= (positionTarget - 2)) {
            if (debug) {
                Serial.print("STOPPED,");
                Serial.print("timeRunning:");
                Serial.print(timeRunning);
                Serial.print(",encoder:");
                Serial.print(countEncoder);
                Serial.print(",speedActual:");
                Serial.print(speedActual);
                Serial.print(",pwm:");
            }

            stop();
            return;
        }

        timeRunning += usecElapsed;
        if (flagUpdate == 0)
            return;

        float speed;
        if (timeRunning < timeAtSpeed) {
            speed = (float)timeRunning / 1000000.0 * (float)accelRate;
            speedProfile = (int)speed;
            // if (debug) { Serial.print(" - Accel speedProfile = "); Serial.println(speedProfile); }
        } else if (timeRunning < timeDecel) {
            speedProfile = speedTarget;
            // if (debug) { Serial.print(" - At Speed speedProfile = "); Serial.println(speedProfile); }
        } else {
            if (speedAtDecel <= -10000)
                speedAtDecel = speedProfile;
            speed = (float)(timeRunning - timeDecel) / 1000000.0 * (float)decelRate;
            speedProfile = speedAtDecel - (int)speed;
            if (speedProfile < speedMinimum)
                speedProfile = speedMinimum;
            // if (debug) { Serial.print(" - Decel speedProfile = "); Serial.println(speedProfile); }
        }

        posProfile += (speedProfile * (timeRunning - timeRunningLast)) / 1000000;
        timeRunningLast = timeRunning;

        long perror = posProfile - countEncoder;
        int serror = speedProfile - speedActual;

        // This program uses a PI loop to control speed
        // This loop is NOT tuned.  Meaning testing is required to tune the loop
        // which will provide the best preformance
        pwmLoopI += serror / 4;
        pwmLoopP = serror / 2;

        auto outputPWM = pwmLoopP + pwmLoopI;
        if (outputPWM < 0)
            outputPWM = 0;
        if (outputPWM > 254)
            outputPWM = 254;

        if (debug) {
            Serial.print("timeRunning:");
            Serial.print(timeRunning);
            Serial.print(",encoder:");
            Serial.print(countEncoder);
            Serial.print(",speedProfile:");
            Serial.print(speedProfile);
            Serial.print(",speedActual:");
            Serial.print(speedActual);
            Serial.print(",pos_error:");
            Serial.print(perror);
            Serial.print(",speed_error:");
            Serial.print(serror);
            Serial.print(",loopI:");
            Serial.print(pwmLoopI);
            Serial.print(",loopP:");
            Serial.print(pwmLoopP);
            Serial.print(",pwm:");
            Serial.println(outputPWM);
        }

        analogWrite(pinPWM, outputPWM);
    }

private:
    long timeAtSpeed = 0;
    long timeDecel = 0;

    long timeRunning = 0;
    long timeRunningLast = 0;
    long timerUpdate = 0;

    int pinPWM;
    bool forward = true;

    bool running = false;
    bool stopped = false;
    int counterStopped = 0;

    int pwmLoopI = 0;
    int pwmLoopP = 0;

    bool debug = false;

    long countEncoder = 0;
    long countEncoderLast = 0;

    long accelRate = 200;
    long decelRate = 200;
    long posProfile = 0;

    long positionTarget = 0;
    int speedTarget = 0;
    int speedProfile = 0;
    int speedMinimum = 0;
    int speedAtDecel = -10000;
};

MotionLogic mtrLeft;
MotionLogic mtrRight;

// Interupt function for counting left motor encoder pulses
void encoderIntLeft()
{
    mtrLeft.incrEncoder();
}

// Interupt function for counting right motor encoder pulses
void encoderIntRight()
{
    mtrRight.incrEncoder();
}

void setMotorDirection()
{
    if (mtrLeft.isForward()) {
        digitalWrite(PIN_MTR1_DIR_FWD, HIGH);
        digitalWrite(PIN_MTR1_DIR_REV, LOW);
    } else {
        digitalWrite(PIN_MTR1_DIR_FWD, LOW);
        digitalWrite(PIN_MTR1_DIR_REV, HIGH);
    }

    if (mtrRight.isForward()) {
        digitalWrite(PIN_MTR2_DIR_FWD, HIGH);
        digitalWrite(PIN_MTR2_DIR_REV, LOW);
    } else {
        digitalWrite(PIN_MTR2_DIR_FWD, LOW);
        digitalWrite(PIN_MTR2_DIR_REV, HIGH);
    }
}

void initDisplay()
{
    display.clear();
    display.setCursor(0, 0);
    display.print(F("Cmd:"));

    display.setCursor(0, 2);
    display.print(F("Rng:"));

    display.setCursor(0, 3);
    display.print(F("Time:"));
}

void updateDisplay()
{
    static int lastCmd = 0;
    static int printStep = 0;

    switch (printStep) {
    case 0: {
        auto cmd = cmdQueue.current();
        if (lastCmd != cmd) {
            display.setCursor(4, 0);
            switch (cmd) {
            case VEHICLE_START_WAIT:
                display.print(F("WAIT START     "));
                break;
            case VEHICLE_START:
                display.print(F("WAIT RELEASE   "));
                break;
            case VEHICLE_FORWARD:
                display.print(F("FORWARD        "));
                break;
            case VEHICLE_TURN_RIGHT:
                display.print(F("TURN RIGHT     "));
                break;
            case VEHICLE_TURN_LEFT:
                display.print(F("TURN LEFT      "));
                break;
            case VEHICLE_FINISHED:
                display.print(F("FINISHED       "));
                break;
            case VEHICLE_ABORT:
                display.print(F("ABORT          "));
                break;
            default:
                display.print(F("***unknown**"));
                break;
            }
            lastCmd = cmd;
        }
        break;
    }
    case 1:
    case 2:
        break;
    case 3:
        display.setCursor(4, 2);
        display.print("no sonic");
        break;
    case 4: {
        display.setCursor(6, 3);
        auto time = (float)timerRunTime / 1000000.0;
        display.print(time, 3);
        break;
    }
    default:
        printStep = 0;
        return;
    }

    ++printStep;
}

void setup()
{
    // Only uncomment one motor at a time to use the Serial Plotter function to tune the PID loop
    // mtrLeft.enableDebug();
    // mtrRight.enableDebug();

    if (mtrLeft.getDebug() || mtrRight.getDebug()) {
        Serial.begin(115200);
        Serial.println(F("Setup()..."));
    }

    // Serial.println(F("Display init()"));
    display.init();      // initialize the lcd
    display.backlight(); // open the backlight
    display.clear();
    display.setCursor(0, 0);
    display.print(F("Start Up....."));

    pinMode(PIN_PB_START, INPUT_PULLUP);

    pinMode(PIN_MTR1_PWM, OUTPUT);
    pinMode(PIN_MTR2_PWM, OUTPUT);

    pinMode(PIN_MTR1_ENCA, INPUT_PULLUP);
    pinMode(PIN_MTR2_ENCA, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PIN_MTR1_ENCA), encoderIntLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_MTR2_ENCA), encoderIntRight, RISING);

    pinMode(PIN_MTR1_DIR_FWD, OUTPUT);
    pinMode(PIN_MTR1_DIR_REV, OUTPUT);
    pinMode(PIN_MTR2_DIR_FWD, OUTPUT);
    pinMode(PIN_MTR2_DIR_REV, OUTPUT);
    digitalWrite(PIN_MTR1_DIR_FWD, LOW);
    digitalWrite(PIN_MTR1_DIR_REV, LOW);
    digitalWrite(PIN_MTR2_DIR_FWD, LOW);
    digitalWrite(PIN_MTR2_DIR_REV, LOW);

    int speedAccel = 100;
    int speedMin = SPEED_MIN;
    mtrLeft.setParams(speedAccel, speedMin, PIN_MTR1_PWM);
    mtrRight.setParams(speedAccel, speedMin, PIN_MTR2_PWM);

    initDisplay();

    loadCommandQueue();

    usLast = micros();
}

void loop()
{
    // this block calculates the number microseconds since this function's last execution
    auto current = micros();
    auto usecElapsed = current - usLast;
    usLast = current;
    if (usecElapsed > usScanLong)
        usScanLong = usecElapsed;
    timerusScan += usecElapsed;
    scanCount++;
    if (timerusScan > 1000000) {
        usScanAvg = timerusScan / scanCount;
        timerusScan = 0;
        scanCount = 0;
        usLongResetCount++;
        if (usLongResetCount > 10) {
            usScanLong = 0;
            usLongResetCount = 0;
        }
    }

    // update motor speed and status
    mtrLeft.updateMotion(usecElapsed);
    mtrRight.updateMotion(usecElapsed);

    // updates the display every 200000us or 0.2 seconds
    if (msTimerPrint > 200000) {
        updateDisplay();
        msTimerPrint = 0;
    }
    msTimerPrint += usecElapsed;

    int newCmd = false;
    if (cmdQueue.firstScan()) {
        newCmd = true;
        // Serial.print(F("New Vehicle Cmd = "));
        // Serial.println(cmdQueue.current());
    }

    int pbStart = !digitalRead(PIN_PB_START);

    if (pbStart) {
        timerPBStartOn += usecElapsed;
        timerPBStartOff = 0;
    } else {
        timerPBStartOn = 0;
        timerPBStartOff += usecElapsed;
    }

    if (cmdQueue.current() > VEHICLE_START && cmdQueue.current() < VEHICLE_ABORT) {
        if (timerPBStartOn > 100000) {
            cmdQueue = {};
            cmdQueue.add(VEHICLE_ABORT);
            mtrLeft.stop();
            mtrRight.stop();
            setMotorDirection();
        }
    }

    if (flagTimeRun)
        timerRunTime += usecElapsed;

    switch (cmdQueue.current()) {
    case VEHICLE_START_WAIT:
        if (timerPBStartOn > 100000)
            cmdQueue.next();
        break;
    case VEHICLE_START:
        timerRunTime = 0;
        if (timerPBStartOff > 100000) {
            cmdQueue.next();
            flagTimeRun = true;
        }
        break;
    case VEHICLE_FORWARD:
        if (newCmd) {
            long distance = cmdQueue.getParameter1() * PULSES_PER_MM;
            mtrLeft.startMove(distance, speedFwd);
            mtrRight.startMove(distance, speedFwd);
            setMotorDirection();
        }

        if (mtrLeft.isStopped() && mtrRight.isStopped()) {
            setMotorDirection();
            cmdQueue.next();
        }
        break;
    case VEHICLE_TURN_RIGHT:
        if (newCmd) {
            mtrLeft.startMove(ENCODER_COUNTS_90_DEG, speedTurn);
            mtrRight.startMove(ENCODER_COUNTS_90_DEG, -speedTurn);
            setMotorDirection();
        }

        if (mtrLeft.isStopped() && mtrRight.isStopped()) {
            setMotorDirection();
            cmdQueue.next();
        }
        break;
    case VEHICLE_TURN_LEFT:
        if (newCmd) {
            mtrLeft.startMove(ENCODER_COUNTS_90_DEG, -speedTurn);
            mtrRight.startMove(ENCODER_COUNTS_90_DEG, speedTurn);
            setMotorDirection();
        }

        if (mtrLeft.isStopped() && mtrRight.isStopped()) {
            setMotorDirection();
            cmdQueue.next();
        }
        break;
    case VEHICLE_SET_MOVE_SPEED:
        speedFwd = cmdQueue.getParameter1();
        cmdQueue.next();
        break;
    case VEHICLE_SET_TURN_SPEED:
        speedTurn = cmdQueue.getParameter1();
        cmdQueue.next();
        break;
    case VEHICLE_SET_ACCEL:
        mtrLeft.setAccel(cmdQueue.getParameter1());
        mtrRight.setAccel(cmdQueue.getParameter1());
        cmdQueue.next();
        break;
    case VEHICLE_FINISHED:
    default:
        if (newCmd) {
            mtrLeft.stop();
            mtrRight.stop();
            setMotorDirection();
        }
        flagTimeRun = false;
        break;
    case VEHICLE_ABORT:
        mtrLeft.stop();
        mtrRight.stop();
        setMotorDirection();
        flagTimeRun = false;
        if (timerPBStartOff > 200000) {
            loadCommandQueue();
        }
        break;
    }
}
