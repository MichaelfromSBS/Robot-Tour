// Calculation for a single turn:
// t_acc = V / A
// d_acc = A * t_acc^2 (accel + decel distance)
// d_tot = PULSES_90_DEG [* 2] (depending on 90 or 180 degrees turn)
// d_const = d_tot - d_acc
// t_const = d_const / V
// t_single_turn = t_acc * 2 + t_const
//
// Then solve for moving speed (v unknown):
// d_tot = d_tot_mm * PULSES_PER_MM
// t_acc = v / A
// d_acc = A * t_acc^2 = v^2 / A (accel + decel distance)
// d_const = d_i - d_acc = d_i - v^2 / A
// t_const = d_const / v
//         = (d_i - v^2 / A) / v
//         = d_i / v - v / A
// t_move = (t_acc * 2 + t_const) + ...
//        = 2 * v * N / A + d_tot / v - v * N / A
//        = d_tot / v + v * N / A <─┐
//                                  |
// t_move = t_tot - t_turn <────────┘

#include <Adafruit_BNO055.h>
#include <LiquidCrystal_I2C.h>

#define PIN_MTR1_ENCA    2
#define PIN_MTR2_ENCA    3
#define PIN_PB_START     4
#define PIN_MTR1_DIR_FWD 5
#define PIN_MTR1_DIR_REV 6
#define PIN_MTR2_DIR_FWD 7
#define PIN_MTR2_DIR_REV 8
#define PIN_MTR1_PWM     9
#define PIN_MTR2_PWM     10

#define PULSES_PER_MM             2.278
#define PULSES_90_DEG             313
#define SPEED_MIN                 120
#define MAX_COMMANDS              60
#define MOTION_UPDATE_INTERVAL_US 30000

#if 0
    #define ASSERT(e)                              \
        if (!(e)) {                                \
            display.clear();                       \
            display.setCursor(0, 0);               \
            display.print(F("ASSERTION FAILED:")); \
            display.setCursor(0, 1);               \
            display.print(F(#e));                  \
            display.setCursor(0, 2);               \
            display.print(__LINE__);               \
            abort();                               \
        }
#else
    #define ASSERT(e) \
        if (!(e))     \
            abort();
#endif

namespace {

// LiquidCrystal_I2C display { 0x27, 20, 4 };

constexpr size_t NUM_SEGMENTS = 1;

struct Pose {
    double x = 0,
           y = 0,
           heading = 0;
};

struct Segment {
    enum class Type {
        Straight,
        Turn90,
        Turn180
    };

    Type type;
    double distance;
};

using SegmentSequence = Segment[NUM_SEGMENTS];

struct DriveSignal {
};

struct PIDCoefficients {
    double p, i, d;
};

// use two pids for pose error -> velocity (linear and angular)
// use existing one for velocity -> wheel speeds

class PIDController {
public:
    PIDController(PIDCoefficients coeffs)
        : m_coeffs { coeffs }
    {
    }

private:
    PIDCoefficients m_coeffs;
};

class Localizer {
public:
    void init()
    {
        pinMode(PIN_MTR1_ENCA, INPUT_PULLUP);
        pinMode(PIN_MTR2_ENCA, INPUT_PULLUP);

        attachInterrupt(
            digitalPinToInterrupt(PIN_MTR1_ENCA), [] { ++s_encoder_count_left; }, RISING);

        attachInterrupt(
            digitalPinToInterrupt(PIN_MTR2_ENCA), [] { ++s_encoder_count_right; }, RISING);

        auto success = m_imu.begin();
        ASSERT(success);
        m_imu.setExtCrystalUse(true);
    }

    void reset()
    {
        m_pose_estimate = {};
        m_heading_offset = -get_raw_heading();
    }

    void update()
    {
        noInterrupts();
        auto count_left = s_encoder_count_left;
        auto count_right = s_encoder_count_right;
        interrupts();

        auto time = static_cast<double>(micros()) / 1000000;

        auto position_left = ticks_to_mm(count_left);
        auto position_right = ticks_to_mm(count_right);
        auto heading = get_raw_heading() + m_heading_offset;

        if (!m_is_first_update) {
            auto delta_left = position_left - m_previous_position_left;
            auto delta_right = position_right - m_previous_position_right;
            auto delta_time = time - m_previous_time;

            auto wheel_to_field = [heading = m_pose_estimate.heading](double left, double right, double& x, double& y) {
                auto avg = (left + right) / 2;
                x = avg * cos(heading);
                y = avg * sin(heading);
            };

            double dx, dy;
            wheel_to_field(delta_left, delta_right, dx, dy);
            m_pose_estimate.x += dx;
            m_pose_estimate.y += dy;
            m_pose_estimate.heading = heading;

            auto velocity_left = delta_left / delta_time;
            auto velocity_right = delta_right / delta_time;
            wheel_to_field(velocity_left, velocity_right, m_velocity_estimate.x, m_velocity_estimate.y);
            m_velocity_estimate.heading = get_angular_velocity();
        } else {
            m_is_first_update = false;
        }

        m_previous_position_left = position_left;
        m_previous_position_right = position_right;
        m_previous_time = time;
    }

    const Pose& get_pose_estimate()
    {
        return m_pose_estimate;
    }

    const Pose& get_velocity_estimate()
    {
        return m_velocity_estimate;
    }

    auto get_euler()
    {
        return m_imu.getVector(Adafruit_BNO055::VECTOR_EULER);
    }

private:
    double get_raw_heading()
    {
        return m_imu.getVector(Adafruit_BNO055::VECTOR_EULER).z();
    }

    double get_angular_velocity()
    {
        return m_imu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE).z();
    }

    double ticks_to_mm(unsigned long ticks)
    {
        return WHEEL_RADIUS * TWO_PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    static constexpr double WHEEL_RADIUS = 37.5,
                            GEAR_RATIO = 1,
                            TICKS_PER_REV = 540;
    static inline volatile unsigned long s_encoder_count_left = 0,
                                         s_encoder_count_right = 0;
    Adafruit_BNO055 m_imu;
    Pose m_pose_estimate,
        m_velocity_estimate;
    double m_previous_position_left = 0,
           m_previous_position_right = 0;
    double m_previous_time = 0;
    bool m_is_first_update = true;
    double m_heading_offset = 0;
};

class Drive {
public:
    Drive()
    {
        pinMode(PIN_MTR1_PWM, OUTPUT);
        pinMode(PIN_MTR2_PWM, OUTPUT);

        pinMode(PIN_MTR1_DIR_FWD, OUTPUT);
        pinMode(PIN_MTR1_DIR_REV, OUTPUT);
        pinMode(PIN_MTR2_DIR_FWD, OUTPUT);
        pinMode(PIN_MTR2_DIR_REV, OUTPUT);

        digitalWrite(PIN_MTR1_DIR_FWD, LOW);
        digitalWrite(PIN_MTR1_DIR_REV, LOW);
        digitalWrite(PIN_MTR2_DIR_FWD, LOW);
        digitalWrite(PIN_MTR2_DIR_REV, LOW);
    }

    void follow_segment_sequence(const SegmentSequence& seq)
    {
        m_current_sequence = &seq;
        m_current_segment_index = 0;
        m_last_segment_index = -1;
    }

    void update()
    {
        m_localizer.update();
        if (m_current_sequence) {
            auto drive_signal = update_drive();
            set_drive_signal(drive_signal);
        }
    }

    const Pose& get_pose_estimate()
    {
        return m_localizer.get_pose_estimate();
    }

private:
    DriveSignal update_drive()
    {
        if (m_current_segment_index >= NUM_SEGMENTS) {
            m_current_sequence = nullptr;
            return {};
        }

        auto time = static_cast<double>(micros()) / 1000000;
        auto is_new_segment = m_current_segment_index != m_last_segment_index;
        if (is_new_segment) {
            m_current_segment_start_time = time;
            m_last_segment_index = m_current_segment_index;
        }

        auto delta_time = time - m_current_segment_start_time;

        const auto& current_segment = (*m_current_sequence)[m_current_segment_index];
        switch (current_segment.type) {
        case Segment::Type::Straight:
            break;
        case Segment::Type::Turn90:
            break;
        case Segment::Type::Turn180:
            break;
        default:
            ASSERT(!"Unreachable");
        }
    }

    void set_drive_signal(const DriveSignal& drive_signal)
    {
        (void)drive_signal;
    }

    void set_motor_powers(double left, double right)
    {
        if (left >= 0) {
            digitalWrite(PIN_MTR1_DIR_FWD, HIGH);
            digitalWrite(PIN_MTR1_DIR_REV, LOW);
        } else {
            digitalWrite(PIN_MTR1_DIR_FWD, LOW);
            digitalWrite(PIN_MTR1_DIR_REV, HIGH);
        }

        if (right >= 0) {
            digitalWrite(PIN_MTR2_DIR_FWD, HIGH);
            digitalWrite(PIN_MTR2_DIR_REV, LOW);
        } else {
            digitalWrite(PIN_MTR2_DIR_FWD, LOW);
            digitalWrite(PIN_MTR2_DIR_REV, HIGH);
        }

        analogWrite(PIN_MTR1_PWM, left * 255);
        analogWrite(PIN_MTR2_PWM, right * 255);
    }

    Localizer m_localizer;
    const SegmentSequence* m_current_sequence = nullptr;
    size_t m_current_segment_index = 0;
    size_t m_last_segment_index = -1;
    double m_current_segment_start_time = 0;
};

enum {
    VEHICLE_START_WAIT = 1,
    VEHICLE_START,
    VEHICLE_FORWARD,
    VEHICLE_TURN_RIGHT,
    VEHICLE_TURN_LEFT,
    VEHICLE_TURN_180,
    VEHICLE_SET_MOVE_SPEED,
    VEHICLE_SET_TURN_SPEED,
    VEHICLE_SET_ACCEL,
    VEHICLE_FINISHED,
    VEHICLE_ABORT
};

class CommandQueue {
public:
    struct Command {
        int cmd;
        int param;
    };

    int currentCmd() { return list[start].cmd; }
    int currentParam() { return list[start].param; }

    bool firstScan()
    {
        auto saved = isFirstScan;
        isFirstScan = false;
        return saved;
    }

    void add(int cmd, int param = 0)
    {
        ASSERT(end < MAX_COMMANDS);
        list[end++] = { cmd, param };
    }

    void next()
    {
        isFirstScan = true;
        if (start == end)
            return;
        ++start;
    }

    void clear()
    {
        start = 0;
        end = 0;
    }

    void load()
    {
        clear();
        add(VEHICLE_START_WAIT); // Do not change
        add(VEHICLE_START);      // Do not change

        add(VEHICLE_SET_MOVE_SPEED, 500); // Calculate
        add(VEHICLE_SET_TURN_SPEED, 200); // Keep constant; use low turn speed for consistency
        add(VEHICLE_SET_ACCEL, 400);      // Keep constant

        // Distance between dowel and center of robot: 75mm
        // Define robot movement below
        // add(...);

        add(VEHICLE_FINISHED); // This MUST be the last command
    }

private:
    Command list[MAX_COMMANDS];
    size_t start = 0;
    size_t end = 0;
    bool isFirstScan = true;
};

class MotionLogic {
public:
    explicit MotionLogic(int pwmPin)
        : pinPWM { pwmPin }
    {
    }

    bool isForward() { return forward; }
    bool isStopped() { return !running; }

    void incrEncoder() { ++countEncoder; }

    void enableDebug() { debug = true; }
    bool getDebug() { return debug; }

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

        float tAccel = (float)speedTarget / accelRate;
        float tDecel = (float)speedTarget / decelRate;
        float distAccel = (float)speedTarget / 2.0 * tAccel;
        float distDecel = (float)speedTarget / 2.0 * tDecel;
        float distAtSpeed = (float)positionTarget - distAccel - distDecel;
#if 1
        if (distAtSpeed < 0.0) {
            // Serial.println("Motion - Short move logic");
            distAccel = (float)positionTarget / 2.0;
            distDecel = (float)positionTarget / 2.0;
            distAtSpeed = 0.0;

            tAccel = sqrt(distAccel * 2.0 / accelRate);
            tDecel = sqrt(distDecel * 2.0 / decelRate);
        }
#else
        ASSERT(distAtSpeed > 0);
#endif
        float tAtSpeed = distAtSpeed / speedTarget;

        timeAtSpeed = tAccel * 1000000;
        timeDecel = timeAtSpeed + tAtSpeed * 1000000;

        pwmLoopI = 0;
        pwmLoopP = 0;
        countEncoder = 0;
        countEncoderLast = 0;
        timeRunning = 0;
        timerUpdate = 0;
        running = true;
    }

    void stop()
    {
        forward = true;
        analogWrite(pinPWM, 0);
        running = false;
        speedTarget = 0;
    }

    void updateMotion(long usecElapsed)
    {
        if (!running) {
            forward = true;
            pwmLoopP = 0;
            pwmLoopI = 0;
            return;
        }

        timeRunning += usecElapsed;
        timerUpdate += usecElapsed;

        int speedActual; // p/s
        if (timerUpdate >= MOTION_UPDATE_INTERVAL_US) {
            auto delta = countEncoder - countEncoderLast;
            speedActual = delta * 1000000 / timerUpdate;
            countEncoderLast = countEncoder;
            timerUpdate = 0;
        } else {
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

        int speedProfile; // p/s
        if (timeRunning < timeAtSpeed) {
            speedProfile = (float)timeRunning / 1000000.0 * accelRate;
        } else if (timeRunning < timeDecel) {
            speedProfile = speedTarget;
        } else {
            speedProfile = speedTarget - (float)(timeRunning - timeDecel) / 1000000.0 * decelRate;
            if (speedProfile < speedMinimum)
                speedProfile = speedMinimum;
        }

        int speedError = speedProfile - speedActual;

        // This program uses a PI loop to control speed
        // This loop is NOT tuned.  Meaning testing is required to tune the loop
        // which will provide the best performance
        pwmLoopI += speedError / 4;
        pwmLoopP = speedError / 2;

        auto outputPWM = constrain(pwmLoopP + pwmLoopI, 0, 254);

        if (debug) {
            Serial.print("timeRunning:");
            Serial.print(timeRunning);
            Serial.print(",encoder:");
            Serial.print(countEncoder);
            Serial.print(",speedProfile:");
            Serial.print(speedProfile);
            Serial.print(",speedActual:");
            Serial.print(speedActual);
            Serial.print(",speedError:");
            Serial.print(speedError);
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
    long timeAtSpeed = 0; // us
    long timeDecel = 0;   // us

    long timeRunning = 0; // us
    long timerUpdate = 0; // us

    int pinPWM;
    bool forward = true;
    bool running = false;

    int pwmLoopI = 0;
    int pwmLoopP = 0;

    bool debug = false;

    long countEncoder = 0;
    long countEncoderLast = 0;

    long accelRate = 100; // p/s^2
    long decelRate = 100; // p/s^2

    long positionTarget = 0;      // p
    int speedTarget = 0;          // p/s
    int speedMinimum = SPEED_MIN; // p/s
};

unsigned long usLast = 0;
CommandQueue cmdQueue;
MotionLogic mtrLeft { PIN_MTR1_PWM };
MotionLogic mtrRight { PIN_MTR2_PWM };

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

void initPins()
{
    pinMode(PIN_PB_START, INPUT_PULLUP);
}

void initDisplay()
{
#if 0
    display.init();
    display.backlight();

    display.clear();
    display.setCursor(0, 0);
    display.print(F("Cmd:"));

    display.setCursor(0, 3);
    display.print(F("Time:"));
#endif
}

void updateDisplay(unsigned long time)
{
#if 0
    static int lastCmd = 0;

    auto cmd = cmdQueue.currentCmd();
    if (lastCmd != cmd) {
        display.setCursor(5, 0);
        switch (cmd) {
        case VEHICLE_START_WAIT:
            display.print(F("WAIT START  "));
            break;
        case VEHICLE_START:
            display.print(F("WAIT RELEASE"));
            break;
        case VEHICLE_FORWARD:
            display.print(F("FORWARD     "));
            break;
        case VEHICLE_TURN_RIGHT:
            display.print(F("TURN RIGHT  "));
            break;
        case VEHICLE_TURN_LEFT:
            display.print(F("TURN LEFT   "));
            break;
        case VEHICLE_TURN_180:
            display.print(F("TURN 180    "));
            break;
        case VEHICLE_FINISHED:
            display.print(F("FINISHED    "));
            break;
        case VEHICLE_ABORT:
            display.print(F("ABORT       "));
            break;
        default:
            ASSERT(!"Unreachable");
        }
        lastCmd = cmd;
    }

    display.setCursor(6, 3);
    display.print((float)time / 1000000.0, 3);
#endif
}

Localizer s_localizer;
}

#if 1
void setup()
{
    Serial.begin(115200);
    s_localizer.init();
}

void loop()
{
    static int counter = 0;

    s_localizer.update();
    if (++counter >= 100) {
        auto pose = s_localizer.get_euler();
        const auto& velocity = s_localizer.get_velocity_estimate();
        Serial.print(pose.x());
        Serial.print(", ");
        Serial.print(pose.y());
        Serial.print(", ");
        Serial.print(pose.z());
        Serial.print("; ");
        Serial.print(velocity.x);
        Serial.print(", ");
        Serial.print(velocity.y);
        Serial.print(", ");
        Serial.println(velocity.heading);
        counter = 0;
    }

    delay(100);
}
#else
void setup()
{
    // Only uncomment one motor at a time to use the Serial Plotter function to tune the PID loop
    // mtrLeft.enableDebug();
    // mtrRight.enableDebug();

    if (mtrLeft.getDebug() || mtrRight.getDebug()) {
        Serial.begin(115200);
        Serial.println(F("Setup()..."));
    }

    initPins();
    initDisplay();

    cmdQueue.load();

    usLast = micros();
}

void loop()
{
    static int speedFwd = 100;
    static int speedTurn = 100;
    static bool timerRunning = false;

    static unsigned long timerRunTime = 0;
    static unsigned long msTimerPrint = 0;
    static unsigned long timerPBStartOn = 0;
    static unsigned long timerPBStartOff = 0;

    auto current = micros();
    auto usecElapsed = current - usLast;
    usLast = current;

    mtrLeft.updateMotion(usecElapsed);
    mtrRight.updateMotion(usecElapsed);

    if (msTimerPrint > 200000) {
        updateDisplay(timerRunTime);
        msTimerPrint = 0;
    }
    msTimerPrint += usecElapsed;

    if (!digitalRead(PIN_PB_START)) {
        timerPBStartOn += usecElapsed;
        timerPBStartOff = 0;
    } else {
        timerPBStartOn = 0;
        timerPBStartOff += usecElapsed;
    }

    auto newCmd = cmdQueue.firstScan();
    auto cmd = cmdQueue.currentCmd();

    if (cmd > VEHICLE_START && cmd < VEHICLE_ABORT) {
        if (timerPBStartOn > 100000) {
            cmdQueue.clear();
            cmdQueue.add(VEHICLE_ABORT);
            return;
        }
    }

    if (timerRunning)
        timerRunTime += usecElapsed;

    switch (cmd) {
    case VEHICLE_START_WAIT:
        if (timerPBStartOn > 100000)
            cmdQueue.next();
        break;
    case VEHICLE_START:
        if (timerPBStartOff > 100000) {
            cmdQueue.next();
            timerRunning = true;
            timerRunTime = 0;
        }
        break;
    case VEHICLE_FORWARD:
        if (newCmd) {
            long distance = cmdQueue.currentParam() * PULSES_PER_MM;
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
            mtrLeft.startMove(PULSES_90_DEG, speedTurn);
            mtrRight.startMove(PULSES_90_DEG, -speedTurn);
            setMotorDirection();
        }

        if (mtrLeft.isStopped() && mtrRight.isStopped()) {
            setMotorDirection();
            cmdQueue.next();
        }
        break;
    case VEHICLE_TURN_LEFT:
        if (newCmd) {
            mtrLeft.startMove(PULSES_90_DEG, -speedTurn);
            mtrRight.startMove(PULSES_90_DEG, speedTurn);
            setMotorDirection();
        }

        if (mtrLeft.isStopped() && mtrRight.isStopped()) {
            setMotorDirection();
            cmdQueue.next();
        }
        break;
    case VEHICLE_TURN_180:
        if (newCmd) {
            mtrLeft.startMove(PULSES_90_DEG * 2, speedTurn);
            mtrRight.startMove(PULSES_90_DEG * 2, -speedTurn);
            setMotorDirection();
        }

        if (mtrLeft.isStopped() && mtrRight.isStopped()) {
            setMotorDirection();
            cmdQueue.next();
        }
        break;
    case VEHICLE_SET_MOVE_SPEED:
        speedFwd = cmdQueue.currentParam();
        cmdQueue.next();
        break;
    case VEHICLE_SET_TURN_SPEED:
        speedTurn = cmdQueue.currentParam();
        cmdQueue.next();
        break;
    case VEHICLE_SET_ACCEL:
        mtrLeft.setAccel(cmdQueue.currentParam());
        mtrRight.setAccel(cmdQueue.currentParam());
        cmdQueue.next();
        break;
    case VEHICLE_FINISHED:
        if (newCmd) {
            mtrLeft.stop();
            mtrRight.stop();
            setMotorDirection();
            timerRunning = false;
        }
        break;
    case VEHICLE_ABORT:
        mtrLeft.stop();
        mtrRight.stop();
        setMotorDirection();
        timerRunning = false;
        if (timerPBStartOff > 200000)
            cmdQueue.load();
        break;
    default:
        ASSERT(!"Unreachable");
    }
}
#endif
