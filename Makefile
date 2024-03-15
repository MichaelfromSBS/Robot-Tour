SRC := robot.ino
FQBN := arduino:renesas_uno:minima

.PHONY: upload compile

upload: compile
	arduino-cli upload -b $(FQBN) -t

compile: $(SRC)
	arduino-cli compile -b $(FQBN) --build-cache-path ~/Library/Arduino15

monitor:
	arduino-cli monitor -b $(FQBN) -p $(wildcard /dev/tty.usbmodem*)
