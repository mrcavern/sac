sac
===

Arduino Irrigation and Clima Control System

As shipped the project builds a simulator of the SAC for running inside a
terminal emulator. The terminal handling library nchanterm is included with
the source - thus no further dependencies are neccesary.

To build for arduino, sac.c and sac.h needs to be added to a sketch - and the
ARDUINO_MODE define at the top must be changed to 1.

Sac depends on the following arduino libraries:

TimerThree, EEPROM, SerialLCD and Software Serial.

