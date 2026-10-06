# ESP32-Stepper-DMX-board

## DMX Channels

The current firmware implements the real motor + LED-dimmer DMX profile. There is no WLED DMX implementation in this build.

Combined fixture profile (LED dimmer default start address = 8):

1   - LED Pan
2   - LED Pan fine
3   - Ref Pan
4   - Ref Pan fine
5   - LED Pan Inv (0 stop / 1-127 CCW / 128 stop / 129-255 CW)
6   - Ref Pan Inv (0 stop / 1-127 CCW / 128 stop / 129-255 CW)
7   - Reset / Homing (0 no / 1-255 start)
8   - Dimmer 0 coarse
9   - Dimmer 0 fine
10  - Dimmer 1 coarse
11  - Dimmer 1 fine
12  - Dimmer 2 coarse
13  - Dimmer 2 fine
14  - Dimmer 3 coarse
15  - Dimmer 3 fine
16  - Dimmer 4 coarse
17  - Dimmer 4 fine
18  - Dimmer 5 coarse
19  - Dimmer 5 fine
20  - Dimmer 6 coarse
21  - Dimmer 6 fine
22  - Dimmer 7 coarse
23  - Dimmer 7 fine
24  - Strobe (0 off / 1-255 speed)
25-240 - RGB pixel data for 72 LEDs (3 channels per LED)

## Connector Pinout
1 - GND
2 - +24V
3 - DMX_B (xlr pin 2)
4 - DMX_A (xlr pin 3)


