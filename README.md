# annet-cleft

_Cleft_ is a sculpture about 8' around. This project is meant to animate it, eventually responding to participants.

It has 15 segments to move. So, 15 stepper motors.
 
This project has code and electronics for the control-system.

We use a fast arduino (samd21 48MHz Zero/adafruuit-metro-m0) to get through all the floating point math in the stepper-library. Movement of the sculpture is slow, so we need less than 2 revolutions per second (<400 steps/sec). We also want more RAM for animations.

# Fritzing (wiring/electronics)

## fritzing/perf-shift-register.fzz

A perfboard layout of the current design:

* Includes place holders for stepper-driver, limit-switch, 24v power. But, those are not actual part-numbers, just place-holders.
* 1 output shift register
* 1 input shift register
* cat6 cabling
* screw-blocks for motor/limit-switch
* Controls 4 motors/limit-switches each.

We'll need 4 of these.

Plus, an arduino-to-cat6 adapter, and a final shift-out to led-bar for indicator.

The assembled system is: arduino -> perfboard -> perfboard... -> perboard w/led-bar. Going around the 8' circle (so about 24' long system).

# Arduino code

Various arduino projects in `arduino/`.

The projects share several files. Unfortunately, Windows doesn't do relative-shortcuts, and you need privileges for NTFS-links.

Thus, the same file appears to be repeated in the arduino/ projects. But, it is supposed to be the same file.

Requiring installation of a library for our common files is a complication (and synchronization issue).

So:

* Manually keep files identical
* The files are arduino/*/*.h, that appear in more than 1 directory.
* If you edit one of those, you'll need to copy it to the other places (and make sure they still work there).
* Bummer.

### 

We use SPI for talking to the shift registers because it is fast. A `.transfer(bytearray)` will write to the shift-out and simultaneously read from the shift-in. Note the stepper protocol: send the `direction` bit (w/step==low), send the leading edge of `step` (high for step, low for don't move), send the trailing-edge of `step` == low. That's 3 states (3 .transfer's), each requiring a latch pulse. Shift-in requires a latch-pulse _before_ reading.

## cleft/

The performance code. Interprets sensors, decides on animation, runs motors.

More info in `cleft.ino`.

## spi_test/

Several different modes to test the shift-in/out register setup. Use with a test jig of leds, and/or stepper-driver+motor.

See `spi_test.ino`, comments, constants, and the `loop()`.

## resource_test/

Test code for evaluating the use of arduino resources by the systems in cleft/. I used this to prove that we have enough RAM, and that each sub-system is fast enough.

# Processing code

In `processing/` :

## visualization/

A visualization of cleft. 

You can manually move each segment.

It also knows how to read the output from arduino/cleft/
and show it as a visualization of the animiation.

Uses a real simple protocol to talk to the arduino over the serial-port.
