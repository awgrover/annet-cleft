# annet-cleft

code and design

# Arduino code

Various arduino projects in arduino/

The projects share several files. Unfortunately, Windows doesn't do relative-shortcuts, and you need privileges for NTFS-links.

Thus, the same file appears to be repeated in the arduino/ projects. But, it is supposed to be the same file.

Requiring installation of a library for our common files is a complication (and synchronization issue).

So:
* Manually keep files identical
* The files are arduino/*/*.h, that appear in more than 1 directory.
* If you edit one of those, you'll need to copy it to the other places (and make sure they still work there).
* Bummer.

## cleft/

The performance code. Interprets sensors, decides on animation, runs motors.

## resources/

Test code for evaluating the use of resources by the systems in cleft/.

# Processing code

In processing/

## visualization/

A visualization of cleft. You can manually move each segment.

It also knows how to read the output from arduino/cleft/ and arduino/animation/
as a visualization of the animiation.

# Fritzing wiring/schematics

To document the electronics.
