Why this fork?
===============

This fork aims to make the Blinkstick.com firmware run on the
[Digispark](http://digistump.com/products/1) USB development board
with additional color pattern capabilities and auto shut-off.

With the stock firmware if you want to display a light pattern
that takes place over a few seconds like a gradual fade from one color to
another, the host CPU is responsible for rendering the effect all the while
blocking itself with sleep commands and continuous data transfer to the Digistump.

This fork allows the host CPU to upload in one batch up to 62 colors with durations so
the Digistump can render the pattern leaving the host CPU free for other work.

Use Case
========

Use this fork if you want to use your Digistump as a status or activity indicator for events
like CPU temperature, HDD activity, upload activity, DB activity, etc. I designed the
IO to never block or wait, be instantly responsive, but if the device is busy
rendering a pattern new color/pattern requests are silently ignored. This design works in
systems that use continual status updates or heartbeat indications.

The LED will also shut off then blink after every 60s of inactivity. For example, if the last
status color is green, but then the host goes away, the LED would give a false sense of status.
If the LED is off and periodically blinking like a smoke detector with a low battery,
then you will know something has gone wrong.

Capabilities
============

It is possible to render a color pattern lasting up to 158 seconds in one single command. Up to 62
color segments may be uploaded at one time. Each color segment is 4 bytes with the first three bytes
representing the colors red, green, and blue, and the fourth byte the delay in 1/10th milliseconds. Since the
fourth byte can only range from 0-255, this is multiplied by 10 to give a delay range from 0-2550 ms with
a 10 ms resolution.

My favorite pattern is a gradual pulse that increases in intensity for 30 steps then fades away over another 30 steps,
all with a single USB set-and-forget command.

Data Format
==========

To simplify this design, only reports 1 (set a simple color), and 20 (set a color pattern), are used. The payload format
for report 20 is:

    report id + color count + ((r, g, b, delay) * color count)

Example:

    [20, 2, 255, 0, 0, 50, 0, 0, 0, 0]

This will turn on the LED to red for 500 ms then turn the LED off. Then

    [20, 2, (255, 0, 0), 50, (0, 0, 0), 0]

can be interpreted as

    [(report 20), (2 colors), (red), for 500 ms, (black), forever]

BlinkStick Firmware modified
===================

2019/06/29 - Eric Draken (ericdraken.com)

The following is a list of modifications on this branch.

1. Removed the code for an RGB LED
2. Set USB descriptors in main.cpp
3. Added an inactivity timer to turn off the LED
4. Exclusivity for the Digistump ATTiny85 (and clones)
5. Color patterns can be uploaded to alleviate the host CPU
6. LED shuts off after 60s of inactivity
7. LED blinks every 60s of inactivity

### Compile and upload firmware

You need to install the Arduino tools as described in the
[Digispark tutorial: 'Connecting and Programming Your Digispark'](http://digistump.com/wiki/digispark/tutorials/connecting) first.

````
make clean
make hex
/opt/arduino-1.5.8-64bit/hardware/digistump/avr/tools/avrdude -cdigispark --timeout 30 -Uflash:w:main.hex:i
````

### Images

Here is the setup used to develop and test this firmware.

![Digispark USB with custom soldered ADA106 LED](/pictures/attiny85-front.jpg)
![Digispark parts placement](/pictures/attiny85-back.jpg)

BlinkStick Firmware
===================

BlinkStick is a DIY open source USB RGB LED. It's small, the 
size of a USB flash stick and designed to be easy to assemble. 

You can find more details about it here:

http://www.blinkstick.com

This repository contains the firmware required for the ATTiny85 chip.

Details about setting up the development environment will be released soon.


Original License
================

Released under CC-BY-NC-SA 3.0 license:

http://creativecommons.org/licenses/by-nc-sa/3.0/

(c) 2013 by Agile Innovative Ltd
