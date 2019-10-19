Why this fork?
===============

This fork aims to make the Blinkstick.com firmware run on the 
[Digispark](http://digistump.com/products/1) USB development board
in combination with **software PWM**.

## Why software PWM? ##

To drive a single RGB-LED a 3 channel PWM is needed. Unfortunately, the 
Digispark pinout does not offer 3 hardware PWM channels when using 
V-USB at the same time. The hardware timer1 output resides on 
Pin PB4 which is already used by USB D- line. Therefore this fork 
uses 3 freely configurable software PWM channels build on timer1.


BlinkStick Firmware
===================

BlinkStick is a DIY open source USB RGB LED. It's small, the 
size of a USB flash stick and designed to be easy to assemble. 

You can find more details about it here:

http://www.blinkstick.com

This repository contains the firmware required for the ATTiny85 chip.

Details about setting up the development environment will be released soon.

License
=======

Released under CC-BY-NC-SA 3.0 license:

http://creativecommons.org/licenses/by-nc-sa/3.0/

(c) 2013 by Agile Innovative Ltd
