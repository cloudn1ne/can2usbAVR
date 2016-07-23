# can2usbAVR

Be sure to have the arduino build tools installed. they come with the arduino gui download usually.

Hex file
========

https://github.com/t4eeditor/can2usbAVR/tree/master/hex

Flashing firmware (avrdude):
============================

edit the Makefile - setup the correct serial port: 

PROGRAMMER = -C avrdude.conf  -v -V -patmega328p -carduino -P/dev/cu.usbmodem1d1111 -b115200 -D

run "make flash"

alternatively run "make" and upload the .hex file with another tool.


Flashing firmware (on Windows):
===============================

see http://www.hobbytronics.co.uk/arduino-xloader

