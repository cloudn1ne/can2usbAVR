# can2usbAVR

If you want to rebuild form source you need to have the arduino build tools installed. they come with the arduino gui download usually.
If you only want to flash the .hex file to the Arduino avrdude (MacOs/Linux) or Xloader (Windows) will be sufficient.

Hex file
========

https://github.com/t4eeditor/can2usbAVR/tree/master/hex

Flashing firmware (on Mac/Linux via avrdude):
============================

edit the Makefile - setup the correct serial port: 

PROGRAMMER = -C avrdude.conf  -v -V -patmega328p -carduino -P/dev/cu.usbmodem1d1111 -b115200 -D

run "make flash"

alternatively run "make" and upload the .hex file with another tool.


Flashing firmware (on Windows via Xloader):
===============================

see http://www.hobbytronics.co.uk/arduino-xloader

