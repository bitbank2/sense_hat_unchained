Sense Hat Unchained
-------------------
A C library to communicate with the LEDs and sensors on the Raspberry Pi
Sense Hat.
![Orange Pi Lite](/animated.gif?raw=true "Sense Hat set free")

Copyright (c) 2017 BitBank Software, Inc.
written by Larry Bank
bitbank@pobox.com

The reason for 'unchained' is that the Sense Hat is often assumed to be an
accessory which only works on RPI hardware with RPI provided software.
In actuality, the Sense Hat is just a collection of I2C devices that can work
with any computer which has an I2C bus. The purpose of this library is to
provide a simple example of how to work with the Sense Hat without needing
any special software. The code here is not complete in that it doesn't expose
all modes/options of the sensors. It does provide a minimal example of how to 
initialize and read data from them. I have tested the Sense Hat on Raspberry
Pi computers (1B/3B/ZeroW) and Orange Pi boards.
