# Introduction

Source code and PCB design files for an NFC-enabled sweat sensor.  Published as
part of ["Stretchable and biodegradable batteries with high energy and power
density"](https://doi.org/10.1002/adma.202204457).

# Hardware

The board has been designed in KiCad. You can use it generate your own Gerbers.

Here's a shortened BOM:

* MCU: ATtiny85V
* NFC tag: ST25DV64K

# Software

Two build systems can be used:

* PlatformIO
* Traditional Makefile using standard toolchain (avr-gcc, avrdude)

I recommend an SOIC-8 clip for flashing.
