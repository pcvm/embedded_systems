The easiest way to use NewliquidCrystal_1.3.4.zip is to

1) delete the existing LiquidCrystal library in your system, and

2) extract the contents of NewliquidCrystal_1.3.4.zip into a directory/folder
   renamed to LiquidCrystal.

Original source is
   https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home

--------------------

It is a derivate of the original LiquidCrystal Library as sourced in the
Arduino SDK. It has been developed to be compatible with the current
LiquidCrystal library, its performance is almost 5 times faster and fully
extendable if need be.

Being faster, gives your applications more time to do more things than just
controlling the LCD. So, its cool, you can do more stuff.

It supports most Hitachi HD44780 based LCDs, or compatible, connected to
any project using: 4, 8 wire parallel interface, I2C IO port expander and
Shift Regiter.

It currently supports 4 types of connections:
. 4 bit parallel LCD interface
. 8 bit parallel LCD interface
. I2C IO bus expansion board with the PCF8574* I2C IO expander ASIC such
  as I2C LCD extra IO.
. ShiftRegister adaptor board as described Shift Register project home or
  in the HW configuration described below, 2 and 3 wire configurations supported.
. ShiftRegister 3 wire latch adaptor board as described ShiftRegister 3 Wire Home
. Support for 1 wire shift register ShiftRegister 1 Wire

