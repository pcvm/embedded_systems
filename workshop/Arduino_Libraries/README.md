# Arduino_Libraries

These libraries were used for the AVR microcontroller workshop using the Arduino environment:

* MsTimer2.zip provides timer2 overflow interrupts for timekeeping
   - Set up via the usual #include <MsTimer2.h>
   - Allows us to write interrupt_timer2overflow()

* NewliquidCrystal_1.3.4.zip provides a replacement LiquidCrystal library that supports LCD communication via I2C and a PCF8574* I2C IO expander. To install:
   - delete the existing LiquidCrystal library in your system;
   - extract the contents of NewliquidCrystal_1.3.4.zip into a directory/folder renamed to LiquidCrystal.
   - Original source is https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
   - See readme-new-liquidcrystal.txt an overview.
