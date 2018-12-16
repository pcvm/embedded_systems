# raspberry-pi_ada-fruit_clock

* rpaf_clock.py
  - Provides an NTP synchronised clock based on a RaspberryPi Zero/A/A+/B/B+ driving an adaFruit https://www.adafruit.com/products/1268 LED display (the LED module uses a HT16K33 controller IC described at https://www.adafruit.com/datasheets/ht16K33v110.pdf).
  - The clock is NTP locked because the underlying OS runs time synchronisation tasks (such as ntpd).
  - This program is derived from sample code ex_7segment_clock.py as supplied by AdaFruit in the Adafruit-Raspberry-Pi-Python-Code-master.zip archive.
  - Modifications:
    - replaced some constant numbers in the sample code by variable names chosen to clarify how to address individual digits and access individual LEDs (such as the colon) that adorn the 7-segment display;
    - added a small code fragment to clear the display when hour count rolls back to 0 so that a leading '0' char for the hours is not displayed;
    - added a try/catch exception handler to catch shutdown events and leave the display cleared (except for one LED representing the exception cause).
  - To do:
    - Probably should use an alarm call via select/kqueue to sleep until the *next* 1 second instance instead of delaying 1 second/loop, which ignores loop processing time (and so very slowly becomes slightly delayed).
  - Files:
    - rpaf_clock.py does simple time keeping and time display (via I2C link to AdaFruit LED);
    - Adafruit_*.py correspond to LED driver code from AdaFruit.
