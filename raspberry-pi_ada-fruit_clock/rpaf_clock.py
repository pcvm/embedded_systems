#!/usr/bin/python
#
# rpaf_clock.py -- raspberryPi & adaFruit clock
#
# This program is derived from sample code ex_7segment_clock.py as supplied
# by AdaFruit in the Adafruit-Raspberry-Pi-Python-Code-master.zip archive.
#
# Modifications / pmusumeci@gmail
# - replaced some constant numbers in the sample code by variable names
#   chosen to clarify how to address individual digits and access individual
#   LEDs (such as the colon) that adorn the 7-segment display;
# - added a small code fragment to clear the display when hour count rolls
#   back to 0 so that a leading '0' char for the hours is not displayed;
# - added a try/catch exception handler to catch shutdown events and leave
#   the display cleared (except for one LED representing the exception cause).
# To do
# - Probably should use an alarm call via select/kqueue to sleep until the
#   *next* 1 second instance instead of delaying 1 second/loop, which ignores
#   loop processing time.

import time
import datetime
from Adafruit_7Segment import SevenSegment

#
# Define constant address values for accessing digits with writeDigit().
# - The four digits are addressed, left to right, as
#      left-hand-most 1st digit: 0
#                     2nd digit: 1
#                     3rd digit: 3
#     right-hand-most 4th digit: 4
af7s_digit1 = 0
af7s_digit2 = 1
af7s_digit3 = 3
af7s_digit4 = 4
#
# Define constant address values for accessing LEDs using disp.setBufferRow()
# - This function can access the indicator LEDs at row address of 2
af7s_ledRow = 2
# - The colon and indicator LEDs are controlled by the value set with a call
#   to disp.setBufferRow(af7s_ledRow, bitPattern) where bitPattern has bits
#   used as follows:
af7s_FLAGS={ 'blank': 0,
             'colon': 2,
           'topleft': 4,
        'bottomleft': 8,
          'topright':16 }
# - We can now set and clear the colon with calls such as
#   disp.setBufferRow(af7s_ledRow, af7s_FLAGS['colon']) #set colon
#   disp.setBufferRow(af7s_ledRow, af7s_FLAGS['blank']) #clear colon

# ===========================================================================
# Clock Example
# ===========================================================================

# instantiate a display controller at address 0x70 (this is the
# default address when no address selection links are soldered)
#
segment = SevenSegment(address=0x70)

print "Press CTRL+C to exit"
try:

  # Continuously update the time displayed
  # - this endless loop contains a 1 second delay
  # - digit displays are overwritten even if they remain unchanged

  # Note: we need to clear the display at startup, and there is also a
  #       a need to blank out the leading digits of the hours after a
  #       rollover from 23:59 or 11:59 back to zero. For convenience,
  #       keep track of previous_hours and then clear the display when
  #       the next hours is numerically < previous_time.
  previous_hours=25  # ensure display is initially cleared

  while(True):
    now = datetime.datetime.now()                       # load time into integer variables
    hour = int(now.hour)
    htens = hour/10
    minute = int(now.minute)
    second = int(now.second)

    if hour < previous_hours: segment.disp.clear(True)  # clear display when hour resets
    previous_hours = hour

    if htens>0: segment.writeDigit(af7s_digit1, htens)  # set hours tens (if non 0)
    segment.writeDigit(af7s_digit2, hour % 10)          # set hours units

    segment.writeDigit(af7s_digit3, minute / 10)        # set minutes
    segment.writeDigit(af7s_digit4, minute % 10)

    if second % 2 > 0:                                  # set/clear colon (other LEDs off)
      segment.disp.setBufferRow(af7s_ledRow, af7s_FLAGS[ 'colon' ] )
    else:
      segment.disp.setBufferRow(af7s_ledRow, af7s_FLAGS[ 'blank' ] )

    time.sleep(1)                                       # delay for 1 second

                                                        # exit with clear display
except KeyboardInterrupt:                               # with LED indicating event
  segment.disp.clear( True )
  segment.disp.setBufferRow(af7s_ledRow, af7s_FLAGS[ 'topright' ])
  quit()
except:
  segment.disp.clear( True )
  segment.disp.setBufferRow(af7s_ledRow, af7s_FLAGS[ 'topleft' ])
  quit()
