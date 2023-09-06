# esp8266_clock_thing2

This is a time-of-day clock demo featuring NTP synchronisation and WiFi access point configuration. It grew (unfortunately) from a code sample set for an embedded systems workshop that changed from Atmel AVR processors to the ESP8266 platform in 2015-2017. The main display for time is an Adafruit I2C connected 4 digit LED, and an additional I2C connected 4x20 LCD can show time as well as optional temperature and humidity (plus status/debug info). The clock works fine with only an LED display but can also handle 4x 8x8 dot matrix displays, and now supports reference time sources { NTP, GPS }.

My blog page for "Phillip's Clock Thing" gives a general introduction at https://pmusumeci.blogspot.com/p/ntp-locked-clock.html and links to a user guide

* Phillip_Clock_Thing2.ino
   - main source code
* display_LCD.h display_LED7seg.h display_LEDdotmatrix.h display_font_data.h display_init.h
   - handles 7-segment LED and LED dot-matrix displays
* sensor_data_io.h
   - handles one-wire and other temperature and humidity sensors
* configure_via_uart_or_telnet.h configure_via_wifi_ap.h
   - handles system configuration
* daytime_retrieval_from_source.h
   - manages a read of UTC from a GPS receiver or manages a state machine to retrieve UTC from an NTP server
* eeprom_config_io.h
   - manages eeprom access to system configuration data
* gps_ss.h
   - handles access to a GPS receiver via a software serial RX port (i.e. timed bit manipulation)
   - handles data extraction from GPS "sentence" data
* perform_OTA.h
   - handles over the air updates
* notes*.h
   - provides circuit and background information
