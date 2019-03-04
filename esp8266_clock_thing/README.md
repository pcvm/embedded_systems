# esp8266_clock_thing

An exploratory sequence of programs eventually produced a time-of-day clock demo featuring NTP synchronisation and WiFi access point configuration. These files grew from a code sample set for an embedded systems workshop that changed from Atmel AVR processors to the ESP8266 platform in 2015-2017. The main display for time is an Adafruit I2C connected 4 digit LED, and an additional I2C connected 4x20 LCD can show time as well as optional temperature and humidity (plus status/debug info). The clock works just fine with only an LED display.

* Phillip_Clock_Thing2_11.ino
   - On the first power up, the system creates an open wifi access point with a name starting as ClockThing and a configuration web page at URL http://192.168.4.1 so that you can enter config data for WiFi SSID and WiFi password, and time zone (as an hours difference relative to UTC/GMT e.g. 10 for north eastern Australia). An optional IP number for a local time server may also be entered e.g. 192.168.1.2 (usually ignored but might be useful if you want your local systems all using say a common openntpd server). After entering all details, touch the Submit button and then power the clock off and then on. At subsequent starts, the system connects to an NTP server and starts time keeping.
   - How to re-enter local settings: with the push button switch pressed and held, do a power up and wait until the LED shows "ConF" (that is LED-speak for "Config") and then release the push button. As done on the first power up, set your mobile phone to connect to the wifi access point with a name starting as ClockThing and then visit URL http://192.168.4.1. Fill in the details and touch Submit.
   - How to resynchronise the time: press the push button switch for a few seconds during normal operation to trigger an NTP resync.
   - Daily time synchronisations are set for a time like 2:34 (see source code defines TimeSync_*). An indicator LED (bottom left) is left turned on if a time sync fails (e.g. no internet access). The ESP8266 oscillator is quite accurate so missing some daily NTP resyncs is not a problem.
   - How to build the circuit: wiring details are in the source code.
   - Summer time: this can be selected by adding a toggle switch to a resistor potential divider connected to the analogue input (wiring details in the source code).
   - Unused source code: code to drive 4 SPI connected 8x8 LED arrays is a work in progress.

* Phillip_Clock_Thing2_12.ino
   - This is release 11 but with an alternative system setup mechanism via the serial port. At boot time, the serial port is initialised to 115200 bits/second and all busses and peripherals are configured with progress messages. If the system is booting up for the first time or with the push button switch pressed to select setup, then a message is issued asking if the serial port should be used for setup. The user has 2 seconds in which to respond and if desired, can enter WiFi and time zone details (otherwise setup will use the usual access point WiFi mode and web page at http://192.168.4.1).
   - All files were removed and then uploaded again.

* src/
   - The current clock resulted from the sequence of source code files (in src/).

* A growing set of references at the top of the source file sequence indicates where ideas (and libraries) came from i.e. (March2016):
   - 1 What works, what doesn't -- https://github.com/esp8266/Arduino/blob/master/doc/reference.md
   - 2 Using newLiquidCrystal from https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home and see also http://tronixstuff.com/2014/09/24/tutorial-serial-pcf8574-backpacks-hd44780-compatible-lcd-modules-arduino/
   - 3 http://www.switchdoc.com/2015/10/iot-esp8266-timer-tutorial-arduino-ide/
   - 4 Ntp access was inspired by https://github.com/Nurgak/Electricity-usage-monitor (example) but then I read the NTPClient-Arduino example.
   - 5 Initial access point code (for input of wifi parameters) taken from  WiFiAccessPoint-Arduino code example, https://learn.sparkfun.com/tutorials/esp8266-thing-hookup-guide/example-sketch-ap-web-server, and http://www.esp8266.com/viewtopic.php?f=29&t=2153.
   - 6 Persistent wifi config and timezone data added to eeprom (see various Arduino examples).
   - 7 Dot matrix display uses LedControl class and font scans for digits from http://tronixstuff.com/2013/10/11/tutorial-arduino-max7219-led-display-driver-ic/
   - 8 The DHT temperature sensor support comes from Lady Ada @adafruit (install the "non-unified" DHT library).
   - 9 The one-wire DS1820 temperature sensor support comes from the example code at https://milesburton.com/Dallas_Temperature_Control_Library
   - 10 Add timezone data to eeprom storage (now holds 2 strings and an int), and check if theButton is set at power up to allow user reset (enter local access point mode and input WiFi SSID/password and timezone data)
   - 11 If theButton input is set during normal operation, resynchronise with the ntp time source (hey, a user interface!)

Selected library files are provided in libs/.
