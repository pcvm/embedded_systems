/* -*- mode: c++; -*- */

/*
 * It provides an...
 *
 *   ESP8266 demonstration of I2C, SPI and 1-wire interface use, LCD and LED display
 *   (7-segment and dot matrix array) use, some GPIO (switch input, 1Hz output) and
 *   ADC input, and other io using the UART (console config) & WiFi networking (telnet
 *   config, initial http config and later OTA updates). An earlier educational app
 *   was built using an AVR microcontroller that drove an LCD, designed last century.
 *
 *   Clock Thing is a:
 *   . time-of-day clock using real-time timer interrupts and with a daily ntp call
 *     to resynchronise, all running on a NodeMCU module (ESP8266) or similar with
 *   . a default display of a 4x20 LCD attached via an I2c to 8bit adapter (PCF8574),
 *   . an optional simultaneous LED display that is either
 *     - an I2C connected 4 digit 7-segment AdaFruit LED backpack (HT16K33 driver) or
 *     - an SPI connected 4 section 8x8 LED dot matrix (MAX7219 driver + Aussie s/w),
 *   . an optional display of temperature and humidity (DHT serial io) or temperature
 *     (Dallas Semi DS1820 1-wire) is provided if appropriate sensors are attached,
 *   . an optional 1Hz digital time keeping output signal is available for use in
 *     driving mechanical time displays, and
 *   . user controls that are physically in the clock include:
 *     - a push button switch to provide for manually forcing a time resync (momentary
 *       press), inspecting computer network settings (longer press <8 seconds), and
 *       triggering a software update (really long press >10 seconds and <20 seconds);
 *     - a toggle switch to advance the time by 1 hour for daylight savings time.
 *
 *   Initial setup can be done with a WiFi connected smartphone or PC:
 *   . while holding the push button switch pressed, power up and only release when
 *     you see a message saying "Rel " "PB  " "for " "conf" (using limited 4 char
 *     7-segment ascii text) or similar on the more capable LCD or LED dot matrix
 *     displays;
 *   . if the PB switch is then released, a wifi access point is activated with an
 *     SSID of "ClockThing-xxxx" with the clock display showing the xxxx string. As
 *     there is usually only one wifi network with a name containing "ClockThing",
 *     connect as soon as you see it and then go to URL http://192.168.4.1 and fill in
 *     the configuration details for:
 *     - the wifi network SSID and password that the clock will normally connect to;
 *     - the time zone offset from GMT/UTC e.g. 10 for QLD/au, 1 for Europe, 0 for UK;
 *     - options such as a display dimming schedule (see the cut&paste examples);
 *   . when complete, press the submit icon. The clock will reboot and connect to WiFi.
 *
 *   Clock Thing can respond to the user when in normal use:
 *   . a momentary press of the push button switch will trigger a clock resync with an
 *     internet time server;
 *   . holding down the push button switch for different times gives access to various
 *     options:
 *     - holding the PB switch pressed for 9 seconds will cycle through a display of
 *       the 4 numbers in the IP address of the clock (tagged -BUTTONDOWN-less-than-10-);
 *     - holding the PB switch pressed for 10 to 19 seconds will trigger an over-the-air
 *       (OTA) software update from Phillip's server  (tagged -BUTTONDOWN-10-to-lt-20-);
 *     - holding the PB switch pressed for 20 to 29 seconds will toggle the 12/24 hour
 *       display mode                                 (tagged -BUTTONDOWN-20-to-lt-30-);
 *     - rapid toggling of the day light savings mode switch will display the last number
 *       in the IP address;
 *   . a console terminal interface is also available (once initial networking is setup).
 *     A user can telnet in to the IP address displayed by pressing the PB for 9 seconds
 *     and enter h for help. All settings can be changed and saved.
 *
 *   Clock Thing has support for using a local time server:
 *   . some people (i.e. geek type people) run their own ntpd time server, so this can
 *     be selected via optional setting of "t192.168.1.210" where the raw IP number
 *     follows 't'. Quite useful if you have more than one Clock Thing and/or have a
 *     robust time reference available.
 *
 *   Clock Thing can survive some hardware failures:
 *   . push button switch failure with USB attached computer available
 *     - if the push button switch fails as an open circuit, initial config can never be
 *       accessed and so the clock will proceed to normal operation. However, it is still
 *       possible to configure the clock at power up using a USB connection to a terminal
 *       emulator (115200 b/s) in an attached computer (there is a 2 second interval at
 *       startup when any received character triggers entry to a configure Q&A session);
 *     - if the push button switch fails as a short circuit, initial config is skipped
 *       because the switch is not released when prompted. As noted above, a terminal
 *       emulator attached during power up can be used to configure the system;
 *   . push button switch failure with no USB option but networking already setup
 *     - so long as you have already configured the networking and the clock connects
 *       to your WiFi network, you can telnet into the clock and press h for help.
 *       Some configuration, software update, and inspection commands are provided (these
 *       are always available and are referred to as geek mode commands);
 *     - the telnet command will look something like "telnet 192.168.1.123" but there
 *       may be an issue in figuring out the 4 numbers that form the "IP" address. The
 *       first 3 numbers will be the same for all of the devices on your home network so
 *       simply inspecting the WiFi settings on a smartphone gets you most of the "IP"
 *       numbers but you still need the last number --- rapid toggling of the day light
 *       savings switch will display the last number in the IP, so then you can telnet.
 *   . push button switch related repairs
 *     - if you have some basic soldering skill, replacing the switch is an option.
 *     - in one instance, the actual input pin of the ESP8266 chip was zapped so rewiring
 *       to use a different input was possible. We have had success rewiring the switch to
 *       gpio10 instead of gpio16, and then the user was able to enter an optional setting
 *       via telnet (or bootup USB attached terminal) of "g10". This will automatically
 *       disable the 1Hz time reference output (search the source code for strings such as
 *       "defaultButton" and "PIN ASSIGNMENT").
 *   . if the above work arounds do not solve hardware failures, there is a final option
 *     of a custom executable (see SPECIAL below).
 */
