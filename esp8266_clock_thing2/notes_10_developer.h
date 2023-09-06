/* -*- mode: c++; -*- */

/*
 * NodeMCU circuit diagram has a note titled
 * "MATTERS NEEDING ATTENTION"
 *   On every boot/reset/wakeup,
 *   GPIO15 MUST keep LOW, GPIO2 MUST keep HIGH.
 *   GPIO0 HIGH ->RUN MODE, LOW -> FLASH MODE.
 *   When you need to use the sleep mode, GPIO16 and RST should be connected,
 *   and GPIO16 will output LOW to reset the system at the time of wakeup.
 *
 * This implies GPIO 0,2,15 are used for system control so care is needed if
 * they are to be used by application software after startup. This still
 * leaves the default user I2C and SPI pins (Arduino), GPIO16, GPIO10,
 * etc. for direct use.  For a description of the pin modes and selection, see
 * https://www.esp8266.com/wiki/doku.php?id=esp8266_gpio_pin_allocations
 *
 * Current pin use: see PIN ASSIGNMENTS definitions below.
 * Current switches:
 *   GPIO16 handles the pushbutton but alternatives e.g. GPIO10 can be chosen via 'g' config option
 *   ADC handles the summer time switch input
 * Note on IO lines from https://tttapa.github.io/ESP8266/Chap04%20-%20Microcontroller.html
 *   "GPIO 0-15 all have a built-in pull-up resistor, just like in an Arduino.
 *   GPIO16 has a built-in pull-down resistor."
 */

/*
 * NOTE: Section 3.1.1 "General Purpose Input/Output Interface (GPIO)",
 *       p.17 of document 0A-ESP8266__Datasheet__EN_v4.4.pdf states
 *           All digital IO pins are protected from over-voltage with a
 *           snap-back circuit connected between the pad and ground. The
 *           snap back voltage is typically about 6V, and the holding
 *           voltage is 5.8V. This provides protection from over-voltages
 *           and ESD. The output devices are also protected from reversed
 *           voltages with diodes.
 *       That is, the ESP8266 gpio pins appear to be 5V tolerant so in this work, the
 *       peripheral devices such as the PCF8574T (LCD backpack) and the MAX7219 (LED
 *       dot matrix driver) are powered by 5V with I2C and SPI lines directly connected
 *       to the ESP8266 which is powered by 3.3V. In general, the input/output lines are
 *       connected to 3.3V signals.
 *
 * Arduino main user function: void loop()
 *    Initialisation function: void setup()
 */

/*
 * REFERENCES
 *
 *  1 The display uses newLiquidCrystal from https://github.com/fmalpartida/New-LiquidCrystal which
 *    offers a convenient way to access LCD modules attached to an I2C PCF8574T IO expanded "backpack"
 *      Download June2019=> NewLiquidCrystal_1.5.1.zip.
 *      Install=> unzip into your libraries area and then rename its directory to LiquidCrystal i.e. it
 *                is visible in a directory with a name like ~/Documents/Arduino/libraries/LiquidCrystal.
 *  2 https://www.switchdoc.com/2015/10/iot-esp8266-timer-tutorial-arduino-ide/
 *      Has notes on the use of os_timer and "Soft Real Time".
 *  3 Ntp access was inspired by https://github.com/Nurgak/Electricity-usage-monitor (CC3200) and later the
 *    Arduino code was stabilised and formalised which led to the NTPClient-Arduino example.
 *  4 Initial access point code (for input of wifi parameters) taken from WiFiAccessPoint-Arduino code example,
 *      https://learn.sparkfun.com/tutorials/esp8266-thing-hookup-guide/example-sketch-ap-web-server, and
 *      https://www.esp8266.com/viewtopic.php?f=29&t=2153
 *  5 Persistent wifi config and timezone data added to eeprom (see various Arduino examples).
 *  6 Dot matrix display uses LedControl class and font scan from on-line sources.
 *  7 The DHT temperature sensor support comes from Lady Ada @adafruit (install the "non-unified" DHT library).
 *  8 The one-wire DS1820 temperature sensor support comes from the example code at
 *      https://www.milesburton.com/Dallas_Temperature_Control_Library
 */

//  THE HARDWARE USED HERE
//  
//  1a Plan view of circuit using a NodeMCU v1.0 module with builtin USB adapter for development host link
//     (the circuit uses UART control lines to manage the reset and download control).
//
//     NOTE: this is the recommended board because the dev host has full control of downloading and reset.
//     NOTE: the pull up/down resistors can be 10K (marked as XXX below).
//           Some esp8266 boards have had GPIO16 LEAKAGE current issues requiring a 1K (noted below).
//     NOTE: the AdaFruit LED interface takes +5V power (pin Vin) and +3.3V as the IO high threshold,
//           and the LCD backpack takes +5V power.
//
//     NodeMCU orientation: plan view is with esp8266 & antenna at top, and usb connector at bottom.
//
//            o-----------------------------------------------------------------o 3.3V
//            |                                                                 |
//            |   SUMMER TIME                                                   o RESYNC {push button}
//            |  \ {toggle}        _______________                               \ Usually GPIO16 is PB
//        3.3Vo-o o-XXX-o------A0-o ADC0   GPIO16 o D0--------------------------o  in but GPIO10 an option
//            |     10K |         o NC     GPIO5  o D1---i2c_SCL->[]            |
//            |         |         o NC     GPIO4  o D2---i2c_SDA->[] Adafruit   | NOTE ON GPIO16 LEAKAGE:
//  ________> |         | D12-SD3 o GPIO10 GPIO0  o D3       GND->[] 7seg I2C   |  /If system constantly
//  GPIO10 is |         | D11-SD2 o GPIO9  GPIO2  o D4       +5V->[] backpack   | / enters "Conf" mode,
//  opt. GPS  |         |     SD1 o MOSI     3.3V o-------------->[]        10K |/  try a 1K in // to 10K
//  or PB in  |         |     CND o CS        GND o-------------------------XXX-o   between GND and GPIO16
//  or anal.  |         |     SD0 o MISO   GPIO14 o D5------HspiCLK->() SCK
//  clock     |         |     CLK o SCLK   GPIO12 o D6------HspiQ                 SPI dot matrix display
//  pulse out |     10K o-XXX-----o GND    GPIO13 o D7-RXD2-HspiD--->() MOSI      (can use +5Vdc levels)
//            o-------------------o 3.3V   GPIO15 o D8-TXD2-HspiCS-->() SS
//                                o EN     GPIO3  o D9--RX-RXD0
//                                o RST    GPIO1  o D10-TX-TXD0
//         I2C      {}<-----------o GND       GND o
//         4x20 LCD {}<-----------o Vin      3.3V o
//         backpack {}<---i2c SDA  ---x--===--x---     left x == USER switch
//         (+5Vdc)  {}<---i2c SCL        usb          right x == FLASH switch
//                                    connector
//                                    for power
//                                  & programming
////
//  1b Plan view of circuit using an ESP-12 e or f device on an electrodragon white daughterboard
//     with a FTDI-232RL type usb-uart adapter setup in 3.3V mode, and connected as described in
//     Figure "ESP to Serial" https://github.com/esp8266/Arduino/blob/master/doc/ESP_to_serial.png
//                       from https://github.com/esp8266/Arduino/tree/master/doc
//     and featuring no push button switches.  Previous copy is at
//     web.archive.org/web/20170507042752/https://github.com/esp8266/Arduino/blob/master/doc/boards.md
//
//     NOTE: This has serious boot-out-of-reset issues so is not recommended, and has R2 & R4 missing.
//     NOTE: The circuit at ESP_to_serial.png shows separate 3.3V supplies connected between the
//           USB-uart interface and the ESP8266 3.3V. This should be avoided.
//     NOTE: The ESP-12E module on a WHITE daughterboard (0.1" pinouts) has the 3.3V regulator
//           installed on the underside so (very important) ensure the top-side bridging link is
//           removed on https://www.electrodragon.com/product/esp8266-smd-adapter-board-wi07-12/
//
//     ESP-12 orientation: plan view with Vcc at lower left and GND at lower right
//
//        FTDI-RTS & add R4=10K    ___________
//        pullup to 3.3V....RESET o           o GPIO1/RX....FTDI-TX   (top right)
//                            adc o           o GPIO3/TX....FTDI-RX
//        (R1 pullup on LHS) CHPD o           o gpio5/I2C_scl
//        theButton........GPIO16 o           o gpio4/I2C_sda
//                 gpio14/SPI_sck o           o GPIO0.......FTDI-DTR & add R2=10K pullup to 3.3V
//                gpio12/SPI_miso o           o gpio2.......DHT11 temp&humidity sensor or One-Wire or GPS
//                gpio13/SPI_mosi o           o gpio15/SPI_ss (R3 pulldown on RHS)
//        +ve power supply....VCC o           o GND.........FTDI GND & power supply GND
//                                 -----------
//                               [R1] [0Ohm] [R3]
//                                      ^--- remove this 0Ohm bridge if using the 3.3V reg
////
//  1c Plan view of TronixLabs red adapter board which is similar to Adafruit
//     Huzzah with push buttons for RESET and GPIO0 (to download: press both
//     buttons and then release RESET so module sees GPIO0 pressed at startup)
//
//     ESP-12 orientation: plan view with Vcc at lower left and GND at lower right
//
//                          RESET o ( 1) (22) o GPIO1 D10-TX-TXD0   (top right)
//                            adc o ( 2) (21) o GPIO3 D9--RX-RXD0
//                             EN o ( 3) (20) o gpio5 D1 i2c_SCL
//        theButton........GPIO16 o ( 4) (19) o gpio4 D2 i2c_SDA
//            gpio14 D5 Hspi sclk o ( 5) (18) o GPIO0 D3 .......DTR with 10K pullup to 3.3V (host control)
//            gpio12 D6 Hspi miso o ( 6) (17) o gpio2 D4 .......DHT11 temp&humidity sensor or One-Wire or GPS
//            gpio13 D7 Hspi mosi o ( 7) (16) o gpio15 D8-TXD2 Hspi CS
//        +ve DC input........VCC o ( 8) (15) o GND.............GND
//                           CS0  o ( 9) (14) o  SCLK
//                          MISO  o (10) (13) o  MOSI
//                        # gpio9 o (11) (12) o gpio10 #       #esp12 and board have mismatched gpio 9&10!
//                                 -----------                  (pin names from esp12 module shown here)
//                           [pbRST](+5)(+3.3)[pbFlash]
//  
//  2  The PCF8574T I2C io expander can be soldered directly on to the LCD module if the optional
//     16x2 or 20x4 LCD is used, although it really is easier to just buy the LCD with its I2C
//     adapter.  See for example https://www.google.com/search?q=I2C+LCD+20x4
//
//  3  An Adafruit 7-segment LED backpack which is based on the HT16K33 driver chip, or a separate
//     HT16K33 break-out board soldered to your 7-segment display).  See for example
//       https://www.google.com/search?q=HT16K33+break-out+board
//       NOTE: The HT16K33 also supports an extensive keyboard decode capability which is not
//             used here but worth noting if more user input is needed.
//
//  4  A 4 element 8x8 LED array cascaded on an SPI bus, using the max7219 driver chip.
//       NOTE: This display option needs further work so it can use fast DMA transfers.
//  
//  THE SOFTWARE USED HERE
//  
//  1  As noted earlier, control of the display on the I2C "LCD backpack" makes use of
//     NewliquidCrystal_*.zip from https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
//     to _replace_ the default library LiquidCrystal. Earlier versions such as 1.3.4 required
//     some changes so that the esp8266 development tools could use NewliquidCrystal but when
//     tried again with versions 1.5.x, everything works as-is after unzipping.
//  
//  2  The ESP8266 environment provides extern C functions to specify a timer interrupt function
//     to call back when a software based timeout occurs. This uses an os_timer_t data structure
//     that is specified via os_timer_setfn() which also specifies the user call back. The timer
//     interrupt interval is set by a call to os_timer_arm() which in this example sets the timer
//     mode to continuous operation.
//  
//  3  Pin allocations are documented at
//           https://www.esp8266.com/wiki/doku.php?id=esp8266_gpio_pin_allocations
//           Booting up in a (default) UART-enabled mode: GPIO1==uart-TX, GPIO3==uart-RX.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

// r10. some sensor attributes are concatenated to give a final My_Version C-string
// r11. option to specify time server
// r12. added ability to configure system via serial port link (in addition to wifi AP)
// r13. uniquely identify init stage entered&active with serial port messages and LED flashing
// r14. add timed resync press to allow alternative modes, update summer time every second
// r15. added timed resync and also timed summer-time switch activity to display least sig IP 8bits
//      (on current network) and also add mDNS service to ClockThink-xxxx where xxxx is least sig
//      MAC 16 bits, added telnet access as alternative to serial console after start up
// r16. added pulse output option to drive analogue clock motor
// r17. added OTA from home server, triggered by PB press > 10 seconds or a telnet '@' command.
//      Added local network OTA triggered by telnet 'U' command and then using the web interface
//      at http://ClockThing-XXXX/update for user selection and upload of the firmware file, and
//      docs say that curl can also then be used to push firmware to this OTA e.g.
//        curl -F "image=@firmware.bin" ClockThing-XXXX.local/update
//        curl -F "image=@firmware.bin" IP_number/update
// r18. moved full IP display to sustained PB press so momentary partial IP display removed
//      from the first few seconds after an NTP resync. Sustained switching activity on the
//      summer time switch is retained as a way to display the least significant byte of IP
// r19. updated and enabled dot matrix display support
// r20. discovered arduino-cli and gained access to gcc "--warnings all" so code cleanup done
// r21. at power up, a depressed push button allows for reconfiguration if released when prompted
//      otherwise the push button is marked as faulty & ignored (must then use telnet to configure)
// r22. test code wrapped in ifdef(use_LocalUpdateServer) and later (r25) removed, and now
//      display OTA compiled defaults (the values before EEPROM updates applied)
// r23. better status updates for the various forms of update
// r24. make time display on the dot matrix LED display be an alternative to using the 7-segment
//      LED display i.e. all displays are initialised and retain status message output, but as a
//      simple way to reduce IO tasks the time of day output is directed to one or other LED display
//      (via tests on variable led_display_is)
// r25. remove unused code for ifdef(use_LocalUpdateServer), updated call to ESPhttpUpdate.update used in
//      enable_OTA_update_from_main_server(), fixed form decoding used by enable_reconfigure_via_wifi(),
//      add convenient specification of misc_bytes[MISC_parms_size] via some preset options selected
//      with an upper case char: {D, P}. Note: all upper case chars except 'L' are available
// r30. further code reorganisation,
//      bump version number so users can easily check if system has r25 fixes installed
// r31. add 12-hour display mode (HOURS_mode_is)
// r32. minor code cleanups, changed PBdown for 20..30secs to be (temporary) HOURS_mode_is toggle
// r33. better status updates on LED dot matrix, additional code reorganisation
// r33. improve message timing at boot, add user cmd echo to DotMatrix display, remove help on display
//      brightness change commands (while they work, a time update on a display resets the brightness)
// r43. switch to use of wifiMulti to now also provide recovery wifi AP at ("localnet", "pass1234")
// r44. add softSerial support for reading time from a NEO-8M-001 GPS receiver, using gpio2

// 2023: restarted revision numbering at 1
// r1.  implemented GPS time source (as an alternative to NTP)
// r2.  manage internet access as necessary (NTP use) or optional (GPS use) so network setup is optional,
//      further code cleanup by having attribute/option setting/getting/definitions etc. in main .ino file,
//      improve GPS time sync by waiting for a change in seconds to ensure fractional seconds = 0, and do a
//      similar change for NTP time sync for consistency (but note that network latency is not addressed)
// r3.  added a message to dot-matrix display during system startup to say if it is not going to be active
//      (at least user then knows why time not visible), made L[01] config found in eeprom be maintained
//      after a system reconfigure (unless user specifically changes it)
// r4.  no code change but libraries updated and executable rebuilt (with r4 tag)
// r5.  Phillip_Clock_Thing2.ino now sets WiFi.setPhyMode(WIFI_PHY_MODE_11G) in order to avoid any 5GHz
//      wifi network that shares an SSID with a 2.4GHz wifi network (very important)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
