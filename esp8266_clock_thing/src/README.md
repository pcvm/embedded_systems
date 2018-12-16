# src

* file esp8266_I2C_LCD.ino
  - I2C driving an LCD "PCF8574T backpack" display;
  - To simplify use of the display on the I2C "LCD backpack", we take files from F. Malpartida's NewliquidCrystal_1.3.4.zip (https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads) to _replace_ the default library LiquidCrystal (on OS-X, check for a location such as /Applications/Arduino.app/Contents/Java/libraries/LiquidCrystal);
  - NOTE: the esp8266 compiler chain fails on SR (shift register) and Software protocol variants of NewliquidCrystal so just remove or rename files matching {&#42;_SR*, SI2C*, &#42;_SI2C*}. In sh:<br>&nbsp; &nbsp;  cd LiquidCrystal <br>&nbsp; &nbsp; for x in <strong>&#42;_SR* SI2C* &#42;_SI2C* </strong> ; do echo $x ; mv ${x} ${x}_disabled ; done

* file esp8266_I2C_LCD_timerIRQ.ino
  - adding timer IRQ use;
  - Note: call os_timer_setfn() before os_timer_arm();

* file esp8266_I2C_LCD_timerIRQ_ntp.ino
  - adding NTP access (system uses your wifi network to access a time server);
  - See the NTPClient-Arduino example code;

* file esp8266_I2C_LCD_timerIRQ_ntp_ap.ino
  - adding a wifi access point for configuration data input (you get to enter local wifi network details and specify your time zone);

* file esp8266_I2C_LCD_timerIRQ_ntp_ap_nv.ino
  - adding EEPROM use for configuration store (system can save and recall config details);

* file esp8266_I2C_LCD_timerIRQ_ntp_ap_nv_dotm.ino
  - now driving SPI linked MAX7219-connected LED dot matrix displays;

* file esp8266_I2C_LCD_timerIRQ_ntp_ap_nv_dotm_DHT.ino
  - adding One-Wire protocol access to DS temperature sensors _or_ serial protocol access to a DHT sensor; and

* file esp8266_I2C_LCD_timerIRQ_ntp_ap_nv_dotm_DHT_ui.ino
  - adding a one button switch user interface.

---------------------------------------------------------------------------

PROGRAMMING ENVIRONMENT

Host:
* Arduino 1.6.7 (or Arduino nightly from 2-Feb-2016)
* Additional Boards Manager URL: http://arduino.esp8266.com/stable/package_esp8266com_index.json
  - This provides Library/Arduino15/packages/esp8266/tools/xtensa-lx106-elf-gcc etc.

Target:
* ESP8266 Generic Module
* CPU Frequency:80MHz ResetMethod:ck
* Flash Mode:DIO Frequency:40MHz Size:512K
* Upload Using:Serial Speed:921600
* Programmer: USBasp

LIBRARIES

The default ESP8266 libraries are used, except for
 - LiquidCrystal in folder: /Users/phillip/Documents/Arduino/libraries/LiquidCrystal (this is really newLiquidCrystal by F Malpartida, referenced above)
 - DallasTemperature at version 3.7.6 in folder: /Users/phillip/Documents/Arduino/libraries/DallasTemperature
 - LedControl at version 1.0.6 in folder: /Users/phillip/Documents/Arduino/libraries/LedControl

The final messages in a build include the following library information:
<pre>
Using library ESP8266WiFi at version 1.0 in folder: /Users/phillip/Library/Arduino15/packages/esp8266/hardware/esp8266/2.0.0/libraries/ESP8266WiFi 
Using library Wire at version 1.0 in folder: /Users/phillip/Library/Arduino15/packages/esp8266/hardware/esp8266/2.0.0/libraries/Wire 
Using library LiquidCrystal in folder: /Users/phillip/Documents/Arduino/libraries/LiquidCrystal (legacy)
Using library EEPROM at version 1.0 in folder: /Users/phillip/Library/Arduino15/packages/esp8266/hardware/esp8266/2.0.0/libraries/EEPROM 
Using library OneWire in folder: /Users/phillip/Library/Arduino15/packages/esp8266/hardware/esp8266/2.0.0/libraries/OneWire (legacy)
Using library DallasTemperature at version 3.7.6 in folder: /Users/phillip/Documents/Arduino/libraries/DallasTemperature 
Using library LedControl at version 1.0.6 in folder: /Users/phillip/Documents/Arduino/libraries/LedControl 
Using library ESP8266WebServer at version 1.0 in folder: /Users/phillip/Library/Arduino15/packages/esp8266/hardware/esp8266/2.0.0/libraries/ESP8266WebServer 

Sketch uses 258,012 bytes (59%) of program storage space. Maximum is 434,160 bytes.
Global variables use 39,326 bytes (48%) of dynamic memory, leaving 42,594 bytes for local variables. Maximum is 81,920 bytes.
warning: serialport_set_baudrate: baud rate 921600 may not work
Uploading 262160 bytes from /var/folders/5l/b10qh4yn70jfsyh_967qnzn80000z8/T/build38bc27e8901f817645a0f38814bcfa50.tmp/esp8266_I2C_LCD_timerIRQ_ntp_ap_nv_dotm_DHT_ui.ino.bin to flash at 0x00000000
</pre>
---------------------------------------------------------------------------
