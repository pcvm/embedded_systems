# embedded_systems

This work and play relates to some small personal projects and also my teaching in the area of embedded systems. Initial commits were in 2016 with files reloaded in Dec-2018.

* esp8266_clock_thing
  - A sequence of files that grow to form a digital clock using a 4 char 8x8 LED dot matrix display and a 4x20 LCD module, with NTP time sync and temperature display.
  - Developed as an aid for an embedded systems workshop.

* esp8266_clock_thing2 (2022)
  - Latest clock source code plus user information. Some code for a https server is not used (lack of CPU power) but reference time is now taken from an optional GPS source as well as the original NTP source.
  - The whole project has grown far beyond any initial plans and, even though code has been split into separate files to preserve some sanity, it is "messy". In defence, some of the display hardware and even software to decode NTP responses did not exist when the early versions of this teaching demo began. Actually, it all began in 2000 with...

* avr_clock_demo_2000
  - An example of managing timer interrupts in C code on the ATMEL AT90S8515 (before Arduino existed)
  - This provides a simplistic digital clock using a Hitachi type 2row/16column LCD module (this same module is a display mode for esp8266_clock_thing2 via an I2C link)
  - coded in C around year 2000

* emonTx_energy_monitor
  - A fork of the Atmel AVR control program for the open energy monitor (from 2016).
  - This minimalist system has been frozen as I want to change the hardware.
  - Developed for a home energy monitoring system.

* esp8266_Humidity+Temperature_thing_wifi
  - A basic web server providing hygrometer and temperature results from an AM2302 sensor.
  - Uses Adafruit DHT-sensor-library for DHT22/AM2302.
  - User provides wifi details before compiling and uploading firmware.

* raspberry-pi_ada-fruit_clock
  - A minimalist clock using a Raspberry Pi Zero and AdaFruit display.
  - Developed from the idea of just how simple can an off-the-shelf NTP clock be...

* relayr_experiments
  - Provides sample mqtt loggers using the relayr.io service.
  - Examples in python and C/C++ are provided.

* workshop
  - Materials from a 2015 introductory microcontroller workshop.
