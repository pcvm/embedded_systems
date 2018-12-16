/* -*- mode: c++; -*-
 *
 * ESP8266 DEMONSTRATION of I2C and LCD, 1-bit LED output, and Serial communications
 *
 *   p.musumeci@ieee.org Jan-2016
 *
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
 * THE HARDWARE USED HERE
 *
 * - An FTDI-232RL based usb-uart adapter is used in 3.3V mode, and connected as described in
 *   Figure "ESP to Serial" at https://github.com/esp8266/Arduino/blob/master/doc/boards.md
 *
 *   NOTE: the circuit at https://github.com/esp8266/Arduino/blob/master/doc/ESP_to_serial.png
 *         shows direct connection between the 3.3V supply from the USB-uart interface and the
 *         3.3V from the ESP8266 board - you should keep these supplies separate (unless your
 *         USB-uart power source really is strong enough to drive everything).
 *
 * - The ESP-12E module is on a WHITE daughterboard (0.1" pinouts) with the 3.3V regulator added
 *   on the underside and (very important) the middle bridging 0-Ohm resistor removed on top.
 *   See http://www.electrodragon.com/product/esp8266-smd-adapter-board-wi07-12/
 *
 * - The WHITE daughterboard has R1 and R3 of Figure "ESP to Serial" already installed, so pay
 *   careful attention to adding R2 and R4 as noted below:
 *
 *                 ESP-12 series connections (top view)
 *
 *    FTDI-RTS & add R4=10K    ___________
 *    pullup to 3.3V....RESET o           o GPIO1/RX....FTDI-TX   (top right)
 *                        adc o           o GPIO3/TX....FTDI-RX
 *    (R1 pullup on LHS) CHPD o           o gpio5/I2C_scl
 *    theButton........GPIO16 o           o gpio4/I2C_sda
 *             gpio14/SPI_sck o           o GPIO0.......FTDI-DTR & add R2=10K pullup to 3.3V
 *            gpio12/SPI_miso o           o gpio2.......DHT11 temperature&humidity sensor or One-Wire
 *            gpio13/SPI_mosi o           o gpio15/SPI_ss (R3 pulldown on RHS)
 *    +ve power supply....VCC o           o GND.........FTDI GND & power supply GND
 *                             -----------
 *
 *                           [R1] [0Ohm] [R3]
 *                                  ^--- remove this 0Ohm bridge if using the 3.3V reg
 *
 * - The PCF8574T I2C io expander can be soldered directly on to the LCD module.
 *
 * + An optional LED driven by GPIO16 can be selected in order to free up the line used for
 *   serial output. Feed GPIO16 via a 1K resistor to an LED and define LEDon16 below.
 *   (this line is used later for a user input switch)
 *
 * THE SOFTWARE USED HERE
 *
 * - To simplify use of the display on the I2C "LCD backpack", we take files from
 *   NewliquidCrystal_1.3.4.zip (https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads)
 *   to _replace_ the default library LiquidCrystal (on OS-X, check for a location such as
 *   /Applications/Arduino.app/Contents/Java/libraries/LiquidCrystal into which the files of
 *   NewliquidCrystal are copied).
 *
 *   NOTE: the esp8266 compiler chain fails on SR (shift register) and Software protocol variants
 *         of NewliquidCrystal so just remove or rename files matching {*_SR*, SI2C*, *_SI2C*}.
 *         In sh:  for x in *_SR* SI2C* *_SI2C* ; do echo $x ; mv ${x} ${x}_disabled ; done
 *
 * - The ESP8266 environment provides extern C functions to specify a timer interrupt function
 *   to call back when a software based timeout occurs. This uses an os_timer_t data structure
 *   that is specified via os_timer_setfn() which also specifies the user call back. The timer
 *   interrupt interval is set by a call to os_timer_arm() which in this example sets the timer
 *   mode to continuous operation.
 *
 * - Pin allocations are documented at
 *       http://www.esp8266.com/wiki/doku.php?id=esp8266_gpio_pin_allocations
 *     Booting up in a UART-enabled mode (the default) means GPIO1 provides
 *     uart-TX and GPIO3 provides uart-RX.
 *
 * REFERENCES
 * 1 What works, what doesn't -- https://github.com/esp8266/Arduino/blob/master/doc/reference.md
 * 2 Using newLiquidCrystal from https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home and see also
 *   http://tronixstuff.com/2014/09/24/tutorial-serial-pcf8574-backpacks-hd44780-compatible-lcd-modules-arduino/
 */

#define LEDon16 1

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
//
// Instantiate LCD at I2C backpack address 0x27
// - LCD operates in 4-bit data mode
// - I2C address = PCF8574T base address 0x20 + least significant 3 bits set by user
//
#define BACKLIGHT_PIN  3        // PCF8574 connections to the LCD
#define En_pin         2        //
#define Rw_pin         1        //
#define Rs_pin         0        //
#define D4_pin         4        // b4
#define D5_pin         5        // b5
#define D6_pin         6        // b6
#define D7_pin         7        // b7
//
LiquidCrystal_I2C  lcd(0x27,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);

// Choose LED to flash
#if defined(__AVR__)
#define LED 13  // Atmel/AVR
#else
#if defined( LEDon16 )
#define LED 16  // Use free gpio pin to drive an LED, which allows Serial to be used
#else
#define LED  2  // Default ESP8266 LED on the TX line (LED use interferes with Serial port use)
#endif
#endif

//
// system setup function
//
int global_count;
void setup()
{
                                // initialise I2C
  Wire.begin( 4,5 );            // specify SDA==>GPIO4, SCLK==>GPIO5
  // Can also set speed e.g. Wire.setClock( 100000 )

                                // initialise serial comms and LED pin
#if defined( LEDon16 )
  Serial.begin(115200);
#endif
  pinMode(LED, OUTPUT);
                                // activate LCD module
  lcd.begin (20,4);             // for 20 x 4 format and
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);       // active backlight
 
  global_count=0;               // demo program loop counter
}

//
// main user function
//
void loop()
{
  digitalWrite(LED, HIGH);    // turn LED on and off
  delay(250);                 // with waits of 1/4 second
  digitalWrite(LED, LOW);
  delay(250);
  digitalWrite(LED, HIGH);
  delay(250);
  digitalWrite(LED, LOW);
  delay(1000);

  global_count++;
#if defined( LEDon16 )
  Serial.println(global_count);
#endif

  lcd.home ();                // set cursor to 0,0
  lcd.print("Hello world :-) v2");
  lcd.setCursor (0,1);        // move to start of 2nd line
  lcd.print(global_count);

  lcd.setBacklight(LOW);      // Backlight off
  delay(250);
  lcd.setBacklight(HIGH);     // Backlight on
}

