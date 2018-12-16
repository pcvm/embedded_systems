/*
 * echo_audio
 *   reads in microphone signal via pin SENSOR_pin_adc and averages adjacent values
 *   input data is 12 bit i.e. 0..1023
 *   output data is 8 bit i.e. 0..255 so sum(new+old)/2 must be divided by 16 for scaling
 *
 * See also analogue input examples
 */

///////////////////////////////////////////////////////////////////////////
#include <Wire.h>	// access I2C
#include <LCD.h>	// and LCD
#include <LiquidCrystal_I2C.h>

// Specify LCD geometry and optionally choose "fat" digits
#define nCOLS           20
#define nROWS            4

// Instantiate I2C connected LCD at backpack address 0x27
// - this is like 4-bit mode for standard LiquidCrystal described at
//     https://www.arduino.cc/en/Reference/LiquidCrystalConstructor
//   but with extra arguments defining I2C port address and E pin number
// - the address = base address 0x20 + least significant 3 bits set by user
#define BACKLIGHT_PIN  3
#define En_pin  2
#define Rw_pin  1		// Note that these pin numbers refer
#define Rs_pin  0		// to connections between the LCD
#define D4_pin  4		// backpack PCF8574 chip and the LCD
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7

LiquidCrystal_I2C  lcd(0x27,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);
///////////////////////////////////////////////////////////////////////////

// Define signal names in terms of standard Arduino board pin numbers
#define LED_pin         13	// pin 13
#define SENSOR_pin_adc  A0	// adc0	

// Define array for sensor values using smallest possible int types
#define nHistory 16		// number of samples to keep for processing
static uint16_t sensorValues[nHistory];
static uint8_t  nexts = 0;	// note where next sample is stored
static uint8_t  lasts = nHistory-1;

// initialise IO ports
void setup() {
                                // initialize digital LED_pin as an output
  pinMode(LED_pin, OUTPUT);

  lcd.begin(nCOLS, nROWS);      // activate LCD module
  lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
  lcd.setBacklight(HIGH);
  delay(100);

                                // clear sample buffer
  for (int i = 0; i < nHistory; i++)
    sensorValues[i] = 0;
}

// main loop
// - sum nHistory elements and hack into a bar graph
					// for nHistory==16
const char bg[]="===============>";	// 16 chars
const char bl[]="                ";	// 16 chars
void loop() {
                                // read sensor and smooth sensorValue
  sensorValues[nexts] = analogRead(SENSOR_pin_adc);

  lasts = nexts;                // update cyclic indices
  if (++nexts >= nHistory)
    nexts = 0;

  uint16_t sum = 0;
  for (uint8_t i = 0; i < nHistory; i++)
    sum += sensorValues[i];

  // scale data for display as a bar graph no wider than 20 columns
  //   . sum of 16x 12 bit numbers ==> 4+12
  //   . to scale this back to 8 bits, we right shift by 4+12-8
  //   . an extra 4 bit shift brings the value into the range 0..15
  sum = sum >> (12 + 4 - 8);
  if (sum > nHistory)
    sum = nHistory;

  lcd.setCursor(0, 0);
  lcd.print(bg + nHistory - sum);
  lcd.print(bl + sum);
}
