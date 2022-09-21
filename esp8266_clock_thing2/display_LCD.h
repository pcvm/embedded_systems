/* -*- mode: c++; -*- */

//
// Definitions for the I2C attached 20 column LCD
// Note: currently use fmalpartida improved LCD access code from
//       https://github.com/fmalpartida/New-LiquidCrystal
//

// LCD geometry:
#define lcd_ROWS   4		// a 2 row display just shows the top 2 rows of content vs. a 4 row display
#define lcd_COLS  20

// The same code drives both the 4x20 and 2x20 displays so a preference for useful
// information is placed in the first 2 rows and additional optional information is
// placed in the optional extra 2 rows. Display row allocations:
// --> eeprom reading
//     row 0	msg, ssid	msg, ssid
//     row 1	password	password
//     row 2	time zone	<not visible>
//     row 3
// --> eeprom writing
//     row 0	clr msg,         writing			   Summary (use rows 1-3 as stack)
//     row 1	         hdr msg,       ssid password timezone :-)	   ssid password timezone
//     row 2								        ssid     password
//     row 3									         ssid
// --> general use
//     row 0	Time and Temperature, and sensor state machine (ssm) status byte
//     row 1	Resync time
//     row 2	IP number
//     row 3	Port number, plus some switch events
//
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

// Instantiate one display
LiquidCrystal_I2C  lcd(0x27,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);

// Define lcd convenience macros and functions
//
#define lcd_setCursorSyncFlag           lcd.setCursor(lcd_COLS-2,1)
//
void clear_lcd_row( int r ) {           // overwrites text with ' ' chars
  lcd.setCursor(0,r);
  for ( int i=0; i<lcd_COLS; i++ ) lcd.print(' ');
  lcd.setCursor(0,r);
}
