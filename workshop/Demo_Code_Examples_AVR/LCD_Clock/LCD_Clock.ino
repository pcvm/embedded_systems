/*
 * Clock_I2C_LCD implements a 6 digit clock using an I2C-connected LCD and
 * timer 2 interrupts for time keeping. It demonstrates interrupt
 * configuration and LCD use with graphics and text.
 *
 * LCD is attached to a PCF8574T-based I2C LCD-backpack using an enhanced
 * LiquidCrystal_I2C library contributed by F Malpartida
 * - library is available from
 *     https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
 *   in the wiki at https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
 *   and it is designed for I2C LCD-backpacks titled
 *     "YwRobot Arduino LCM1602 IIC V1",
 *     "Arduino-IIC-LCD GY-LCD-V1",
 *     "LCM1602 IIC A0 A1 A2",
 *   and probably most recent 2015 generic Chinese I2C backpacks using
 *   one PCF8574 chip (in contrast, an Adafruit backpack uses 2 chips)
 * - this new LiquidCrystal library replaces the existing LiquidCrystal
 *   folder/directory located in the Arduino libraries area. On OS-X, this
 *   is visible at /Applications/Arduino.app/Contents/Java/libraries with
 *   contents such as
 *     Bridge        Firmata       MsTimer2      Robot_Motor   SpacebrewYun  Temboo
 *     Esplora       GSM           RobotIRremote SD            Stepper       WiFi
 *     Ethernet      LiquidCrystal Robot_Control Servo         TFT
 * - other references:
 *     http://forum.arduino.cc/index.php?topic=158312.0
 *     https://arduino-info.wikispaces.com/LCD-Blue-I2C (covers various LCD BackPack versions)
 *     http://tronixstuff.com/2014/09/24/tutorial-serial-pcf8574-backpacks-hd44780-compatible-lcd-modules-arduino/
 *
 * MsTimer2 provides real-time interrupts
 * - https://www.arduino.cc/en/Reference/Libraries and choose 
 *     http://playground.arduino.cc/Main/MsTimer2 to provide real time interrupts
 * - download and install in the libraries area (see the list of library items
 *   found on the development host, shown above)
 *
 * p.musumeci@ieee.org July 2015 (BSD licence)
 */

#define VersionCLK "1v2"

// Specify LCD geometry and optionally choose "fat" digits
#define nCOLS           20
#define nROWS            4
//#define nCOLS           16
//#define nROWS            2

#define time_column1     0	// first column of time display
#if (nROWS>=4)
#define enableFatDigits  1	// define if using multi-char graphics to draw large characters
#define nFatDigits       4
#endif

#define LED_flash 13

#include <MsTimer2.h> // access timer2 overflow interrupts for timekeeping

#include <Wire.h>     // access I2C

// Use LCD attached to I2C backpack
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

///////////////////////////////////////////////////////////////////////////

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

#if defined( enableFatDigits )

//
// Use graphics char capability to define LARGE chunky digits that occupy
// columns on multiple rows
//
// The underlying LCD chars use 8 horizontal slices of 5 pixels each. We
// can subdivide this char into 4 "fat" pixel segments that occupy a
// top-left, a top-right, a bottom-left, and/or a bottom-right 1/4 char
// "chunk". The original char uses 8 horizontal slices so a quarter uses 4
// slices.
//
// The new segments will be organised as 4 possible single segments set
// (for user defined graphics chars 0..3) and also 4 possible adjacent
// pairs of segments (for user defined graphics chars 4..7).
//
// Define GRAPHICS CHARS 0..3:
//
// Define a left fat pixel fragment as 4 slices of a char with the left 3
// real pixels set
#define fatL         B11100, \
                     B11100, \
                     B11100, \
                     B11100
// Define a right fat pixel fragment as 4 slices of a char with the right 3
// real pixels set
#define fatR         B00111, \
                     B00111, \
                     B00111, \
                     B00111
// Define an empty fat pixel as 4 slices of 0
#define fat0         0,0,0,0
//
// It is now possible to assemble fatL and fatR fragments to form the 4
// quarter parts where only one fat pixel is set:
uint8_t FatPixTL[8] = { fatL, fat0 }; // char(0) Top-Left
//                   11100
//                   11100
//                   11100
//                   11100
//                   00000
//                   00000
//                   00000
//                   00000
uint8_t FatPixTR[8] = { fatR, fat0 }; // char(1) Top-Right
//                   00111
//                   00111
//                   00111
//                   00111
//                   00000
//                   00000
//                   00000
//                   00000
uint8_t FatPixBL[8] = { fat0, fatL }; // char(2) Bottom-Left
//                   00000
//                   00000
//                   00000
//                   00000
//                   11100
//                   11100
//                   11100
//                   11100
uint8_t FatPixBR[8] = { fat0, fatR }; // char(3) Bottom-Right
//                   00000
//                   00000
//                   00000
//                   00000
//                   00111
//                   00111
//                   00111
//                   00111
//
// Define GRAPHICS CHARS 4..7:
//
// It is also possible to make use of pairs of fatL and fatR fragments
// adjacent to each other. Let a "fatU" be 4 slices with all pixels
// set. Note: changed fragment height to 3 for cleaner layout.
#define fatU         B11111, \
                     B11111, \
                     B11111, \
                     0        // changed this 0 and next 0 from B11111 for fragment height==3
#define fatD         0, \
                     B11111, \
                     B11111, \
                     B11111
// It is now possible to also assemble the 4 different pairs of quarter
// parts which form full tops, bottoms, left sides, and right sides:
uint8_t FatPixT_[8] = { fatU, fat0 }; // char(4) Top
//                   11111
//                   11111
//                   11111
//                   11111
//                   00000
//                   00000
//                   00000
//                   00000
uint8_t FatPixB_[8] = { fat0, fatD }; // char(5) Bottom
//                   00000
//                   00000
//                   00000
//                   00000
//                   11111
//                   11111
//                   11111
//                   11111
uint8_t FatPix_L[8] = { fatL, fatL }; // char(6) Left
//                   11100
//                   11100
//                   11100
//                   11100
//                   11100
//                   11100
//                   11100
//                   11100
uint8_t FatPix_R[8] = { fatR, fatR }; // char(7) Right
//                   00111
//                   00111
//                   00111
//                   00111
//                   00111
//                   00111
//                   00111
//                   00111
//
// Note: the HD44780 already has defined chars for all pixels cleared or
//       set i.e. char(0x20) = all pixels clear, char(0xff) = all pixels
//       set. For convenience:
#define pCLR 0x20                  // 8x 00000 rows
#define pSET 0xff                  // 8x 11111 rows
#define pLOW 0x4f

// loadFatDigit_parts() loads digit fragments into the HD44780 graphics RAM
//
void loadFatDigit_parts()
{
  lcd.createChar(0, FatPixTL);
  lcd.createChar(1, FatPixTR);
  lcd.createChar(2, FatPixBL);
  lcd.createChar(3, FatPixBR);
  lcd.createChar(4, FatPixT_);
  lcd.createChar(5, FatPixB_);
  lcd.createChar(6, FatPix_L);
  lcd.createChar(7, FatPix_R);
}
//
// Given the new 8 graphics chars, we can now define HEX chunky chars which
// fit into a 4x4 set of chars using the 8 FatPix chars defined above, and
// also using all-clear (pCLR) and all-set (pSET) graphics chars.  Each
// char is divided into quarters so in effect we obtain an 8x8 segment
// display.
#define FatWidth  4
#define FatHeight 4		// should be >= nROWS
// A "fat" digit contains 16 characters which are stored as 16 char values
// equal 4 rows of 4 cols i.e.
//    0  1  2  3
//    4  5  6  7
//    8  9 10 11
//   12 13 14 15
// Given that the display has geometry 20 columns, we can specify the first
// column for each fat char and colon over this width:
#define column_d1	time_column1			// hours, tens
#define column_d2	(column_d1+1*FatWidth)		// hours, units
#define column_colon	(column_d1+2*FatWidth)		// colon
#define column_d3	(column_d1+2*FatWidth+1)	// minutes, tens
#define column_d4	(column_d1+3*FatWidth+1)	// minutes, units
//
static const uint8_t columnsFAT[] = {column_d1,column_d2,column_colon,column_d3,column_d4};
//
// Specify location for normal (non-FAT) seconds display
#define column_d5	(nCOLS-2)		// right most 2 columns

// writeFatDigit( int lcd_column, int value )
//   write value as 4 rows of FAT digit char sections at lcd_column
//
void writeFatDigit(int lcd_column, int value) {
  // define the 4x4=16 chars that form each "fat" char for digits '0'..'9'
  // (0..7 refer to user defined chars, other values refer to existing HD44780 font)
  static const uint8_t digits[10][16] = {
    { 3,4,4,2,          6,pCLR,pCLR,7,    6,pCLR,pCLR,7,     1,5,5,0 },        // '0'
    { pCLR,pCLR,3,6,    pCLR,pCLR,pCLR,6, pCLR,pCLR,pCLR,6, pCLR,pCLR,7,pSET },// '1'
    { 3,4,4,2,          pCLR,pCLR,5,0,    3,4,pCLR,pCLR,    7,5,5,2 },         // '2'
    { 3,4,4,2,          pCLR,pCLR,3,0,    pCLR,pCLR,1,2,    1,5,5,0 },         // '3'
    { 7,pCLR,pCLR,pCLR, 7,pCLR,3,pCLR,    pCLR,4,pSET,4,    pCLR,pCLR,0,pCLR },// '4'
    { 7,4,4,0,          7,5,5,pCLR,       pCLR,pCLR,pCLR,6, 3,5,5,0 },         // '5'
    { 3,0,pCLR,pCLR,    6,pCLR,pCLR,pCLR, 6,4,4,2,          1,5,5,0 },         // '6'
    { 3,4,4,2,          pCLR,pCLR,pCLR,6, pCLR,pCLR,pCLR,6, pCLR,pCLR,pCLR,6 },// '7'
    { 3,4,4,2,          1,5,5,0,          7,pCLR,pCLR,6,    1,5,5,0 },         // '8'
    { 3,4,4,2,          6,pCLR,pCLR,7,    pCLR,4,4,7,       pCLR,pCLR,pCLR,7 } // '9'
  };

  // HEX definitions and other character definitions can be added to above
  // table,
  // taking into consideration the following input mappings for convenience:
  //
  if (value == ' ')
    value = -1;                             // map ' ' to blank pCLR
  if (value >= '0' && value <= '9')
    value = value - '0';                    // '0'..'9'==>0..9 offset into digits[]

  int cnt = 0;
  for (int row = 0; row < FatHeight; row++) // for row=0..3
  {                                         //
    lcd.setCursor(lcd_column, row);         // move cursor to first column
    for (int i = 0; i < FatWidth; i++)      // then write 4 successive columns
      lcd.write(value < 0 ? pCLR : digits[value][cnt++]);
  }
}

// writeFatColon() rewrites colon at column for flashing
//
// Note: the HD44780 has two colon-like char predefined. Both are used.
#define pCOL1 0xa1
#define pCOL2 0xdf
void writeFatColon(int column, int is_set) {
  uint8_t a, b;

  if (is_set) {
    a = pCOL1;
    b = pCOL2;
  } else {
    a = b = pCLR;
  }

  lcd.setCursor(column, 1);
  lcd.write(a);
  lcd.setCursor(column, 2);
  lcd.write(b);
}

#endif

///////////////////////////////////////////////////////////////////////////
                                    // Run timer2 at 1/4 ms interrupt period
const int delta_time = 250;         //   micro-seconds per interrupt
const int counts_per_second = 4;    //   interrupt counts per second

volatile int timer_event_counter;   //   counts timer2 events
volatile int time2update;           // token to SET after each second interval

// Interrupt routine interrupt_timer2overflow() is triggered by timer2
// overflow
// - should be kept to a minimum length e.g. increment flags like
//   time2update to delegate large work to other "helper" tasks, while
//   doing simple LED flash work here
void interrupt_timer2overflow() {
  static boolean output = HIGH;

  if (++timer_event_counter == counts_per_second) {
    timer_event_counter = 0;        // clear event counter
    ++time2update;                  // set token to increment time by 1 second
    digitalWrite(LED_flash, output);// update flashing LED inside this interrupt
    output = !output;
  }
}

//
// Define constants and variables for time display
//
// Variables to hold time
struct Time_rep {
  int hours, minutes, seconds;
};
Time_rep the_time;
//
// Variables for time display
#define time_slen 12
char time_str[time_slen];       // new computed value
char lcd_data[time_slen];       // value currently on LCD

void init_timeKeeping() {
                                // set all time keeping vars to 0
  the_time.hours = the_time.minutes = the_time.seconds = 0;
  time2update = timer_event_counter = 0;

  for (int i = 0; i < strlen(time_str); i++)
    time_str[i] = lcd_data[i] = 0;
  update_timeDisplay();         // display current time

  lcd.setCursor(nCOLS - strlen(VersionCLK), 0);
  lcd.print(VersionCLK);        // display version ID
}

// increment internal time representation by 1 second
//
void increment_time() {
  time2update = 0;              // clear token signalling this increment

  if ( ++the_time.seconds < 60 ) return;
  the_time.seconds=0;
  if ( ++the_time.minutes < 60 ) return;
  the_time.minutes=0;
  if ( ++the_time.hours < 24 )   return;
  the_time.hours=0;              return;
};

// update_timeDisplay() updates the LCD and serial port displays
// - to minimise I2C bus use, maintain a local copy of
//   current LCD data in lcd_data and update only those
//   digits i for which time_str[i] != lcd_data[i]
//
void update_timeDisplay() {
  char *x;

  sprintf(time_str, "%2d:%02d:%02d", the_time.hours, the_time.minutes,
          the_time.seconds);

#if defined(enableFatDigits)            // fat char mode means individual digit output
  for (int i = 0; i < 5; i++)           // display 1st 5 chars as FAT
    if (time_str[i] == ':')
      writeFatColon(columnsFAT[i], the_time.seconds & 1);
    else if (time_str[i] != lcd_data[i])// send only values that change, update copy
    {
      writeFatDigit(columnsFAT[i], time_str[i]);
      lcd_data[i] = time_str[i];
    }
  lcd.setCursor(column_d5, nROWS - 1);
  x = time_str + 6;                     // remaining text displayed as normal chars
#else
  lcd.setCursor(time_column1, nROWS - 1);
  x = time_str;                         // all text displayed as normal chars
#endif
  lcd.print(x);

  Serial.print(time_str);               // echo time to console
  Serial.print("\n");
}

///////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(57600);
  pinMode(LED_flash, OUTPUT);

                                // activate LCD module
  lcd.begin(nCOLS, nROWS);
  lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
  lcd.setBacklight(HIGH);
  delay(100);

#if defined(enableFatDigits)    // fat char mode means individual digit output
  loadFatDigit_parts();         // for which custom graphics are loaded
#endif

  init_timeKeeping();
  delay(100);

  MsTimer2::set(delta_time, interrupt_timer2overflow);
  MsTimer2::start();
}

void loop() {
  int x;
  static byte cmd[3];           // user input takes the form of 3 char sequences

  if (time2update != 0) {
    increment_time();
    update_timeDisplay();
  }

  // Interpret the 3 most recent serial port chars as user commands of the
  // forms
  //   argument_digit_tens argument_digit_units command_char
  // or
  //   command_char
  // when there are no arguments needed. Maintain cmd[] as a sliding window
  // holding the recent serial port chars.
  //
  while (Serial.available()) {
    cmd[0] = cmd[1];            // update sliding window of 7bit user keystrokes
    cmd[1] = cmd[2];
    cmd[2] = 0x7f & Serial.read();

                                // update value of any argument_digits
    x = 10 * (cmd[0] - '0') + (cmd[1] - '0');
    if (x > 99 || x < 0)
      x = 0;

    switch (cmd[2])             // decode command_char
    {
    case 'c':
      init_timeKeeping();
      Serial.print("[time cleared]\n");
      break;
    case 'm':
      the_time.minutes = x;     // (setting minutes will clear seconds)
      the_time.seconds = timer_event_counter = 0;
      Serial.print("[M]\n");
      break;
    case 'h':
      the_time.hours = x;
      Serial.print("[H]\n");
      break;
    }
  }
}

///////////////////////////////////////////////////////////////////////////
