/* -*- mode: c++; -*- */

// Definitions for the I2C attached 4-digit 7-segment LED "backpack" from AdaFruit

#define n7seg_digits 4
#define n7seg_digitsM1 (n7seg_digits-1)

#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>

// set Adafruit driver address
#define AddressOf7segDisplay 0x70

// Instantiate one display (Adafruit 7-segment driver backpack)
Adafruit_7segment the7segDisplay = Adafruit_7segment();

// define some 7-segment activity functions that create a rotating
// individual segment pattern around the outer segments of the LHS
// digit, one for negative rotation and one for positive
uint8_t active_a=0;
void display_7seg_activity_neg() {
  active_a = 0xff & (active_a << 1);
  if ((active_a==0)||(active_a==64)) active_a=1;
  the7segDisplay.writeDigitRaw(0,active_a);
  the7segDisplay.writeDisplay();
}
void display_7seg_activity_pos() {
  active_a = 0xff & (active_a >> 1);
  if (active_a==0) active_a=32;
  the7segDisplay.writeDigitRaw(0,active_a);
  the7segDisplay.writeDisplay();
}
void display_7seg_activity_clear() {
  active_a = 0;
  the7segDisplay.writeDigitRaw(0,active_a);
  the7segDisplay.writeDisplay();
}

// display_7seg_progress_update() provides a rotating segment in the left
//                                most digit with regular direction change
unsigned char _progress_cnt=0;
void display_7seg_progress_update() {
  _progress_cnt++;
  if ( 8 & _progress_cnt ) display_7seg_activity_pos();
  else                     display_7seg_activity_neg();
}

//
// Managing individual segments, decimal points, colons, etc.
//
				// To set colon and decimal points, use
				// writeDigitRaw(location, bitmap) with location=2
				// and bitmap values
#define StatusBits7seg_cc 0x02	// - center colon
#define StatusBits7seg_td 0x04	// - left colon, top dot
#define StatusBits7seg_bd 0x08	// - left colon, bot dot
#define StatusBits7seg_dp 0x10	// - decimal point
				// With only colon bits in use, use
				// the7segDisplay.drawColon(boolean) and then
				// the7segDisplay.writeColon() to transmit bits
				//
				// To control individual bits, use macros to set/clear bits
				// and then call the7segDisplay.writeColon() to transmit bits
#define StatusBits7seg_setBits( x ) the7segDisplay.displaybuffer[2] |= (x)
#define StatusBits7seg_clrBits( x ) the7segDisplay.displaybuffer[2] &= ~(x)
#define StatusBits7seg_initialise   the7segDisplay.displaybuffer[2] &= 0

// Clock use of indicator "dots"
// - use upper dot to flag an NTP sync is in progress (the display is
//   actually upside down so this is in fact the lower left dot!)
void flag_NTP_sync_active() { StatusBits7seg_setBits( StatusBits7seg_bd ); }
void flag_NTP_sync_done()   { StatusBits7seg_clrBits( StatusBits7seg_bd ); }

// Brightness can be set 0..15, 15=default=MAX
#define max7segBrightness 15
#define mid7segBrightness  7
#define min7segBrightness  0
unsigned char the7seg_current_brightness = 15;

typedef struct _display_intensity {	// actually setting a specific brightness in a time range but the
  signed char brightness;		// user can think of it as dimming since default brightness = max
  signed char start_hour;
  signed char stop_hour;
  signed char current_value;
} display_intensity;
display_intensity LEDctl = {-1,-1,-1,-1};

unsigned char setDisplayBrightness( int new_value ) {
  if (new_value<min7segBrightness) new_value=min7segBrightness;
  if (new_value>max7segBrightness) new_value=max7segBrightness;
  if (LEDctl.current_value!=new_value) {
    LEDctl.current_value = new_value;	// maintain local copy
    the7segDisplay.setBrightness(LEDctl.current_value);
  }
  return LEDctl.current_value;
}
void setDisplayBrightnessMax() {
  setDisplayBrightness(max7segBrightness);
}
void setDisplayBrightnessP( Stream *console, int new_value ) {
  console->printf("\nDisplay brightness = %d\n\n", setDisplayBrightness(new_value));
}
int getDisplayBrightness() {
  return LEDctl.current_value;		// use local copy
}
int manage_display_brightness(int hours) {
  unsigned char hstart=LEDctl.start_hour;
  unsigned char hstop =LEDctl.stop_hour;
  if ( hstop < hstart ) hstop += 24;	// wrap earlier start time over into the 24... range, and
  if ( hours < hstart ) hours += 24;	// also wrap early time over as it might be in the wrapped active zone,
					// so we can now do a simple in range test

			    //  __________true_if_active__________
  unsigned char value_wanted = (((hours<hstop) && (hours>=hstart)) ? LEDctl.brightness : max7segBrightness);
  if (value_wanted != LEDctl.current_value)
    setDisplayBrightness(value_wanted);
  return LEDctl.brightness;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

// (note: should make these support functions for time_s12 and have PM_remove/restore_12HFLAG() fns)
#define PM_ENCODE_12HFLAG( x ) ( (x) | 0x80      )
#define PM_REMOVE_12HFLAG( x ) ( (x) & 0x7f      )
#define IS_PM_12HFLAG_SET( x ) (((x) & 0x80) != 0)

// display_7seg_4digits() displays the time string data on a 7seg 4digit display
//
#define GET_BCD_BITS(x) (0x0f & (x))    // code clarity
//
void display_7seg_4digits( char* day_time, int the_colon, int digitsChange )  {
  if (digitsChange) {                   // new data means update Adafruit internal buffer and transmit
    // draw colon
    if (the_colon) StatusBits7seg_setBits( StatusBits7seg_cc );
    else           StatusBits7seg_clrBits( StatusBits7seg_cc );

    // day_time[0] is the left most char encoded with flag to display PM
    if (IS_PM_12HFLAG_SET(day_time[0])) StatusBits7seg_setBits( StatusBits7seg_td );
    else                                StatusBits7seg_clrBits( StatusBits7seg_td );

    // day_time[0] may also be omitted when it is blank (leading 0 was suppressed)
    if (PM_REMOVE_12HFLAG(day_time[0]) ==' ') the7segDisplay.writeDigitRaw(0, 0);
    else                                      the7segDisplay.writeDigitNum(0, GET_BCD_BITS(day_time[0]));
    the7segDisplay.writeDigitNum(1, GET_BCD_BITS(day_time[1]));
    the7segDisplay.writeDigitNum(3, GET_BCD_BITS(day_time[3]));
    the7segDisplay.writeDigitNum(4, GET_BCD_BITS(day_time[4]));
    the7segDisplay.writeDisplay();

  } else {				// no new data means only transmit colon to display
    if (the_colon) StatusBits7seg_setBits( StatusBits7seg_cc );
    else           StatusBits7seg_clrBits( StatusBits7seg_cc );
    the7segDisplay.writeColon();
  }
}

//
// display_7seg_NdigitCounter(x)    displays x on the 7seg LED display
//                                  with NdigitCounter7seg active digits
// display_7seg_Mchar_NdigCounter() is like display_7seg_NdigitCounter() but
//                                  allows a leading char to indicate mode
unsigned char NdigitCounter7seg = 4;            // default
void display_7seg_Mchar_NdigCounter(int x, char mode) {
                                                // make display value sane
  if (x<0)					// . non-negative (lower limit)
    x=0;
  else switch (NdigitCounter7seg) {		// . all '9' chars for specific n digits (upper limit)
    case 4: if (x>9999) x=9999; break;
    case 3: if (x>999 ) x=999;  break;
    case 2: if (x>99  ) x=99;   break;
    case 1: if (x>9   ) x=9;    break;
    }

  char x_str[12];
  if ( mode != 0) x_str[0]=mode;
  sprintf(x_str,"%4d",x);
  display_7seg_str4(x_str);
}

void display_7seg_NdigitCounter(int x) {
  display_7seg_Mchar_NdigCounter( x, 0);
}

// display_7seg_NdigitCounterInit() should be called for each use
//                                  of display_7seg_NdigitCounter()
void display_7seg_NdigitCounterInit(int ndigits) {
  if (ndigits>4) ndigits=4;                     // limit ndigits to sane value
  else if (ndigits<1) ndigits=1;
  StatusBits7seg_clrBits( StatusBits7seg_cc );

  NdigitCounter7seg = ndigits;
  display_7seg_str4((char*) "    ");
}

void display_7segmentSetFlags( unsigned char flagsBits )  {
  the7segDisplay.displaybuffer[2] = flagsBits;
  the7segDisplay.writeColon();
}

//void Adafruit_7segment::drawColon(boolean state) {
//  if (state)
//    displaybuffer[2] = 0x2;
//  else
//    displaybuffer[2] = 0;
//}

#include "display_font_data.h"

// load_7seg_char() updates an internal buffer but does not update the display
//                  Uses our special "7-seg" ascii font in mapASCIIto7seg[]
//
void load_7seg_char( int location, char ch ) {
  int x = ch;
  if ((x > '~')||(x < ' ')) x=0;		// map ascii to SAFE table address used here i.e.
  else x = x - ' ';				// {' '..'~'} is mapped to {0..('~'-' ')}, and anything else becomes 0

  location = 0x3 & location;			// map 0,1,2,3 to 0,1,3,4 (as used by display)
  if (location>1) location += 1;

						// handle special mappings
  if (ch == '\t') {
    the7segDisplay.writeDigitRaw(location, 0x36);	// double-elle (bits not reversed)
    return;
  }

  the7segDisplay.writeDigitRaw(location, mapASCIIto7seg[x]);	// x in a SAFE table address
}

// display_7seg_str4() prints up to 4 chars, left justified
void display_7seg_str4( char* ptr ) {
  //  the7segDisplay.clear();	// clear buffer
  for (int p=0; p<4; p++){	// load buffer
    if (*(ptr+p) == 0) break;		// (load up to 4 chars)
    load_7seg_char( p, *(ptr+p) );
  }
  the7segDisplay.writeDisplay();
}

// ticker_7seg_str4() provides a ticker tape text display
void delay_with_time_keeping(int);
//
const int MAX_LENGTH_TICKER_STR = 40;
void ticker_7seg_str4( char* msg, char marker, int delay_default, int delay_marker ) {
  int hlength = strlen(msg);			// display message scrolling horizontally

  if (hlength>MAX_LENGTH_TICKER_STR)
    hlength = MAX_LENGTH_TICKER_STR;

  for (int p=0;p<hlength-n7seg_digitsM1;p++) {	// with delays
    display_7seg_str4(msg+p);
    if ((marker!=0)&&(*(msg+p)==marker)) delay( delay_marker );  //delay_with_time_keeping(delay_marker);
    else                                 delay( delay_default ); //delay_with_time_keeping(delay_default);
  }
}

// display_7seg_int3() writes a 0..99 integer into the RHS 3 char locations
void display_7seg_int3( int value ) {
  char x[5];		// {' '}, {' ','-'}, {['0'-'9']}, {['0'-'9']}, {NULL}
  if (value >  99) value=99;
  if (value < -99) value=-99;
  sprintf( x, " %3d", value);
  display_7seg_str4( x );
}
