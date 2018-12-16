//
// Dot matrix display of digit chars
//

// define digit char scans
// - these pixel dumps for chars came from a TronixLabs code fragment
//   that has horizontal pixels bit reversed with respect to our needs
//
uint8 digits[][8] = {
  {
    B01110000,  //0
    B10001000,
    B10011000,
    B10101000,
    B11001000,
    B10001000,
    B01110000,
    6,
  },{
    B01000000,  //1
    B11000000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B11100000,
    4,
  },{
    B01110000,  //2
    B10001000,
    B00001000,
    B00010000,
    B00100000,
    B01000000,
    B11111000,
    6,
  },{
    B11111000,  //3
    B00010000,
    B00100000,
    B00010000,
    B00001000,
    B10001000,
    B01110000,
    6,
  },{
    B00010000,  //4
    B00110000,
    B01010000,
    B10010000,
    B11111000,
    B00010000,
    B00010000,
    6,
  },{
    B11111000,  //5
    B10000000,
    B11110000,
    B00001000,
    B00001000,
    B10001000,
    B01110000,
    6,
  },{
    B00110000,  //6
    B01000000,
    B10000000,
    B11110000,
    B10001000,
    B10001000,
    B01110000,
    6,
  },{
    B11111000,  //7
    B10001000,
    B00001000,
    B00010000,
    B00100000,
    B00100000,
    B00100000,
    6,
  },{
    B01110000,  //8
    B10001000,
    B10001000,
    B01110000,
    B10001000,
    B10001000,
    B01110000,
    6,
  },{
    B01110000,  //9
    B10001000,
    B10001000,
    B01111000,
    B00001000,
    B00010000,
    B01100000,
    6,
  },{
    B00000000,  //:
    B11000000,
    B11000000,
    B00000000,
    B11000000,
    B11000000,
    B00000000,
    3,
  },{
    B00000000,  //' '
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    8,
  }
};
const int ch_colon   = 10;
const int ch_space   = 11;
const int nDigits    = 11;  // the ' ' char is not bit reversed!
const int nChars     = 12;

const int Font_PixelsWidth  = 5;// Font data is provided in 5x7 pixel wide form
const int Font_PixelsHeight = 7;

#include <LedControl.h>
const int numDevices =  4;      // number of MAX7219s used

int led_intensity;              // current LED intensity

//                 LedControl(int dataPin, int clkPin, int csPin, int numDevices);
LedControl LedCont=LedControl(SPI_mosi, SPI_sck, SPI_ss, numDevices);  // Note: miso unused

// reset all LED driver chips
void LedCont_reset() {
  led_intensity = 8; // default brightness
  for (int x = 0; x < numDevices; x++) {
    LedCont.shutdown(x, false); // enable MAX72XX (powers up in power-saving mode)
                                // set brightness and clear the display
    LedCont.setIntensity(x, led_intensity);
    LedCont.clearDisplay(x);
  }
}

const int n_vPixels_per_digit = 7;
const int n_hPixels_per_digit = 8;
const int np_per_digit=8;	// square digits/chars have np_per_digit x np_per_digit pixels
void show_digit_scans( char x, uint8 *a_digit ) {
  Serial.print("-> "); Serial.println(x);
  for (int s = 0; s < np_per_digit; s++) {
    Serial.print("   ");
    for (int t = 0; t < np_per_digit; t++)
      if ((1<<(np_per_digit-t)) & a_digit[s]) Serial.print("*");
      else                                    Serial.print(" ");
    Serial.println();
  }
  Serial.println();
}

// reset displays and normalise font data
void LedCont_setup() {
  LedCont_reset();              // reset all LED driver chips

  // Reverse the horizontal char pixel scans i.e. reflect
  // through the Y axis. We could avoid this at run-time
  // but RAM is abundant and time does not matter...

                                    // Bit reverse each horizontal scan of each char
  for (int c=0; c<nDigits; c++) {   // . for each digit that is to be bit reversed...
    uint8 *this_digit = digits[c];
    show_digit_scans(char('0'+c), this_digit);

                                    // . for each horizontal scan (or slice) of pixels...
    for (int s = 0; s < n_vPixels_per_digit; s++) {
      uint8 tmp = 0;
                                    // . test original bits R-to-L, to control right-to-
      for (int i = 0; i < n_hPixels_per_digit; i++)
        if ((1 << i) & *(this_digit + s))
          tmp = (tmp << 1) | 1;     // left shift in a 1, or
        else
          tmp = tmp << 1;           // left shift in a 0
      // perform xtra left shifts to center char, based on the
      // width of scanned char stored in digits[c][n_vPixels_per_digit]
      *(this_digit + s) =
        tmp << ((2 + n_hPixels_per_digit - digits[c][n_vPixels_per_digit]) / 2);
    }

#if defined( ROTATE_dm_CHAR )

    // clear the scanned width (located at the position of an 8th 8b scan)
    // after the char is centered and it is of no further use
    // - can now view the digit as occupying the full 8x8 pixels
    digits[c][n_vPixels_per_digit] = 0;	//-pm-

    uint8 dbuffer[np_per_digit];
    for (int s=0;s<np_per_digit;s++) {
      dbuffer[s] = this_digit[s];
      this_digit[s] = 0;
    }
    for (int s=0;s<np_per_digit;s++)
      for (int t=0;t<np_per_digit;t++) {

	// (1<<(s)) vs. (1<<(np_per_digit-1-s)            ==> vertical flip
	// this_digit[np_per_digit-1-t] vs. this_digit[t] ==> horizontal flip
	//
	// choose a rotate by -90 degrees clockwise
	if (dbuffer[s] & (1<<t)) this_digit[t] |= (1<<(np_per_digit-1-s));
      }

#endif

  }

#if defined( ROTATE_dm_CHAR )

  for (int c = 0; c < nDigits; c++) {// . for each digit that is to be bit reversed...
    uint8 *this_digit = digits[c];
    show_digit_scans(char('0'+c), this_digit);
  }

#endif

}

//
// Digital display for clock on a 4-modules of 8x8 dot matrix arrays
// - minimalist code takes in hours/minutes and splits into 4 base 10 digits
// - after leading 0 suppression, each digit has its horizontal pixel scans
//   fed from top to bottom into the array columns
// - use arg colon to enable colon pixels when displaying tmp[1] tens-of-mins

void display_dotMatrix4digits(int dLeft, int dRight, int colon) {
  uint8 *ptr;
  int tmp[4];

  tmp[0] = dRight % 10;
  tmp[1] = dRight / 10;
  tmp[2] = dLeft % 10;
  tmp[3] = dLeft / 10;
  if (tmp[3] == 0)                      // leading hours 0 set to ' '
    tmp[3] = ch_space;
  for (int y = 0; y < 4; y++) {
    ptr = digits[tmp[y]];
    for (int x = 0; x < 7; x++) {       // 7 scans/char
      uint8 slice = *ptr++;

#if defined( ROTATE_dm_CHAR )
      if (y == 1 && (x == 0))		// colon in some reflected universe...
        slice |= (colon ? 0x24 : 0);
#else
      if (y == 1 && (x == 2 || x == 4))	// colon on left of mins-10 digit [y==1]
        slice |= (colon ? 1 : 0);
#endif

      LedCont.setColumn(y, x, slice);
    }
  }
}
