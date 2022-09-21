/* -*- mode: c++; -*- */

// Font data for rendering characters (imperfectly) on a 7-segment display

// On a system with only a 4 digit LED segment display, it is helpful to
// support a limited form of text display for messages, even though the
// display is really designed only for digits. We make use of some examples
// on-line (e.g. adaFruit) but then add some extra remapping for special
// characters in function load_7seg_char() e.g. display "ll" (two lower
// case elle chars) as a single digit with both left and both right
// segments set.
//
// Define a (very basic) 7-segment font for ascii chars ' '...'z'
// From https://forum.sparkfun.com/viewtopic.php?t=2143
// but we need bits 1-7 bit reversed so...
#define BR( x ) ( ((x&0x2)<<5) | ((x&0x4)<<3) | ((x&0x8)<<1) | ((x&0x10)>>1) | ((x&0x20)>>3) | ((x&0x40)>>5) | ((x&0x80)>>7))

// The following table encodes the usual 7-segments noted a,b,c,d,e,f,g as
// bits abcdefg
//
//   aa
// f    b
// f    b
//   gg
// e    c
// e    c
//   dd
//
// The Adafruit code uses the reverse bit encoding of g,f,e,d,c,b,a so we
// employ macro BR (for bit reversal) and then re-use an extended version of
// sparkfun 7-seg font data
//
// Note: a rotating set of segments matching top, right, bottom, left is obtained
//       in the char sequence "^1_|"
// Note: a sequential set of segments matching right, bottom, minus/middle, top, left is
const char * segment_test_cycle = "1_-^|";

const unsigned char mapASCIIto7seg[] = {
  BR(0x00), // ' ' 
  BR(0x00), // '!', No seven-segment conversion for exclamation point 
  BR(0x44), // '"', Double quote 
  BR(0x00), // '#', Pound sign 
  BR(0x00), // '$', No seven-segment conversion for dollar sign 
  BR(0x00), // '%', No seven-segment conversion for percent sign 
  BR(0x00), // '&', No seven-segment conversion for ampersand 
  BR(0x40), // ''', Single quote 
  BR(0x9C), // '(', Same as '[' 
  BR(0xF0), // ')', Same as ']' 
  BR(0x00), // '*', No seven-segment conversion for asterix 
  BR(0x00), // '+', No seven-segment conversion for plus sign 
  BR(0x00), // ',', No seven-segment conversion for comma 
  BR(0x02), // '-', Minus sign 
  BR(0x00), // '.', No seven-segment conversion for period 
  BR(0x00), // '/', No seven-segment conversion for slash 
  BR(0xFC), // '0' 
  BR(0x60), // '1', and this is also right vertical segments
  BR(0xDA), // '2' 
  BR(0xF2), // '3' 
  BR(0x66), // '4' 
  BR(0xB6), // '5' 
  BR(0xBE), // '6' 
  BR(0xE0), // '7' 
  BR(0xFE), // '8' 
  BR(0xF6), // '9' 
  BR(0x00), // ':', No seven-segment conversion for colon 
  BR(0x00), // ';', No seven-segment conversion for semi-colon 
  BR(0x00), // '<', No seven-segment conversion for less-than sign 
  BR(0x12), // '=', Equal sign 
  BR(0x00), // '>', No seven-segment conversion for greater-than sign 
  BR(0xCA), //'?', Question mark 
  BR(0x00), // '@', No seven-segment conversion for commercial at-sign 
  BR(0xEE), // 'A' 
  BR(0x3E), // 'B', Actually displayed as 'b' 
  BR(0x9C), // 'C' 
  BR(0x7A), // 'D', Actually displayed as 'd' 
  BR(0x9E), // 'E' 
  BR(0x8E), // 'F' 
  BR(0xBC), // 'G', Actually displayed as 'g' 
  BR(0x6E), // 'H' 
  BR(0x60), // 'I', Same as '1' 
  BR(0x78), // 'J' 
  BR(0x00), // 'K', No seven-segment conversion 
  BR(0x1C), // 'L' 
  BR(0x00), // 'M', No seven-segment conversion 
  BR(0x2A), // 'N', Actually displayed as 'n' 
  BR(0xFC), // 'O', Same as '0' 
  BR(0xCE), // 'P' 
  BR(0x00), // 'Q', No seven-segment conversion 
  BR(0x0A), // 'R', Actually displayed as 'r' 
  BR(0xB6), // 'S', Same as '5' 
  BR(0x1E), // 'T', Actually displayed as 't' 
  BR(0x7C), // 'U' 
  BR(0x00), // 'V', No seven-segment conversion 
  BR(0x00), // 'W', No seven-segment conversion 
  BR(0x00), // 'X', No seven-segment conversion 
  BR(0x76), // 'Y' 
  BR(0x00), // 'Z', No seven-segment conversion 
  BR(0x00), // '[' 
  BR(0x00), // '\', No seven-segment conversion 
  BR(0x00), // ']' 
  BR(0x10), // '^', Carrot ==> top segment
  BR(0x10), // '_', Underscore ==> bottom segment
  BR(0x00), // '`', No seven-segment conversion for reverse quote 
  BR(0xFA), // 'a' 
  BR(0x3E), // 'b' 
  BR(0x1A), // 'c' 
  BR(0x7A), // 'd' 
  BR(0xDE), // 'e' 
  BR(0x8E), // 'f', Actually displayed as 'F' 
  BR(0xBC), // 'g' 
  BR(0x2E), // 'h' 
  BR(0x20), // 'i' 
  BR(0x78), // 'j', Actually displayed as 'J' 
  BR(0x00), // 'k', No seven-segment conversion 
  BR(0x1C), // 'l', Actually displayed as 'L' 
  BR(0x00), // 'm', No seven-segment conversion 
  BR(0x2A), // 'n' 
  BR(0x3A), // 'o' 
  BR(0xCE), // 'p', Actually displayed as 'P' 
  BR(0x00), // 'q', No seven-segment conversion 
  BR(0x0A), // 'r' 
  BR(0xB6), // 's', Actually displayed as 'S' 
  BR(0x1E), // 't' 
  BR(0x38), // 'u' 
  BR(0x00), // 'v', No seven-segment conversion 
  BR(0x00), // 'w', No seven-segment conversion 
  BR(0x00), // 'x', No seven-segment conversion 
  BR(0x76), // 'y', Actually displayed as 'Y' 
  BR(0x00), // 'z', No seven-segment conversion 
  BR(0x00), // '{' 
  BR(0x0C), // '|', Vertical bar ==> left vertical segments
  BR(0x00), // '}'
  BR(0x00)  // '~' 
};
