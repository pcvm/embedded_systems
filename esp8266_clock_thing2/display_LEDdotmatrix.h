/* -*- mode: c++; -*- */

// File display_LEDdotmatrix.h is based on sample code taken from sample
// program MD_MAX72xx_PrintText.ino
//
// The LED dot matrix driver object is instantiated as object mx, and then the
// "setup" function is mx.begin(). There is also a "loop" function mx_loop().

void mx_loop();

// Use the MD_MAX72XX library to Print some text on the display
//
// Demonstrates the use of the library to print text.
//
// User can enter text on the serial monitor and this will display as a
// message on the display.

#include <MD_MAX72xx.h>
#include <SPI.h>

#define PRINT(s, v) { Serial.print(F(s)); Serial.print(v); }

// Define the number of devices we have in the chain and the hardware interface
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4 // 11

#define CLK_PIN   D5 // or SCK
#define DATA_PIN  D7 // or MOSI
#define CS_PIN    D8 // or SS

// SPI hardware interface
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

// Text parameters
#define CHAR_SPACING  1 // pixels between characters

// Global md_message buffers shared by Serial and Scrolling functions
#define BUF_SIZE  14
char md_message[BUF_SIZE] = "Hello!";
bool newMessageAvailable = true;

void load_md_message(char flag1, char flag2, char * msg, int cnt) {
  md_message[0] = flag1;
  md_message[1] = flag2;
  if (cnt > (BUF_SIZE-3)) cnt=BUF_SIZE-3;
  strncpy(&md_message[2], msg, cnt);	// copy only cnt chars
  newMessageAvailable = true;
  mx_loop();
}
void mdstr( char *msg ) {
  int cnt=strlen(msg);
  if (cnt>BUF_SIZE-1) cnt=BUF_SIZE-1;
  strncpy( md_message, msg, cnt);
  while (cnt<BUF_SIZE) md_message[cnt++] = ' ';
  newMessageAvailable = true;
  mx_loop();
}
void mdstr_merge1( char x ) {
  md_message[0] = x;
  newMessageAvailable = true;
  mx_loop();
}

void printText(uint8_t modStart, uint8_t modEnd, char *pMsg)
// Print the text string to the LED matrix modules specified.
// Message area is padded with blank columns after printing.
{
  uint8_t   state = 0;
  uint8_t   curLen = 0;
  uint16_t  showLen = 0;
  uint8_t   cBuf[8];
  int16_t   col = ((modEnd + 1) * COL_SIZE) - 1;

  mx.control(modStart, modEnd, MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);

  do     // finite state machine to print the characters in the space available
  {
    switch(state)
    {
      case 0: // Load the next character from the font table
        // if we reached end of md_message, reset the md_message pointer
        if (*pMsg == '\0')
        {
          showLen = col - (modEnd * COL_SIZE);  // padding characters
          state = 2;
          break;
        }

        // retrieve the next character form the font file
        showLen = mx.getChar(*pMsg++, sizeof(cBuf)/sizeof(cBuf[0]), cBuf);
        curLen = 0;
        state++;
	// fall through

      case 1: // display the next part of the character
        mx.setColumn(col--, cBuf[curLen++]);

        // done with font character, now display the space between chars
        if (curLen == showLen)
        {
          showLen = CHAR_SPACING;
          state = 2;
        }
        break;

      case 2: // initialize state for displaying empty columns
        curLen = 0;
        state++;
        // fall through

      case 3:	// display inter-character spacing or end of message padding (blank columns)
        mx.setColumn(col--, 0);
        curLen++;
        if (curLen == showLen)
          state = 0;
        break;

      default:
        col = -1;   // this definitely ends the do loop
    }
  } while (col >= (modStart * COL_SIZE));

  mx.control(modStart, modEnd, MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
}

//void mx_setup()
//{
//  mx.begin();
//}

void mx_loop()
{
  if (newMessageAvailable)
  {
    //    PRINT("\nProcessing new message: ", md_message);
    printText(0, MAX_DEVICES-1, md_message);
    newMessageAvailable = false;
  }
}

// Simple probe of first 8x8 module to confirm that SPI+DM hardware is present
// . requires that the SPI connection has MISO for rx (as well as MOSI for tx)
//
void delay_with_time_keeping(int);
//
bool confirm_LED_matrix_present(int ms) {
  ms = ms / (8 * 2);
  for (unsigned int cnt=0; cnt<8; cnt++) {
    mx.setPoint( cnt, cnt, true);
    delay_with_time_keeping(ms);
    if (true  != mx.getPoint( cnt, cnt )) return false;
    mx.setPoint( cnt, cnt, false);
    delay_with_time_keeping(ms);
    if (false != mx.getPoint( cnt, cnt )) return false;
    mx.setPoint( cnt, cnt, true);
  }
  return true;
}
