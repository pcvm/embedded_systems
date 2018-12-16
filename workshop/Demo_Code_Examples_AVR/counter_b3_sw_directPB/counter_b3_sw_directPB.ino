/*
 * Counter
 *   Turns an LED on for one second, then off for one second, repeatedly.
 *   Also maintains a counter and displays the least significant bits on port B bits 0,1 
 *   Derived from blink.ino contributed to arduino by Scott Fitzgerald
 *
 * Shows:
 *   initialisation of 2 port B lines as OUTPUT (IO lines default to input after reset),
 *   use of integer variable for counting (declaration, initialisation in setup, incrementing
 *     in main loop), and
 *   testing bits 0,1 in myCounter to set/clear port B bits 0,1
 *
 * Introduces functions pinMode(), digitalWrite(), delay() along with setup&loop
 *                      digitalRead()
 */

// NOW EXTENDED TO 3-BIT COUNTING ON PORT B 0-2, WITH
// COUNTING DIRECTION DEPENDING ON PORT B 3
// USING DIRECT PORT B WRITES

// Define signal names in terms of standard Arduino board pin numbers
// (defining pin names and similar constants means no unnecessary const numbers
// are littering the program which is usually then easier to read and check)
#define LED_pin 13
#define COUNTER_b0 8
#define COUNTER_b1 9
#define COUNTER_b2 10
#define MASK_count_b0 1
#define MASK_count_b1 (1 << 1)
#define MASK_count_b2 (1 << 2)

// Define input signals
#define SWITCH_b3  11

// Constant and variable declarations
int myCounter;

// The setup function runs once when you press reset or power the board
void setup() {
  // initialise output pins' data direction
  pinMode(LED_pin, OUTPUT);
  pinMode(COUNTER_b0, OUTPUT);
  pinMode(COUNTER_b1, OUTPUT);
  pinMode(COUNTER_b2, OUTPUT);
  // Note: these now output pins will have value=0 after reset.

  // initialise program variables e.g. counters
  myCounter = 0;
}

// The loop function runs over and over again forever
void loop() {
  digitalWrite(LED_pin, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                  // wait for a second
  digitalWrite(LED_pin, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);                  // wait for a second

  // add code for counter update and display
  if (digitalRead(SWITCH_b3))   // (note: not debouncing as this loop has large delays)
    myCounter++;
  else
    myCounter--;
  // individual bits are tested in myCounter then corresponding port B bit
  // set/cleared
  // - direct write to PORTB
  PORTB = myCounter & 7; // mask LS 3 bits
}
