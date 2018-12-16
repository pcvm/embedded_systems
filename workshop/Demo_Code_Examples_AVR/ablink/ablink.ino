/*
 * ablink is the arduino Blink
 *   Turns an LED on for one second, then off for one second, repeatedly.
 *   Derived from blink.ino contributed to arduino by Scott Fitzgerald
 *
 * Shows:
 *   initialisation of pin 13 (port B 5) as OUTPUT (IO lines default to input after reset),
 *   setting and clearing pin 13 with 1000ms (1s) delays
 *
 * Introduces functions setup() and loop() [and a few others]
 */

// Define signal names in terms of standard Arduino board pin numbers
// (using pin names means no unnecessary const numbers littering the program)
#define LED_pin 13

// The setup function runs once when you press reset or power the board
void setup() {
  // initialize digital LED_pin as an output.
  pinMode(LED_pin, OUTPUT);
}

// The loop function runs over and over again forever
void loop() {
  digitalWrite(LED_pin, HIGH); // turn the LED on (HIGH is the voltage level)
  delay(1000);                 // wait for a second
  digitalWrite(LED_pin, LOW);  // turn the LED off by making the voltage LOW
  delay(1000);                 // wait for a second
}
