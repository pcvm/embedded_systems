/*
 * detect_pulse_adc
 *   reads the light transmission sensor attached to pin SENSOR_pin_adc and
 *   smooths it via an exponential decay filter
 *                    sensorValue = alpha * oldSensorValue + (1-alpha) * rawSensorValue;
 *   A pulse is detected when previous change and current change indicate a
 *   turning point.
 *
 * To do:
 * . experiment with values for alpha
 * . add a downcount debouncer so detection of new pulses is prevented
 *   immediately after a new pulse
 * . replace the use of the relatively slow serial port for checking the
 *   sensorValue with a call to lcd.print (see the definitions and setup
 *   in directory examples/LCD)
 *
 * The original code was found online at
 *   https://tkkrlab.nl/wiki/Arduino_KY­039_Detect_the_heartbeat_module
 */

///////////////////////////////////////////////////////////////////////////

// Define signal names in terms of standard Arduino board pin numbers
#define LED_pin         13	// pin 13
#define SENSOR_pin_adc  A0	// adc0	

const double alpha_forgettingFactor = 0.75;
const int sampling_period = 20;	// ms delay between sampling

// initialise IO ports
void setup() {
  // initialize digital LED_pin as an output
  pinMode(LED_pin, OUTPUT);
  // set UART to 57600 baud
  Serial.begin(57600);
}

// main loop
// - filter sensor data looking for a turning point that indicates a pulse
// - display sensor data on console
char status_msg[20];
void loop() {
  static double oldSensorValue = 0.0;
  static double oldChange      = 0.0;
  static double change = 0.0;
  static int debounce = 0;

  // read sensor and smooth sensorValue
  int rawSensorValue = analogRead(SENSOR_pin_adc);
  double sensorValue = alpha_forgettingFactor     * oldSensorValue +
                       (1-alpha_forgettingFactor) * rawSensorValue;

  // calculate change and detect turning point
  // (previous change < 0 and this change > 0 means minimum turning point found)
  change = sensorValue-oldSensorValue;
  int is_pulse = change<0.0 && oldChange>0.0;
  digitalWrite(LED_pin,is_pulse);
  
  // maintain history for next update, and wait before next sampling
  oldSensorValue = sensorValue;
  oldChange      = change;
  delay(sampling_period);

  sprintf(status_msg,"%d\n", (int) sensorValue);
  Serial.print( status_msg );
  //  lcd.setCursor( 0,0 ); lcd.print( rawSensorValue );
  //  lcd.print( ", ");     lcd.print( (int) sensorValue );
}
