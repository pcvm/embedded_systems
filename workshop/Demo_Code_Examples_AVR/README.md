# Demo_Code_Examples_AVR


* ablink/
   - Simply blinks the built-in LED. Based on the Arduino examples Blink.
   - Shows: initialisation of pin 13 (port B 5) as OUTPUT (IO lines default to input after reset), setting and clearing pin 13 with 1000ms (1s) delays.
   - Introduces functions setup() and loop().

* counter/
   - Provides a 2 bit counter where the AVR drives two LEDs. You need to connect the LEDs to the chosen pins via a current limiting resistor.
   - Shows: initialisation of 2 port B lines as OUTPUT (IO lines default to input after reset), the use of integer variable for counting (declaration, initialisation in setup, incrementing in main loop), and testing bits 0,1 in myCounter to set/clear port B bits 0,1.
   - Introduces functions pinMode(), digitalWrite(), delay() along with setup() and loop().

* counter_b3/
   - Extend the above example to drive 3 LEDS for a 3 bit counter.

* counter_b3_sw/
   - Extend the above example to drive 3 LEDS for a 3 bit counter, and now add an input switch so that the counter direction can be chosen.

* counter_b3_sw_directPB/
   - Make the output port update more efficient by directly manipulating the bits with a single statement (instead of multiple bit inspect and digitalWrite() calls).

* detect_pulse_adc/
   - Configure the ADC convertor to read a voltage from a light transmission sensor which contains a light source (LED) and light receiver that senses small changes in light transmission through a finger etc. due to the blood ressure pulse in a heartbeat.
   - The code reads the light transmission sensor attached to pin SENSOR_pin_adc and  smooths it via an exponential decay filter 
            sensorValue = alpha * oldSensorValue + (1-alpha) * rawSensorValue;
     and a pulse is detected when previous change and current change indicate a  turning point.
   - Based on code from https://tkkrlab.nl/wiki/Arduino_KY<C2><AD>039_Detect_the_heartbeat_module

* echo_audio/
   - Introduces use of the LCD driver attached to the I2C bus.
   - Shows ADC data being read, averaged, and displayed on the LCD.

* echo_audio_bar_display/
   - As above but now a bar graph is also generated for the LCD.

* LCD_Clock/
   - Uses MsTimer2.zip (time keeping via timer2 overflow interrupts) and NewliquidCrystal_1.3.4.zip (LCD driver).
   - The 4 row 20 column LCD is used in grapical mode so that large "chunky" segments are drawn to provide large 7-segment display (well, this is one way to show 7-segment data being defined, as well as the chunky graphics chars for the Hitachi LCD).

* configure_LCD/
   - Some include file examples for the LCD.

* Investigating_IDE.txt
   - Shell commands to detect and display handy AVR information.
