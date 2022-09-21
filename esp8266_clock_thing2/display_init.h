/* -*- mode: c++; -*- */

// Display startup sequencing (for enabled display drivers)
// . it is possible to drive both LED displays simultaneously so the
//   initialisation sets up both before one is chosen for time display
// . some DotMatrix specific code is replicated in 7SEGMENT / !7SEGMENT
//   code sections so that message timing below is better for an observer
//   (see comment string "//.")

#if defined( support_DotMatrix_DISPLAY )
  mx.begin();
  mx.clear();
  Serial.println(" LED matrix driver initialised");
  mdstr( (char*) "  Hello" );
#endif

#if defined( support_7SEGMENT_DISPLAY )
						//   initialise LED 7-segment
  the7segDisplay.begin(AddressOf7segDisplay);
  the7segDisplay.clear();
  the7segDisplay.writeDisplay();
  setDisplayBrightnessMax();

  ticker_7seg_str4( (char*)My_Hello,'h',250,2500 ); // display hello message+version number, test digital display
  
  #if defined( support_DotMatrix_DISPLAY )		//.
  mdstr( (char*) (String("  ") + ThisRelease).c_str());	//.(code fragment for DotMatrix is replicated below for ! 7SEGMENT)
  #endif						//.

  Serial.println(" LED 7-segment driver");

  if (led_display_is==using_7seg_led) {		// 7-segment LED display segment test only run if display enabled
    char segtest_string[5] = {' '};
    for (int p=3; p>=0; p--) {
      char * ptr = (char*) segment_test_cycle;	// = "1_-^|";
      while (*ptr != 0) {
	segtest_string[p] = *ptr++;
	display_7seg_str4(0,segtest_string);
	delay_with_time_keeping( 100 );
      }
      segtest_string[p] = ' ';
    }
    display_7seg_str4(0,segtest_string);
  }
  for (int p=0;p<8;p++) {			//   (this tests the status bits display)
    display_7segmentSetFlags( 1<<p );
    delay_with_time_keeping( 100);
  }

#else
  #if defined( support_DotMatrix_DISPLAY )		//.
  mdstr( (char*) (String("  ") + ThisRelease).c_str());	//.(code fragment for DotMatrix)
  delay_with_time_keeping( 1000 );			//.
  #endif
#endif

#if defined( support_DotMatrix_DISPLAY )
  mdstr( (char*) (String( index(ThisRelease,' ') ).c_str()) );
  delay_with_time_keeping( 500 );
#endif
