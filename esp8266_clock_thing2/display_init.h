/* -*- mode: c++; -*- */

// Display startup sequencing (for enabled display drivers)
// . it is possible to drive both LED displays simultaneously so the
//   initialisation sets up both before one is chosen for time display
// . the section to perform 7-seg display tests has to deal with each of
//   the 4 digit displays. We use this count of 4 to schedule a slide of
//   the  char* message ThisRelease on the dot matrix display as it is
//   too long to display. A more elegant (and more complex) method would
//   be to implement a text slide function on the dot matrix display...

{
#if defined( support_DotMatrix_DISPLAY )
  mx.begin();
  mx.clear();
  Serial.println("LED dot matrix driver initialised");
  mdstr( (char*) "  Hello" );
  char * md_TR_ptr = (char*) ThisRelease;
#endif

#if defined( support_7SEGMENT_DISPLAY )
  the7segDisplay.begin(AddressOf7segDisplay);	// initialise LED 7-segment
  setDisplayBrightnessMax();
  Serial.println("LED 7-segment driver initialised");

  ticker_7seg_str4( (char*) My_Hello,'h',250,2500 ); // display hello message+version number, test digital display

#if defined( support_DotMatrix_DISPLAY )
  mdstr( md_TR_ptr );
#endif

  // Perform some 7-seg display tests while sliding the dot-matrix release message

  char segtest_string[5] = {' '};
  for (int p=3; p>=0; p--) {			// begin loop to do a segment test for each of 4 LED 7-seg chars
    char * ptr = (char*) segment_test_cycle;	// = "1_-^|";
    while (*ptr != 0) {
      segtest_string[p] = *ptr++;		// light an individual segment if display in use
      if ( is_system_using_7seg_led() ) display_7seg_str4(segtest_string);
      delay_with_time_keeping( 100 );
    }
#if defined( support_DotMatrix_DISPLAY )
    mdstr( md_TR_ptr++ );			// slide the release message (don't care if display not in use)
#endif
    segtest_string[p] = ' ';
    }						// end loop to do segment test
  if ( is_system_using_7seg_led() ) display_7seg_str4(segtest_string);

  for (int p=0;p<8;p++) {			//   (this tests the status bits display)
    display_7segmentSetFlags( 1<<p );
    delay_with_time_keeping( 100);
  }
#else

#if defined( support_DotMatrix_DISPLAY )
  mdstr( md_TR_ptr );
  delay_with_time_keeping( 2500 );
  mdstr( ((char*) ThisRelease) + 4 );
  delay_with_time_keeping( 2500 );
#endif

#endif
}
