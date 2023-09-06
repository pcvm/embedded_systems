///////////////////////////////////////////////////////////////////////////

void daytime_init_report();
void daytime_init_from_gps();
void daytime_init_from_ntp_trigger_stage1();
void sendNTPpacket( IPAddress & );

// reinitialise_daytime_from_source() refers to master_time_source_is status to source new daytime data
//
void reinitialise_daytime_from_source() {
  if ( is_system_using_GPS_source() )
    daytime_init_from_gps();
  else
    daytime_init_from_ntp_trigger_stage1();
}

//
// daytime_init_from_gps() updates time variables with current time from a GPS source
//
//                         . requires that the data retrieval via functions { enable_gps_updates()
//                           background_gps_updates() } is enabled
//
//                         . because the GPS module maintains internal accurate time of day after
//                           satellite contact is made, it is able to provide continues updates of
//                           current time and so we do not need any state machines to handle access
//                           (like when accessing a remote NTP server with fallback servers). There
//                           may be times when it has no satellite visibility but it maintains its
//                           own internal time reference which we can rely on (you can observe this
//                           as it outputs "sentences" of GPS data with a value for daytime but no
//                           location data)
//
void daytime_init_from_gps() {
  fsm_ntp_disable();		// (not necessary but) ensure NTP fsm is OFF
  Serial.println("Time Sync: GPS");
#if defined( support_DotMatrix_DISPLAY )
  mx.clear();
  mdstr( (char*) "GPSread" );
#endif
  enable_gps_updates();		// ensure serial port enabled so GPS data processed
  system_save_last_time_sync_as_GPS();

  it_hours = dt_mins = dt_secs = 0;

  unsigned long starttime = millis();
  unsigned long runtime = 24 + starttime;
  char difg_msg[8] = "GPS ";		// define 4 chars here
  int dt_secs_snapshot = -1;
  unsigned char sync_success = 0;

  while ( (runtime = (millis() - starttime)) < gps_READWAIT ) {
    background_gps_updates();		// includes system call to support its real-time activities

    if ( (runtime & 1023)==0 ) {	// trigger some activity every second
      runtime += 24;			// reinitialise to obtain 1Hz updates (except first time)
      difg_msg[4] = get_next_spinning_wheel();
      difg_msg[5] = 0;			// define other 2 chars here

      Serial.print("\r");
      Serial.print(difg_msg); Serial.print(" : "); Serial.print(ssport1_wabuffer);

#if defined( support_DotMatrix_DISPLAY )
      mx.clear();
      mdstr( (char*) difg_msg );
#endif
    }

    if ( UTC_from_GPS_is_ready_to_retrieve() ) {	// A time sync may be possible
      // Problem: the NEO-6M always reports centi-seconds as 0 and we sync to whole seconds anyway.
      // Solution: for the first new time info available, we take a snapshot of dt_secs and then
      // wait for the next instance of new time available with a change in seconds before syncing.
      if ( dt_secs_snapshot < 0 ) {
	dt_secs_snapshot = get_gps_time(2);		// (first new time info)
      } else if (dt_secs_snapshot != get_gps_time(2)) {
	it_hours = get_gps_time(0);			// (subsequent new time info with changed seconds so centi-secs == 0)
	dt_mins  = get_gps_time(1);			// (and yes we optimistically assume this is the next second of time)
	dt_secs  = get_gps_time(2);
	clock_tick_set_fractional_seconds(0);		// (as sync waited for a change in seconds, clear sub-second count)
	Serial.println("\n\r");
	sync_success = 1;
	break;			// move on with new values
      }
    }

    do_polling_for_active_services();
  }				// fall through if GPS fails with values = 0    

  if ( !sync_success ) {	// no sync so clear display time vars so user sees crazy result
    dt_hours = dt_mins = dt_secs = 0;
    console->printf("GPS sync failed\n\r");
    return;
  }

  update_summer_time(beloud);
  it_hours = it_hours + the_time_zone;
  if (it_hours >= 24)
    it_hours = it_hours - 24;
  dt_hours = it_hours + dt_summer_offset;
  if (dt_hours >= 24) dt_hours = 0;

				// fractional secs == 0 after wait for subsequent new time info
  console->printf("GPS final: %d:%02d:%02d.0\n\r", dt_hours, dt_mins, dt_secs);

  daytime_init_report();
}

//
// daytime_init_from_ntp_finalise_stage3() updates time variables with current time from an NTP server
//
//                         . this is not a directly called user function
//
//                         . requires that daytime_init_from_ntp_trigger_stage1() be called to
//                           request an NTP update using a state machine to organise any retries
//                           until retrieve_UTC_using_NTP_stage2() receives a valid response, so
//                           that this function loads the new time data
//
//                         . the effectiveness of tenth_secs is questionable as one really needs
//                           to interact with an NTP server to refine the time
//
void daytime_init_from_ntp_finalise_stage3( unsigned long epoch, unsigned int tenth_secs ) {
  Serial.println("Time Sync: NTP");
  system_save_last_time_sync_as_NTP();

  it_hours = (epoch % 86400L) / 3600; // convert epoch to hr,min,secs and adjust for time zone
  dt_mins  = (epoch % 3600) / 60;
  dt_secs  = epoch % 60;
  clock_tick_set_fractional_seconds( tenth_secs );
  update_summer_time(beloud);
  it_hours = it_hours + the_time_zone;
  if (it_hours >= 24)
    it_hours = it_hours - 24;
  dt_hours = it_hours + dt_summer_offset;
  if (dt_hours >= 24) dt_hours = 0;
  console->printf("NTP final: %d:%02d:%02d.%1d\n\r", dt_hours, dt_mins, dt_secs, tenth_secs);
}
void dt_clear() {
  daytime_init_from_ntp_finalise_stage3(0,0);
}

//
// daytime_init_report() updates various clock displays with details of the time sync
//                       and finalises the day time resynchronisation
//
int time_since_last_update = 0;
void daytime_init_report() {
  lcd_clear_row(3);	// erase resync announcement
  lcd_clear_row(1);
  allPrint("Sync@");
  update_time_strings( 1, dt_hours,  dt_mins,  dt_secs);
  Time_of_last_sync = time_str;

  allPrintln(time_str);
  delay(1);				// system call to support its real-time activities

#if defined( support_7SEGMENT_DISPLAY )
  display_7seg_4digits(time_s12,clock_colon,1);
  flag_NTP_sync_done();
#endif
#if defined( support_DotMatrix_DISPLAY )
  mx.clear();
  if ( was_system_last_time_sync_NTP() )
    mdstr( (char*) "Sync ok" );
  else
    mdstr( (char*) "GPS  ok" );
#endif
  delay(1);				// system call to support its real-time activities

  if (time_since_last_update > 0) {
    lcd.print(" +");
    lcd.print(time_since_last_update);
  }

  lcd_setCursorSyncFlag;      // mark sync as complete
  lcd.print(' ');

  show_netstat( 2,-1 );       // clear extra LCD rows and fill with IP data
  lcd_clear_row(3); allPrintln( String(" ") + ReleaseMsg );

  // clear pending time display updates (irrelevant as new time synchronised)
  clear_timekeeping_visible_vars();
}

// daytime_init_from_ntp_trigger_stage1() activates an NTP based day time resynchronisation
//
//                                        . supports daytime_init_from_ntp_finalise_stage3()
//                                          and leads on to retrieve_UTC_using_NTP_stage2()
//
void daytime_init_from_ntp_trigger_stage1() {
#if defined( support_7SEGMENT_DISPLAY )
  flag_NTP_sync_active();	// (macro to set an indicator LED)
#endif

  fsm_ntp_enable();
				// set tmp_ntp_srv to either a specified host or pick from the ntp pool
  char* tmp_ntp_srv;

  if (optional_time_server!=0) {
    tmp_ntp_srv = optional_time_server;
  } else {
    *ntpServerName='0'+(0x3 & timeSyncCounts);
    tmp_ntp_srv = ntpServerName;
  }

  console->println(msg_marker);
  console->print("Using NTP server: ");
  console->println( tmp_ntp_srv );

  WiFi.hostByName(tmp_ntp_srv, timeServerIP);
  console->print("Received NTP server IP=");
  console->print(timeServerIP.toString());
  console->println();

  delay(1);                     // system call to support its real-time activities
  sendNTPpacket(timeServerIP);  // send an NTP packet to a time server
  delay(1);                     // system call to support its real-time activities

  lcd_setCursorSyncFlag;        // mark sync as in process
  timeSyncCounts++;
  allPrint("*");
}

// retrieve_UTC_using_NTP_stage2() is a support function for daytime_init_from_ntp_finalise_stage3()
//
void retrieve_UTC_using_NTP_stage2() {
  int cb = udp.parsePacket();
  if (!cb) {				// No packet...
    lcd_clear_row(0);
    if (fsm_ntp_get_count() > ntp_TIMEOUT) {
      allPrintln("NTP timeout, retry");
      delay_with_time_keeping(2000);
      daytime_init_from_ntp_trigger_stage1();
    } else {
      allPrint(" No packet yet");
      Serial.printf(" (try %d of %d)\n\r", fsm_ntp_get_count(), ntp_TIMEOUT);
      delay_with_time_keeping(300);	// (was 250)
    }
  } else {				// Got packet so decode
    fsm_ntp_disable();
      
    lcd_clear_row(0);
    allPrint("Rx pkt len=");
    allPrintN(cb);
    console->println();
      
    // received a packet so decode
    udp.read(packetBuffer, NTP_PACKET_SIZE);
    delay(1);				// system call to support its real-time activities

    // Note: the timestamp, as seconds since Jan 1 1900, starts at byte 40 of
    //       the received packet and is encoded as a big-endian 32b word
    //
    unsigned long secsSince1900 = 0;
    for (int i = 0; i < 4; i++)	// merge in 4 bytes with most significant first (big endian)
      secsSince1900 =
	(secsSince1900 << 8) | (unsigned long)packetBuffer[40 + i];
    //
    // Note: a 32bit/4byte count of resolution 232 pico-seconds is provided at offset 44
    //       (big endian) and is used to calculate an approx. 10th of second count
    unsigned int picosecs = 0;
    for (int i = 0; i < 4; i++) picosecs = (picosecs << 8) | (unsigned int)packetBuffer[44 + i];
    unsigned int tenth_secs = (int) ( 10.0 * 232.0 * (float) picosecs * 1E-12 );

                                        // process time data in Unix form (seconds since
                                        // 1/1/1970) therefore subtract 70 year offset
    const unsigned long seventyYears = 2208988800UL;

                                        // epoch in Unix form of UTC aka GMT
    unsigned long epoch = secsSince1900 - seventyYears;

    int time_of_this_update = dt_mins * 60 + dt_secs;				// save mins/secs as ms in order to report update size
    daytime_init_from_ntp_finalise_stage3( epoch, tenth_secs );
    time_since_last_update =  (dt_mins * 60 + dt_secs) - time_of_this_update;	// estimate (roughly) the correction
    time_since_last_update = time_since_last_update % 3600;			// (assume < an hour)
  }

  daytime_init_report();
}

///////////////////////////////////////////////////////////////////////////

// send an NTP request to the time server at the given address
void sendNTPpacket( IPAddress &anAddress ) {
  lcd_clear_row(0);
  allPrintln("Sending NTP packet..");
                                // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
                                // Initialize values needed to form NTP request
                                // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011; // LI, Version, Mode
  packetBuffer[1] = 0;          // Stratum, or type of clock
  packetBuffer[2] = 6;          // Polling Interval
  packetBuffer[3] = 0xEC;       // Peer Clock Precision
                                // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
                                // all NTP fields initialised, send
  udp.beginPacket(anAddress, 123);// NTP request on port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

///////////////////////////////////////////////////////////////////////////
