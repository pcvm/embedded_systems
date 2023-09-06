/* -*- mode: c++; -*- */

// Basic reconfiguration via serial comms
// (user is running an app like kermit/putty/cu etc.)

// Basic string input/output
//
void user_input_flush() {
  while (Serial.available() > 0) Serial.read();
  ptr_getB_out = ptr_getB_in;
  //  delay(1);
}
unsigned char user_input_isAvailable() {
  if (io_is_serial())
    return Serial.available();
  else {
    process_network_io();
    return (ptr_getB_out != ptr_getB_in);
  }
}
short int user_input_getchr() {
  if (io_is_serial())
    return Serial.read();	
  else {
    process_network_io();
    return getchr_telnet();
  }
}

// Update a config item String
//
bool changed_config = false;
String new_config_item( String var_description, String current_value ) {
  user_input_flush();				// flush
  console->print( var_description + "\n\rCurrent value: " + current_value +
	      "\n\rEnter a new value then ENTER, or simply ENTER for no change: " );

  String x=my_read_string();
  if (x.length()==0) return current_value;	// no change

  changed_config = true;
  return x;
}

// Main update function reconfigure_system_with_user_io() allows the user to change all config options,
// and new_config_sys_options() handles the more complex sys_options separately.
//
String new_config_sys_options( String old_options ) {
  //
  // special_options_user_info is defined in the main file
  String new_options = new_config_item( special_options_user_info, old_options );
  console->println(String("\n\rReceived  [")+new_options+"]");

  // Note: this options processing handles its own special letters, and one const byte id_GPS_time defined in enum option_byte_sequences_id
  {						// process shortcuts of 1 or 2 chars in a simplistic way
    String shortcuts_extra = "";
						// special handling of 'G' to mean enable GPS time sync
    int tmpi = new_options.indexOf('G');
    if (tmpi>=0) {					// have found a 'G' char
      console->print( String( '[' ) + new_options + "] --> [" );
      shortcuts_extra = String("|") + char(id_GPS_time);// load shortcuts_extra with id_GPS_time char
      char tmparg = 0;
      if ( (int) new_options.length() > (1+tmpi) ) {	// inspect any next char and copy it to tmparg for use as a id_GPS_time arg if valid
	tmparg = new_options[1+tmpi];
	if ( (tmparg < '0') || (tmparg > '9') ) tmparg = 0;
      }
      if (tmparg == 0) {
	new_options.remove(tmpi, 1);
      } else {
	shortcuts_extra += tmparg;			// append any valid arg to shortcuts_extra
	new_options.remove(tmpi, 2);
      }
      console->println( new_options + "]" );
    }						// new_options is now 0 or 1 chars in length

						// remap various 1-char shortcuts to normal long form
    if ( new_options == String('D') )
      new_options = String(char(id_DimSchedule)) + "0,22,6";
    if ( new_options == String('E') )
      new_options = String(char(id_DimSchedule)) + "0,22,6|"       + char(id_LED_select)  + "1";
    if ( new_options == String('F') )
      new_options = String(char(id_DimSchedule)) + "0,22,6|"       + char(id_LED_select)  + "1|"      + char(id_HOURS_mode) + "1";
    if ( new_options == String('P') )
      new_options = String(char(id_TimeServer))  + "192.168.1.10|" + char(id_DimSchedule) + "0,22,6";
    if ( new_options == String('Q') )
      new_options = String(char(id_TimeServer))  + "192.168.1.10|" + char(id_DimSchedule) + "0,22,6|" + char(id_LED_select) + "1";

    new_options += shortcuts_extra;		// append extra options
  }

  if (new_options.length() == 0) new_options += ' ';
  for (int cnt=0; cnt<(int)new_options.length(); cnt++)	// map '|' to ' '
    if (new_options[cnt]=='|') new_options[cnt]=' ';
  if (new_options[new_options.length()-1] != ' ')	// ensure final char is ' ' (helps save_config())
    new_options += ' ';

  console->println(String("\rProcessed [")+new_options+"]");
  return new_options;
}
//
void reinitialise_daytime_from_source();
//
void reconfigure_system_with_user_io() {
  String ssid="";
  String pwd="";
  String utco="";
  String sys_options="";

  if (retrieve_clk_config(bequiet)) {
    ssid = String(ssid_Cstr);
    pwd  = String(pass_Cstr);
    utco = String((int) misc_bytes[0]);		// treat utco as a string, for now
						// treat opt_TimeServer as a string, for now
    if (misc_bytes[1] != 0) {
      int cnt=0;
      while (cnt<MISC_parms_size-1) {
	if ( (misc_bytes[1+cnt]==0) && (misc_bytes[2+cnt]==0) ) break;
	if (misc_bytes[1+cnt]<' ') sys_options+=' ';
	else                       sys_options+=char(misc_bytes[1+cnt]);
	cnt++;
      }
    }
  }

  console->print("\n\n\rYou are about to view and optionally change each item.\n\rOnce done, you have a final review of everything before deciding to accept changes (or start again).\n\r");

  changed_config = false;
  while (true) {
    ssid = new_config_item( String("\n\n\rUpdate SSID (wifi network name)"), ssid );
    pwd  = new_config_item( String("\n\n\rUpdate WiFi password"), pwd );
    utco = new_config_item( String("\n\n\rUpdate time zone offset (0 for London, 1 for Berlin, 10 for Brisbane, etc.)"), utco );

    console->print( "\n\n\rThe next item is usually not needed, unless you wish to enable a special option e.g. a specific time server.\n\r");
    sys_options = new_config_sys_options( sys_options );

    user_input_flush();					// flush input buffer then specifically check WiFi use when GPS timesync 
    char* ssid_extra_info = (char*) "";
    if ( sys_options.indexOf("G1") >= 0 ) {
      console->print( "\n\rUse of GPS timesync detected so WiFi is not essential.\n\rWould you like to disable WiFi (y or n)? " );
      String x=my_read_string();
      if ( x[0] == 'y' ) {
	ssid = wifiDisabled;
	pwd  = "<none>";
	ssid_extra_info = (char*) " (i.e. will not connect)";
	changed_config = true;
      }
    }

    user_input_flush();					// flush input buffer then display and confirm current settings
    console->print( String("\n\rReview the choices:\n\r\n\r") +
		    "  SSID (wifi network name) = " + ssid + ssid_extra_info + "\n\r" +
		    "  WiFi password            = " + pwd  + "\n\r" +
		    "  Time zone offset         = " + utco + "\n\r" );
    if (sys_options.length()==0)
      console->print( "  (no optional settings)" );
    else
      console->print( String("  Optional settings = ") + sys_options );

    console->print( "\n\r\n\rType ENTER if all are OK, or any other key to review/edit again " );
    while ( ! user_input_isAvailable() ) delay(100);
    if ( user_input_getchr() < ' ') break;		// end of setup checks
  }
  console->print("\n\r");

  if ( ! changed_config ) {
    console->print("\n\rNo changes so nothing to save\n\n\r");
    reinitialise_daytime_from_source();
    return;
  }

  char *ssid_ptr = (char *)ssid.c_str();
  char *pwd_ptr = (char *)pwd.c_str();
  signed char byte__utc_offset = atoi(utco.c_str());	// convert string back to a number

							// save config (sys_options checked in save_clk_config())
  save_clk_config(ssid_ptr, pwd_ptr, byte__utc_offset, (char*) sys_options.c_str(), beloud /* bequiet */);

  lcd_clear_row(0);
  console->print( "\n\rRestarting esp8266...\n\rNote: if normal time keeping does not start within 30seconds, please power cycle the clock.\n\n\r" );

  //  sys_options="";					// clear global vars but note system resets
  do_full_system_restart((char*) "    conF OK  ");	// never return
}

///////////////////////////////////////////////////////////////////////////
