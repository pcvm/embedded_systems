/* -*- mode: c++; -*- */

// Basic reconfiguration via serial comms
// (user is running an app like kermit/putty/cu etc.)

// Basic string input/output
//
void user_input_flush() {
  while (Serial.available() > 0) Serial.read();
  ptr_getB_out = ptr_getB_in;
  delay(0);
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
String new_config_item( char* a_name, char* a_value ) {
  user_input_flush();				// flush
  console->print( String( a_name ) + "\n\rCurrent value: " + a_value +
	      "\n\rEnter a new value then ENTER, or simply ENTER for no change: " );

  String x=my_read_string();
  if (x.length()==0) return a_value;	// no change

  changed_config = true;
  return x;
}

// Main update function
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

  console->print("\n\n\rYou are about to view and be able to change each item until all are OK.\n");

  changed_config = false;
  while (true) {
    ssid = new_config_item( (char*) "\n\n\rUpdate SSID (wifi network name)", (char*) ssid.c_str());
    //    Serial.println(("Received ")+ssid);
    pwd  = new_config_item( (char*) "\n\n\rUpdate WiFi password",            (char*) pwd.c_str());
    //    Serial.println(String("Received ")+pwd);
    utco = new_config_item( (char*) "\n\n\rUpdate time zone offset (0 for London, 1 for Berlin, 10 for Brisbane, etc.)", (char*) utco.c_str());
    //    Serial.println(String("Received ")+utco);

    console->print( "\n\n\rThe next item is usually not needed, unless you wish to enable a special option e.g. a specific time server.\n\r");
    {
      String m=String("Special options comprise an id letter and then some extra characters for a value.\n\rCurrent id letters are: ") +
	(char) id_TimeServer     + "=time server IP, " +
	(char) id_pb_switch_gpio + "=gpio pb number, " +
	(char) id_HomeServer     + "=alternative_home_OTA_server_name:port";

      m += "\n\rUse a '|' or ' ' character to separate options, and enter one '|' to force an empty special option.\n\r\
\n\rExamples:\
\n\r|                         no options (and erases any previous options)\
\n\rt192.168.1.10             choose a specific time server (usually a home computer time serving to all of your clocks)\
\n\rt192.168.1.10|hgiotto:80  debug option1: using a local time server and specify a local OTA server (for Phillip's tests)\
\n\rg10                       debug option2: specify alternative wiring for the push button switch    (work-around pin failure)\
\n\rd0,22,6        for most people who just want 8 hours of low level light between 10pm and 6am, and the default time server\
\n\rd0,22,6|L1     as above but now enable dot matrix display instead of 7segment\
\n\rd0,22,6|L1|H1  as above and using 12 hour mode\
\n\rd0,22,6|L1|H2  as above and using 24 hour mode\
\n\n\r\
\n\rSome special single letter preset options and combinations for convenience\
\n\rD              dimmed LED 10pm-6am\
\n\rE              dimmed LED 10pm-6am, DOT MATRIX\
\n\rF              dimmed LED 10pm-6am, DOT MATRIX in 12 hour mode\
\n\rP              dimmed LED 10pm-6am, time server = 192.168.1.10\
\n\rQ              dimmed LED 10pm-6am, time server = 192.168.1.10, DOT MATRIX\
\n\n\r";

      sys_options = new_config_item( (char*) m.c_str(), (char*) sys_options.c_str());
      if ( sys_options == String('D') ) {		// silent remapping
	sys_options = String("d0,22,6");
      }
      if ( sys_options == String('E') ) {
	sys_options = String("d0,22,6|L1");
      }
      if ( sys_options == String('F') ) {
	sys_options = String("d0,22,6|L1|H1");
      }
      if ( sys_options == String('P') ) {
	sys_options = String("t192.168.1.10|d0,22,6");
      }
      if ( sys_options == String('Q') ) {
	sys_options = String("t192.168.1.10|d0,22,6|L1");
      }
      console->println(String("\n\rReceived [")+sys_options+"]");
    }
    if ((sys_options == String('|')) ||
	(sys_options == String(' '))) sys_options = "";
    for (int cnt=0; cnt < (int)sys_options.length(); cnt++)	// map '|' to ' '
      if (sys_options[cnt]=='|')
	sys_options[cnt]=' ';
    if (sys_options[sys_options.length()-1] != ' ')	// ensure a ' ' padding is at end to assist decoding by save_config()
      sys_options += ' ';

    user_input_flush();					// flush input buffer then display current settings
    console->print( String("\n\rCurrent choices:\n\r") +
		    "  SSID (wifi network name) = " + ssid + "\n\r" +
		    "  WiFi password            = " + pwd  + "\n\r" +
		    "  Time zone offset         = " + utco + "\n\r" );
    if (sys_options.length()==0)
      console->print( "  (no optional settings)" );
    else
      console->print( String("  Optional settings = ") + sys_options );

    console->print( "\n\rType ENTER if all are OK, or any other key to review/edit again " );
    while ( ! user_input_isAvailable() ) delay(100);
    if ( user_input_getchr() < ' ') break;		// end of setup checks
  }

  if ( ! changed_config ) {
    console->print("\n\rNo changes so nothing to save\n\n\r");
    set_clock_part1();
    return;
  }

  char *ssid_ptr = (char *)ssid.c_str();
  char *pwd_ptr = (char *)pwd.c_str();
  signed char byte__utc_offset = atoi(utco.c_str());	// convert string back to a number

							// save config (sys_options checked in save_clk_config())
  save_clk_config(ssid_ptr, pwd_ptr, byte__utc_offset, (char*) sys_options.c_str(), beloud /* bequiet */);

  clear_lcd_row(0);
  console->print( "\n\rRestarting esp8266...\n\rNote: if normal time keeping does not start within 30seconds, please power cycle the clock.\n\n\r" );

  do_full_system_restart((char*) "    conF OK  ");	// never return
}

///////////////////////////////////////////////////////////////////////////
