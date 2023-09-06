/* -*- mode: c++; -*- */

//
// The clock configuration is maintained in EEPROM and consists of an integrity
// check region followed by 3 C-style strings representing:
//
// . wifi ssid
// . wifi password
// . timezone offset char and then optional text tokens separated by a ' ' char
//
// Note: . Some recent clocks have been produced with different display modules
//       so there is now a need to retain details of what is the main display
//       to drive. As loss of this data could leave a clock without an active
//       display, the config read function retrieve_clk_config() makes a note if
//       these details have ever been saved and the config write function
//       save_clk_config() will ensure that current display details are retained
//       if the user does not specify display settings. The relevant variable
//       is eeprom_config_selected_LED, and it is updated whenever config info
//       is read.
//       . Some even more recent clocks have a GPS time source option. The web
//       config has to explicitly turn this option off or on when configuring
//       (if that is what the user wants), otherwise the previous settings are
//       retained.
//       . The GPS and LED choices aim to be persistent but respect user choice.

void mx_configure( int, int );	// eeprom info allows display driver choices

const int len_parm_string=31;
char tmp_hs[1+len_parm_string];	// (static storage for working copy of server name)

// Keep track of whether the system configuration is selecting a specific LED display
// (if a choice is read from eeprom config, we will ensure a choice is saved later)
bool eeprom_config_selected_LED = false;
int  eeprom_config_selected_LED_arg = -1;

//
// Simple config bytes dump for humans to read
//
void config_misc_bytes_dump() {
  for (int i = 0; i < MISC_parms_size-1; ++i) {
    console->printf( "%4d", (int) misc_bytes[i] );	// 4 chars/byte value
    if ((0==misc_bytes[i]) && (0==misc_bytes[i+1])) break;
  }
  if (misc_bytes[0]>=0)
    console->printf( "\n\rTime zone = UTC/GMT + %d\n\r    ", (int) misc_bytes[0] );
  else
    console->printf( "\n\rTime zone = UTC/GMT - %d\n\r    ", (int) (-misc_bytes[0]) );
  for (int i = 1; i < MISC_parms_size-1; ++i) {
    signed char x = misc_bytes[i];
    if (x<=0) x='|';
    console->printf( "  %c ", x );
    if ((0==misc_bytes[i]) && (0==misc_bytes[i+1])) break;
  }
}

//
// retrieve_clk_config() reads and decodes the configuration, with optional report
// . return integrity check
//
bool retrieve_clk_config( unsigned char rc_verbose ) {
  configuration_is_OK = false;

  EEPROM.begin(EEPROM_nBytesUsed);
  delay(10);

  // check integrity (each pair of bytes should sum to 0xff)
  int ptr = 0;
  for (int x = 0; x < n_ConfigPattern; x = x + 2) {
    int tmp = 0;
    tmp += EEPROM.read(ptr++);
    tmp += EEPROM.read(ptr++);
    if ( 0xff != tmp ) {
      console->print( "\n\rConfig dump (integrity test FAILED)\n\r" );
      return false;             // 2 of n_ConfigPattern bytes
    }
  }

  if (rc_verbose!=0) console->print( "\n\rConfig dump (integrity test is OK)\n\r" );

  // retrieve strings
  for (int i = 0; i < WIFI_parms_size; ++i)
    ssid_Cstr[i] = char(EEPROM.read(ptr++));
  if (rc_verbose!=0) console->print( String("  ssid_Cstr = ") + (char*) ssid_Cstr + "\n\r" );

  for (int i = 0; i < WIFI_parms_size; ++i)
    pass_Cstr[i] = char(EEPROM.read(ptr++));
  if (rc_verbose!=0) console->print( String("  pass_Cstr = ") + (char*) pass_Cstr + "\n\r" );

  // retrieve bytes
  for (int i = 0; i < MISC_parms_size; ++i)
    misc_bytes[i] = char(EEPROM.read(ptr++));

  if (rc_verbose!=0) {
    console->print("  misc_bytes = ");
    config_misc_bytes_dump();
    console->println();
  }

  // decode byte0=utc offset i.e. time zone
  // - every system must have this parameter so it is hardwired as the first byte
  int option_index = 0;
  the_time_zone = misc_bytes[option_index++];
  if (rc_verbose!=0)
    console->println( String( "    index=0, record type: TZ\n\r    > Time Zone UTC offset = ")+String( (int) the_time_zone ) );

  //
  // - decode other options of the form of id byte followed by
  //   either a value byte or null terminated string
  while ((option_index>0) && (option_index<MISC_parms_size)) {
    char * tmp;
    delay(1);					// system call to support its real-time activities
    if ((rc_verbose!=0)&&(misc_bytes[option_index]>=' '))
      console->printf( "    index=%d, record type: %c\n\r", (int) option_index, (char) misc_bytes[option_index] );

    switch (misc_bytes[option_index++]) {	// id byte

    case id_TimeServer:
      optional_time_server = (char*) &misc_bytes[option_index];	// ptr to static array with null termination
      option_index += 1+strlen(optional_time_server);		// option_index skips string and null termination
      if (rc_verbose!=0)
        console->printf( "    > Time server = %s\n\r", optional_time_server );
      break;

    case id_pb_switch_gpio:
      tmp = (char*) &misc_bytes[option_index];			// ptr to static array with null termination
      option_index += 1+strlen(tmp);				// option_index skips string and null termination
      __set_pb_gpio_id( atoi( tmp ) );
      if (rc_verbose!=0)
        console->printf( "    > GPIO line for pb switch = %d\n\r", (int) get_pb_gpio_id() );
      break;

    case id_HomeServer:
      tmp = (char*) &misc_bytes[option_index];			// ptr to static array with null termination
      option_index += 1+strlen(tmp);				// option_index skips string and null termination
      if (strlen(tmp)>0) {
	strncpy((char*) tmp_hs, tmp, len_parm_string);
	tmp = (char*) tmp_hs;			// change to using the separate storage
	OTA_server_name = tmp;			// (have hostname)
	tmp = index( tmp, ':' );
	if ( tmp != NULL ) {			// (have a port number)
	  *tmp=0;				// (null terminal hostname char*)
	  int pn = atoi( tmp+1 );
	  if ( pn>0 ) OTA_port_number = pn;	// (extract port number)
	}
      if (rc_verbose!=0)
  	console->printf( "    > Lake Placid is @ %s:%d%s\n\r", OTA_server_name, OTA_port_number, OTA_file_name );
      }
      //      return configuration_is_OK = true;
      break;

    case id_DimSchedule:
      tmp = (char*) &misc_bytes[option_index];			// ptr to static array with null termination
      option_index += 1+strlen(tmp);				// option_index skips string and null termination
      if (strlen(tmp)>0) {			// decode brightness:start hour:stop hour
	LEDctl.brightness = (unsigned char) strtol(tmp, &tmp, 10); tmp++;
	LEDctl.start_hour = (unsigned char) strtol(tmp, &tmp, 10); tmp++;
	LEDctl.stop_hour  = (unsigned char) strtol(tmp, &tmp, 10);
      }
      if (rc_verbose!=0)
  	console->printf( "    > Brightness schedule %d from %d hours to %d hours\n\r", LEDctl.brightness, 100*LEDctl.start_hour, 100*LEDctl.stop_hour );
      break;

    case id_LED_select:
      eeprom_config_selected_LED = true;				// note that this system has managed LED choice
      system_switch_to_selected_led( misc_bytes[option_index++] );
      if ( is_system_using_dots_led() ) {
	if ( (misc_bytes[option_index]>='0') && (misc_bytes[option_index]<='9') )
	  eeprom_config_selected_LED_arg=misc_bytes[option_index++]-'0';// instantiate the chosen array driver setup
	else
	  eeprom_config_selected_LED_arg=-1;				// or instantiate the default
	mx_configure( eeprom_config_selected_LED_arg, rc_verbose );
      }
							// (leave option_index pointing to next id)
      if (rc_verbose!=0)
        console->printf( "    > LED display is using %s\n\r", (is_system_using_7seg_led())?"7-segment LED":"dot matrix LED array" );
      option_index++;
      break;

    case id_HOURS_mode:
      system_switch_to_selected_HOURS_mode( misc_bytes[option_index++] );
							// (leave option_index pointing to next id)
      if (rc_verbose!=0)
        console->printf( "    > Hours display mode is %s hour mode\n\r", ( is_system_using_24HOUR_display() )?"24":"12" );
      option_index++;
      break;

    case id_GPS_time:
      system_switch_to_selected_time_source( misc_bytes[option_index++] );
							// (leave option_index pointing to next id)
      update_internet_access_needs();			// note if this system does NOT need internet access
      if (rc_verbose!=0)
        console->printf( "    > Master time sync source is %s\n\r", ( is_system_using_NTP_source() )?"NTP":"GPS" );
      option_index++;
      break;

    default:
      option_index = -999;
      break;
    }
  }

  EEPROM.end();
  configuration_is_OK = true;
  console->flush();

  if (rc_verbose==0) return true;

  lcd_clear_row(0);
  allPrintln("Decoding config");

  DelSec( 1 );
  String x = String(" (") + ssid_Cstr + ')';
  lcd_clear_row(0); allPrintln((char*) x.c_str());

  x = String(" (") + pass_Cstr + ')';
  lcd_clear_row(1); allPrintln((char*) x.c_str());

  lcd_clear_row(2); allPrint(" (UTC/GMT ");
  if (the_time_zone > 0)  allPrint("+");
  if (the_time_zone != 0) allPrintN(the_time_zone);
  if (optional_time_server!=0) {
    allPrint(", Tsrv=");
    allPrint(optional_time_server);
  }
  if (get_pb_gpio_id() != defaultButton) {
    allPrint(", gpio-pb=");
    allPrint(String(get_pb_gpio_id()));
  } else {
    allPrint(" [default gpio-pb=");
    allPrint(String(get_pb_gpio_id())+"]");
  }

  lcd_clear_row(3);
  allPrint(" [OTA=");
  allPrint(OTA_server_name);
  allPrint(":");
  allPrint(String(OTA_port_number).c_str());

  allPrint("])");

  DelSec( 1 );
  return true;
}

//
// save_clk_config() sabe the clock config to non-volatile EEPROM
// . the display options are allowed to change but once a clock has them specified, they can
//   never be omitted
//
void save_clk_config(char *s, char *p, signed char ttz, char *multiple_option_tokens, int verbose) {
  String tmp_options;

  EEPROM.begin(EEPROM_nBytesUsed);
  delay(10);

  console->println();
  console->println("Saving Configuration with args:");
  console->println(s);
  console->println(p);
  console->println( (char*) (String( (int) ttz )).c_str() );
  console->println(multiple_option_tokens);
  console->println();

  // If the LED display choice is managed, ensure that the new multiple_option_tokens also sets a value
  if ( eeprom_config_selected_LED &&					// is managed by eeprom config AND
       (NULL == index( multiple_option_tokens, id_LED_select)) ) {	// no LED config info included so
									// look up original details and append
    // <space char><id_LED_select char><LED_select char><optional orientation char used by some dot matrix><space char>
    tmp_options = String(multiple_option_tokens) + " " + char( id_LED_select ) + char( get_selected_led_as_ASCII() );
    if (eeprom_config_selected_LED_arg >= 0) tmp_options += char( eeprom_config_selected_LED_arg + '0' );
    console->println("Updating options: include display info");
    multiple_option_tokens = (char*) tmp_options.c_str();
    console->println(multiple_option_tokens);
    console->println();
  }

  //
  // Clear EEPROM region
  //
  lcd_clear_row(0);
  console->println();
  console->println();
  allPrintln("Clearing memory...");
  for (int i = 0; i < EEPROM_nBytesUsed; ++i) {
    EEPROM.write(i, 0);
    delay(1);
  }

  //
  // Write "magic" patterns to indicate valid data follows
  //
  lcd_clear_row(1);
  allPrintln("Writing header...");
  int ptr = 0;
  for (int x=0; x < n_ConfigPattern; x++) {
    EEPROM.write(ptr++, header[x]);
    delay(1);
  }

  //
  // Clear and load buffers for string and int data to write
  //
  for (int i = 0; i < WIFI_parms_size; ++i)
    ssid_Cstr[i] = pass_Cstr[i] = 0;		// (was setting default to ' ')
  //

  // copy network info to fixed length buffers
  strncpy(ssid_Cstr, s, WIFI_parms_size);	// (padding with `\0' characters)
  strncpy(pass_Cstr, p, WIFI_parms_size);	// (padding with `\0' characters)
  //
  for (int i = 0; i < MISC_parms_size; ++i)
    misc_bytes[i] = 0;

  // copy options to fixed length misc_bytes buffer
  int cnt=0;
  misc_bytes[cnt++] = ttz;			// Copy time zone ttz etc. to fixed length buffer

  						// Copy any additional options provided in multiple_option_tokens
						// to misc_bytes with option separator ' ' mapped to a 0 byte
  char *cptr = multiple_option_tokens;
  while (*cptr != 0) {
    while ((*cptr != 0)&&(*cptr == ' '))	// SKIP any repeated ' ' separators
      cptr++;
    if ( ! is_valid_misc_option_id(*cptr) )	// REQUIRE leading char of misc option to be valid
      break;					// FINISH option processing on invalid option type else
    while ((*cptr != 0)&&(*cptr != ' '))	// COPY token i.e. leading char and optional parameters
      misc_bytes[cnt++] = *cptr++;
    misc_bytes[cnt++] = 0;			// DELIMIT by writing 0 after each token
  }
  misc_bytes[cnt++] = 0;			// COMPLETE with final write 0 i.e. two consecutive 0 bytes uniquely mark end

  console->print("\n\rOption char*:    ");	// echo original options string (skip TZ byte)
  cptr = multiple_option_tokens;
  while (*cptr != 0) {
    char x = *cptr++;
    if ((x<' ')||(x>'~')) x=' ';
    console->printf("  %c",x);
  }
  console->print("\n\rOption bytes: ");		// echo misc_bytes buffer to user as HEX
  for (int i = 0; i<cnt; i++) {
    console->printf(" %02x", misc_bytes[i]);
  }
  console->println();

  //
  // Write...
  //
  lcd_clear_row(0);
  allPrint("Writing to eeprom:");
  lcd_clear_row(1);
  for (int i = 0; i < WIFI_parms_size; ++i) {
    EEPROM.write(ptr++, ssid_Cstr[i]);
    delay(1);
  }
  allPrint("ssid");
  for (int i = 0; i < WIFI_parms_size; ++i) {
    EEPROM.write(ptr++, pass_Cstr[i]);
    delay(1);
  }
  allPrint(" password");
  for (int i = 0; i < MISC_parms_size; ++i) {
    EEPROM.write(ptr++, misc_bytes[i]);
    delay(1);
  }
  allPrint(" tz");
  if (misc_bytes[1]!=0) allPrint("+opts");

  //
  // Commit changes
  //
  EEPROM.end();
  allPrintln(" :-)");

  DelSec( 1 );
  console->println();
  if (verbose == bequiet) return;

#if defined( __extra_bytes_displayed__ )

  lcd_clear_row(0); allPrintln("Summary");       lcd_clear_row(1);

  DelSec( 1 );
  console->println();
  lcd_clear_row(0); allPrintln("WiFi SSID");     lcd_clear_row(1); allPrint(" "); allPrintln(ssid_Cstr);

  DelSec( 1 );
  console->println();
  lcd_clear_row(0); allPrintln("WiFi Password"); lcd_clear_row(1); allPrint(" "); allPrintln(pass_Cstr);
                                                 lcd_clear_row(2); allPrint(" "); allPrintln(ssid_Cstr);
  DelSec( 1 );
  console->println();
  lcd_clear_row(0); allPrintln("Time Zone");     lcd_clear_row(1); allPrint(" "); allPrintlnN(int(ttz));
                                                 lcd_clear_row(2); allPrint(" "); allPrintln(pass_Cstr);
                                                 lcd_clear_row(3); allPrint(" "); allPrintln(ssid_Cstr);
  DelSec( 1 );
  console->println();
  lcd_clear_row(0); allPrintln("Options");       lcd_clear_row(3);
                                                 lcd_clear_row(1); allPrint(" "); allPrintln(multiple_option_tokens);
                                                 lcd_clear_row(2); allPrint(" ");
  DelSec( 1 );
  console->println();
#endif
}

///////////////////////////////////////////////////////////////////////////
