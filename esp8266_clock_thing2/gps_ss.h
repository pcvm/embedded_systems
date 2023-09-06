//
// Support for GPS access via espSoftwareSerial
//

#include <SoftwareSerial.h>

// Refs:
// . https://randomnerdtutorials.com/guide-to-neo-6m-gps-module-with-arduino/ (includes "NMEA Sentences")
// . https://github.com/plerup/espsoftwareserial/issues/160
// . https://forum.arduino.cc/t/problems-with-communication-between-wemos-d1-mini-and-arduino-mega/1038133/9
//
//  If a time limited read period is needed:
//    #define MAX_MILLIS_TO_WAIT 8000  // wait up to 8 seconds
//    unsigned long starttime = millis();
//    while (ss_can_read() && ((millis() - starttime) < MAX_MILLIS_TO_WAIT)) {
//      /* do stuff */
//    }
//
// Note: define receiver pin as SS_RX before including this file. No TX facility is enabled.

SoftwareSerial ssport1(SS_RX, 0);	// RX port (SS_RX), TX port (none)
unsigned char  ssport1_is_enabled = 0;	// Flag to manage SS port initialisation to be "run once"
//
// Initialise software serial port (do not prevent re-initialisation)
//
unsigned char is_ssport1_enabled() {
  return ssport1_is_enabled;
}
void initialise_ssport1() {
  ssport1.begin(9600, SWSERIAL_8N1);	// NEO-6M module default UART configuration
  ssport1_is_enabled = 1;
}

///////////////////////////////////////////////////////////////////////////

//
// Polled access to GPS
// .  enable_gps_updates()     - enables the real-time bit handling for SS port1 which then
//                               allows GPS data to be received and processed
// .  background_gps_updates() - main background polling function to update time
// .  int UTC_from_GPS_is_ready_to_retrieve()    - true when unread time data available
// .  int get_gps_time(index)  - access 0,1,2 bytes
//

#define t_BSIZE 5
unsigned char t_buffer[t_BSIZE] = {0};	// most recently received time of day, with byte t_STATUS==1 if unread
#define t_STATUS t_buffer[t_BSIZE-1]	// (4 bytes for hr,min,sec,hundredths; last byte cleared when read)

//
// background_gps_updates() handles all background software serial and GPS data retrieval
//
// Method: store strings of the form "$GPGGA,110617.00,..." in wrap-around buffer ssport1_wabuffer
//         with the storage index reset to 0 whenever a gPREFIX '$' char is received. This allows
//         for fairly easy string inspection starting at offset 0 to detect a "$GPGGA" sentence
//         of data to decode.

// Example time:        "$GPGGA,110617.12, ...other stuff"
//      Offsets:         0123456789abcdefg where a=10,b=11,c=12,d=13,e=14,f=15,g=16
const char gPREFIX = '$';
#define GPS_TS_HEADER   "$GPGGA,"
#define	GPS_TS_LEN              7		// header length to string match
#define GPS_TS_DOT                   13		// header offset to sep char between secs and 100ths of secs
#define GPS_TS_COMMA                    16	// header offset to last char that must be a '.'
//
const int ssport1_buff_len = 128;		// 2^7 (power of 2 simplifies wrap-around, n*words supports memset)
const int ssport1_wabuffer_mask = ssport1_buff_len-1;		// mask
char ssport1_wabuffer[sizeof(int) + ssport1_buff_len] = {0};	// string eos pad safety, add word supports memset
						// By restricting all writes to offset 0..ssport1_buff_len-1
int ssport1_wabuffer_index = 0;			// this buffer will contain chars and always '\0' terminated

// 
void enable_gps_updates() {
  ssport1_wabuffer_index = 0;	// clear byte store index
  if ( ! is_ssport1_enabled() )
    initialise_ssport1();	// ensure software serial port active
}

//
void background_gps_updates() {
  if ( ! is_ssport1_enabled() ) return;		// no polling (and no update) to do if serial port is not enabled

  while ( ssport1.available() > 0 ) {		// while data can be read...
    delay(1);					// system call to support its real-time activities
    unsigned char x = ssport1.read();
    if ( x == gPREFIX ) {
      ssport1_wabuffer_index = 0;
      memset((void*) ssport1_wabuffer, 0, ssport1_buff_len);	// new packet so initialise to all '\0' chars
    }
    ssport1_wabuffer[ssport1_wabuffer_index++] = x;
    ssport1_wabuffer_index = ssport1_wabuffer_index & ssport1_wabuffer_mask;

    // Inspect chars for a minimum length to hold a pkt of time data
    if ( ssport1_wabuffer[       0] != gPREFIX ) continue;		// 1st char mismatch so no valid packet
    if ( ssport1_wabuffer[GPS_TS_DOT]   != '.' ) continue;		// sep char mismatch so no valid packet
    if ( ssport1_wabuffer[GPS_TS_COMMA] != ',' ) continue;		// last char mismatch so no valid packet
    if ( 0 != strncmp(ssport1_wabuffer, GPS_TS_HEADER, GPS_TS_LEN) )
      continue;								// no GPS_TS_HEADER match so no valid packet
    // (currently not bothering to confirm that time digits are in range '0'..'9')

    // fall through means valid time data can be extracted as ASCII digits starting at offset GPS_TS_LEN
    x = GPS_TS_LEN;
    t_buffer[0] = (unsigned char) (10 * (ssport1_wabuffer[x+0] - '0') + (ssport1_wabuffer[x+1] - '0'));
    t_buffer[1] = (unsigned char) (10 * (ssport1_wabuffer[x+2] - '0') + (ssport1_wabuffer[x+3] - '0'));
    t_buffer[2] = (unsigned char) (10 * (ssport1_wabuffer[x+4] - '0') + (ssport1_wabuffer[x+5] - '0'));
    t_buffer[3] = (unsigned char) (10 * (ssport1_wabuffer[x+7] - '0') + (ssport1_wabuffer[x+8] - '0'));
    t_STATUS = 1;				// Flag new or refreshed data
  }
}

//
// get_gps_time() reads back the gps time data bytes for index=0,1,2 and marks t_STATUS as read
//                (returns 0 if index outside valid time data range)
//
int get_gps_time(int index) {
  if ( (index<0) || (index>=(t_BSIZE-1)) ) return 0;
  t_STATUS = 0;				// read of any data byte clears the "new" flag
  return (int) t_buffer[index];
}

//
// UTC_from_GPS_is_ready_to_retrieve() is true/non-zero when unread data is in t_buffer
//
int UTC_from_GPS_is_ready_to_retrieve() {
  return t_STATUS;
}
