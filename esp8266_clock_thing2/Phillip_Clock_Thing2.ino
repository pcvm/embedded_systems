/* -*- mode: c++; -*- */

/*
 * Phillip's Clock Thing is a world clock application built with off-the-shelf components.
 *										See -pm-
 *   p.musumeci@ieee.org Jan-2016/Jan-2017/June-2019/Feb-2020/Apr-2021
 *
 * Note/early 2023: calls to delay(0) were found to prevent timer call backs (i.e. internal esp8266
 *                  operation) so delay(1) is now used as a minimum delay to support esp8266 systems.
 */

#pragma GCC diagnostic ignored "-Wunused-parameter"

#define ThisRelease "2023.5d"

#include "notes_01_overview.h"	/* user notes */
#include "notes_10_developer.h"	/* circuit diagrams (ascii graphics) and software development notes */

/*
 *   Compile time options:
 *   . use_LED_DISPLAY                -> almost always enabled
 *   . haveGPS                        -> enabled when a NEO-6M GPS module is provided for time sync (uses a software serial UART
 *                                       on GPIO2 so this excludes options for temperature sensing, 1Hz output, and alt. PB in)
 *   . useDHTsensor, useDS1820sensor  -> (clashes with GPS due to GPIO2 use) the one-wire DS1820 is most often used
 *   . useAnalogueTickTock            -> (clashes with GPS due to GPIO2 use) experimental, was providing 1Hz external time sync.
 *
 *   Run time options:
 *     . with use_LED_DISPLAY enabled, the variable led_display_is is usually set to
 *       using_7seg_led but the alternative using_dots_led is chosen for LED dot matrix use
 *       and requires config option "L1". As it happens, these dot matrix boards from different
 *       manufacturers can (and do) orient the 8x8 LEDs in 4 ways so specific displays may need
 *       a config option of "L10", "L11", "L12", or "L13".
 *     . with haveGPS enabled, the daily time sync can change from an NTP source (via WiFi)
 *       to a GPS source (via the NEO-6M) i.e. the clock does not need an internet connection.
 *       This relies on an optional setting of "S2".
 *
 *   TCPIP ports:
 *     . web servers use port 80, telnet uses 23 (MAX_SRV_CLIENTS=1), ntp uses 2390.
 */

// If a special user message is needed, define DO_SPECIAL_WELCOME as required.
#define DO_SPECIAL_WELCOME
// If a special executable is needed to resurrect a broken system, perhaps by overwriting eeprom storage
// or performing extra diagnostics, define DO_SPECIAL_SETUP as appropriate.
#define DO_SPECIAL_SETUP

///////////////////////////////////////////////////////////////////////////

/*
 * Select compile time options
 */

#define use_LED_DISPLAY 1	// include code for various types of LED displays

#define haveGPS 1		// support time input via software serial UART RX attached to NEO-6M GPS module

#if ! defined( haveGPS )	// GPS sensor support has priority over single bit temperature/humidity sensors
//#define useDHTsensor  1	// . temperature/humidity via DHT11
#define useDS1820sensor 1	// . temperature via DS1820
#endif

#if defined( haveGPS )		// Adjust displayed startup name based on sensor choice
#define sensor_tag "g"
#elif defined( useDHTsensor )
#define sensor_tag "dht"
#define haveTempSensor		1
#elif defined( useDS1820sensor )
#define sensor_tag "ds"
#define haveTempSensor		1
#else
#define sensor_tag ""
#endif

//#define useAnalogueTickTock	1	// enable/disable analogue clock output pulse
// PIN ASSIGNMENT
#if defined( useAnalogueTickTock )
#define CLK_TICK_OUTPUT		10	// Note: GPIO10 is a fallback alternative PB if GPIO16 input damaged
#endif

#if defined( use_LED_DISPLAY )
#define support_7SEGMENT_DISPLAY  1		// an Adafruit 7-segment display attached via I2C
#define support_DotMatrix_DISPLAY 1		// an MD dot matrix attached via SPI
#endif

// System id and version message strings (cpp concatenation)
//
#define My_Version    ThisRelease sensor_tag	// use rD in place of vD due to 7-seg display
#define My_Hello  "    he\to   " My_Version "    ----"
const char* ReleaseMsg = "S/W: " My_Version ;
//
// . user help message string for options
char * special_options_user_info_cstr;

///////////////////////////////////////////////////////////////////////////

// define default time zone and daily synchronisation time
#define DefaultTimeZone (+10)
// Specify the daily time sync time = TimeSync_hours:TimeSync_mins:TimeSync_secs
// Choose 2:34am
#define TimeSync_hours (2)
#define TimeSync_mins  (3)
#define TimeSync_secs  (4)

///////////////////////////////////////////////////////////////////////////

// define IO setup
//

// PIN ASSIGNMENT - default TX/RX pins
#define uartTXpin  1
#define uartRXpin  3

// PIN ASSIGNMENT - user reset switch
// -- when using an esp8266 on the white daughterboard
#define defaultButton 16	// GPIO16 has a 4k7 pull down resistor and a
#define ButtonActive HIGH	// switch connecting it to 3.3V, press for HIGH
// -- progress flashing indicator
#define progressLED    2	// GPIO2 == D4 == LED

// PIN ASSIGNMENT - SPI
#define SPI_miso  12
#define SPI_mosi  13
#define SPI_sck   14    // shift clock
#define SPI_ss    15    // device select

// PIN ASSIGNMENT - I2C
#define I2C_scl    5
#define I2C_sda    4
// - a shared pin is used for temperature and humidity IO, or for GPS serial comms RX
#define DHTPIN     2    // DHT module uses bidirectional serial protocol.
#define DSPIN      2    // Dallas Semi 1-wire protocol handles temperature sensor.
                        // Alternative IO pins: GPIO0 and if SPI input can be
                        // disabled, GPIO12 might also be an option
#define SS_RX      2	// used for soft serial receiver for GPS

///////////////////////////////////////////////////////////////////////////

// System state makes use of the following enum definitions

////////////////////////////////////////
enum verbosity
  {
    bequiet = 0,
    beloud  = 1
  };

////////////////////////////////////////
enum system_mode
  {
   doing_time_keeping       = 0,
   doing_config_using_wifi_APenabled,
   doing_config_using_wifi_LocalLan,
   doing_config_using_uart
  };

unsigned char lock_system_mode = doing_time_keeping;	// assume time keeping at reset   
int is_system_doing_config_using_wifi_APenabled() {
  return lock_system_mode == doing_config_using_wifi_APenabled;
}
int is_system_doing_config_using_wifi_LocalLan() {
  return lock_system_mode == doing_config_using_wifi_LocalLan;
}
int is_system_doing_config_using_wifi() {
  return ((lock_system_mode == doing_config_using_wifi_APenabled) ||
	  (lock_system_mode == doing_config_using_wifi_LocalLan ));
}
int try_switch_to_doing_config_using_wifi( int x ) {
  if ((x == doing_config_using_wifi_APenabled) ||
      (x == doing_config_using_wifi_LocalLan )) {
    lock_system_mode = x;
    x = 1;			// switch is OK
  } else {
    x = 0;			// switch is not valid
  }
  return x;
}
int is_system_doing_time_keeping() { return lock_system_mode == doing_time_keeping; }
void system_switch_to_doing_time_keeping() { lock_system_mode = doing_time_keeping; }
void system_switch_to_doing_config_using_uart() { lock_system_mode = doing_config_using_uart; }
// Note: function enable_reconfigure_via_wifi currently has write access to lock_system_mode

////////////////////////////////////////
enum led_mode						// run-time selection of LED output
  {
    using_7seg_led = 0,
    using_dots_led = 1
  };
unsigned char led_display_is = using_7seg_led;		// assume LED display is 7seg at reset == 0
int is_system_using_7seg_led() { return led_display_is == using_7seg_led; }
int is_system_using_dots_led() { return led_display_is == using_dots_led; }
void system_switch_to_selected_led( int x ) {
  led_display_is = (1 & x)==0 ? using_7seg_led : using_dots_led;
}
int get_selected_led_as_ASCII(){ return '0' + (int) led_display_is; }

////////////////////////////////////////
enum hours_display_mode
  {
    using_24HOUR_display = 0,	// also equal to (1 & '2') i.e. user input char '2' can select
    using_12HOUR_display = 1	// also equal to (1 & '1') i.e. user input char '1' can select
  };
unsigned char HOURS_mode_is = using_12HOUR_display;	// assume hours display mode is 12 hour
int is_system_using_24HOUR_display() { return HOURS_mode_is == using_24HOUR_display; }
int is_system_using_12HOUR_display() { return HOURS_mode_is == using_12HOUR_display; }
void system_switch_to_selected_HOURS_mode( int x ) {	// odd x like '1',1 means use 12hr mode
  HOURS_mode_is = (1 & x)==0 ? using_24HOUR_display : using_12HOUR_display;
}
void system_toggle_selected_HOURS_mode() {
  HOURS_mode_is = (HOURS_mode_is==using_24HOUR_display)?using_12HOUR_display:using_24HOUR_display;
}

////////////////////////////////////////
enum time_sync_source
  {
    using_NTP_source = 0,
    using_GPS_source = 1
  };
unsigned char master_time_source_is = using_NTP_source;	// assume using NTP
unsigned char last_time_sync_was    = using_NTP_source;	// assume using NTP
int is_system_using_NTP_source() { return master_time_source_is == using_NTP_source; }
int is_system_using_GPS_source() { return master_time_source_is == using_GPS_source; }
void system_switch_to_selected_time_source( int x ) {	// odd x like '1',1 means GPS
  master_time_source_is = (1 & x)==0 ? using_NTP_source : using_GPS_source;
}
void system_save_last_time_sync_as_NTP() {
  last_time_sync_was = using_NTP_source;
}
void system_save_last_time_sync_as_GPS() {
  last_time_sync_was = using_GPS_source;
}
int was_system_last_time_sync_NTP() { return last_time_sync_was==using_NTP_source; }

////////////////////////////////////////
enum clock_internet_access
  {
    needed_for_time_sync = 0,	// NTP needs internet access
    optional_for_time_sync = 1	// GPS can work with or without
  };
unsigned char internet_access_is = needed_for_time_sync;// assume using NTP
int is_internet_access_needed() { return internet_access_is == needed_for_time_sync; }
void update_internet_access_needs() {
  if ( is_system_using_GPS_source() ) internet_access_is = optional_for_time_sync;
  else                                internet_access_is = needed_for_time_sync;
}

///////////////////////////////////////////////////////////////////////////

const char* wifiDisabled = "no.wifi";
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;

// Note: mDNS and WiFi access point modes need to create ssid or name strings
//
// Limit the number of attempts to connect to WiFi when it is not necessary for timekeeping
#define WIFI_CONNECT_ATTEMPTS_LIMIT 32

const char* WiFi_statuses[] =  { "WL_IDLE_STATUS=0",
				 "WL_NO_SSID_AVAIL=1",
				 "WL_SCAN_COMPLETED=2",
				 "WL_CONNECTED=3",
				 "WL_CONNECT_FAILED=4",
				 "WL_CONNECTION_LOST=5",
				 "WL_DISCONNECTED=6" };

#include <ESP8266mDNS.h>
#include <WiFiUdp.h>

////////////////////////////////////////
enum wifi_network_config_status
  {
    wifi_not_enabled = 0,
    wifi_STA_enabled = 1,
    wifi_AP_enabled  = 2,
    wifi_enabled     = 3
  };
unsigned char network_status = wifi_not_enabled;
int wifi_internet_is_enabled() { return network_status != wifi_not_enabled; }
char * get_ssid_char_ptr();
void update_network_status_based_on_ssid() {
  if (0 != strcmp( get_ssid_char_ptr(), (char*) wifiDisabled ))
    network_status = wifi_enabled;
  else
    network_status = wifi_not_enabled;
}
void update_network_status_based_on_connect( int is_connected ) {
  if ( is_connected ) {
    network_status = wifi_STA_enabled;
  } else {
    network_status = wifi_not_enabled;
    fsm_ntp_disable();			// can't happen if network is down
  }
}
void update_network_status_for_new_AP() {
  network_status = wifi_AP_enabled;
}

///////////////////////////////////////////////////////////////////////////

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include <EEPROM.h>

#include <ESP8266WebServer.h>
#include <ESP8266WebServerSecure.h>

// Displays
//
// . always drive the LCD
//   Note: currently use fmalpartida improved LCD access code from
//         https://github.com/fmalpartida/New-LiquidCrystal
#include "display_LCD.h"
//
// . optionally drive the dot matrix display
#if defined( support_DotMatrix_DISPLAY )
#include <SPI.h>			// arduino-cli build needs include in main file
#include "display_LEDdotmatrix.h"
void MDSTR( char* x ) { mdstr(x); };
#else
void MDSTR( x )       {};
#endif
//
// . optionally drive the 7-segment display
#if defined( support_7SEGMENT_DISPLAY )
void display_7seg_str4( char* );
#include "display_LED7seg.h"
#endif

///////////////////////////////////////////////////////////////////////////

// Settings for use of TCP and NTP

String IP_as_string;

// NTP access vars
//
IPAddress timeServerIP;			// NTP server address
char* optional_time_server = 0;		// !=0  : points into misc_bytes storage
String Time_of_last_sync="";		// !="" : sync time 

					// Connection for time synchronisation
unsigned int localPort = 2390;		// Local port to listen for UDP packets

char * ntpServerName = (char*)"0.pool.ntp.org";	// try different servers via
int timeSyncCounts = 0;				//   *ntpServerName='0'+(0x3 & ++timeSyncCounts);
						// i.e. overwrite the first byte of ntpServerName

// OTA update vars
//
const char * OTA_server_name = "placid.duckdns.org";
int OTA_port_number          = 48064;
const char * OTA_file_name   = "/Clock_Thing2.nodemcu.bin";
const char * URL_homedocs    = "http://placid.duckdns.org:48064/docs";
const char * URL_sources     = "https://github.com/pcvm/embedded_systems";

///////////////////////////////////////////////////////////////////////////

// User IO
//
// Interactive user communications is provided via the default arduino-style UART and via a telnet
// server, and also via an initial web page configure (using an internal wifi access point). A
// limited form of user IO makes use of the push button switch (momentary press or held press with
// timing measured), as well as detection of rapid toggling of the daylight savings switch. To
// support establishing a connection over a local network, an mDNS service is run and when a WiFi
// connection to the local network is active the device will appear as clockthing-HHHH.local where
// HHHH = hex representation of the least significant 16 bits of the interface MAC.

// The initial user interactive reconfigure is via the serial port but this switches to network
// (telnet) after clock establishes a tcpip listener.

////////////////////////////////////////
enum reconfigure_user_io
  {
   is_using_serial = 0,
   is_using_network
  };
enum reconfigure_user_io conf_mode_of_user = is_using_serial;

// Maximum number of telnet clients that can connect to this device
#define MAX_SRV_CLIENTS 1
//
WiFiServer server(23);
WiFiClient serverClients[MAX_SRV_CLIENTS];

// Programmer note: most general user messages are output via console->print() so that simple
// redirection between local serial comms and telnet can be supported via Stream pointer console.
//
Stream *console;
void io_set_to_serial() { console = &Serial;           conf_mode_of_user = is_using_serial;  }
void io_set_to_telnet() { console = &serverClients[0]; conf_mode_of_user = is_using_network; }
int  io_is_serial()     { return conf_mode_of_user == is_using_serial; }
//
// Char level output with flush
void putchr_telnet( char x ) { serverClients[0].print(x); serverClients[0].flush(); }
void putchr_serial( char x ) { Serial.print(x); Serial.flush(); }
//
// String output with flush
void putstr_telnet( char * x ) {
  if (serverClients[0] && serverClients[0].connected()){
    serverClients[0].write(x, strlen(x));
  }
  serverClients[0].flush();
}
//
// Char level input from telnet session via FIFO
short int getchr_telnet();
short int getchr_telnet_last();

#define DelSec(x) delay_with_time_keeping( x * 1000 )

short int push_button_switch__gpio_line = defaultButton;
unsigned char pb_gpio_is_defaultButton() {
  return push_button_switch__gpio_line == defaultButton;
}
void __set_pb_gpio_id( int x ) {	// should only ever be called once
  if ( x>=0 )
    push_button_switch__gpio_line = (0x1f & x);	// limit pin number to 0..31
  else
    push_button_switch__gpio_line = -1;	/* -ve input means switch will be disabled */
  return;
}
short int get_pb_gpio_id() {		// obtain current push button switch gpio line
  return push_button_switch__gpio_line;	// (-ve if disabled)
}
unsigned char read_pb_input() {
  if ( get_pb_gpio_id() >= 0 )
    return digitalRead(get_pb_gpio_id());
  else
    return LOW;
}

///////////////////////////////////////////////////////////////////////////

#if defined( haveGPS )
// Provides access to
// .  enable_gps_updates()
// .  background_gps_updates()
// .  int UTC_from_GPS_is_ready_to_retrieve()
// .  int get_gps_time(index)
#include "gps_ss.h"
#endif

///////////////////////////////////////////////////////////////////////////

// Activity displays
//
// Sensor reads are managed with a state machine which has an activity display
// on the LCD so define a few symbols
//
// - define 8 step symbols:           .     o    []     o     .     o    []     o
unsigned char ssm_transitions[] = {0xa5, 0xa1, 0xdb, 0xa1, 0xa5, 0xa1, 0xdb, 0xa1};
// - declare mod 8 index
unsigned char ssm_indexp = 0;
//
void ssm_update_status() {
  char status[2] = {0};		// NOTE: var ssm_index is not set up for DHT sensor

  status[0] = (char) ssm_transitions[ssm_indexp];
  lcd.setCursor( 0, 19 );
  lcd.print(status);
  ssm_indexp = 0x7 & (1+ssm_indexp);
}
void ssm_clear_status() {
  lcd.setCursor( 0, 19 );
  lcd.print(" ");	// overprint
  clr_ssm_index();
}
int clr_ssm_index() {
  ssm_indexp = 0;
  return 0;
}
#if defined( support_DotMatrix_DISPLAY )
//
// Doing any staged activity when a matrix display (or Serial) is in use can
// benefit from some spinning wheel chars to manage user patience/impatience
//
// - define 8 step symbols:       -\ |/-\ |/
unsigned char spinning_wheel[] = "-\\|/-\\|/";
// - declare mod 8 index
unsigned char sw_indexp = 0;
//
char get_next_spinning_wheel() {
  sw_indexp = 0x7 & (1+sw_indexp);
  return spinning_wheel[sw_indexp];
}
#endif

///////////////////////////////////////////////////////////////////////////

// Timer interrupt handling

// Choose 0.1s second time measurement resolution
#define Timer_IRQ_Interval_ms 100
#define Timer_IRQs_per_second  10

// Reading from an NTP time source will take some time and so uses a finite state machine (fsm) when
// handling NTP network requests/retries. Reading from a GPS source requires a simple timeout to
// handle the initial wait when a powered up GPS is making satellite contact (but otherwise the GPS
// time access is local and always available)
//
#define ntp_TIMEOUT (8*Timer_IRQs_per_second)	// retry every 8s
#define gps_READWAIT (1*60*1000)		// wait for up to 1 minute for satellite acquisition

extern "C" {
#include "user_interface.h"
}

// The timed event system uses var myTimer_variable and we provide a
// timerCallback() function to then regularly update our timer vars.
static os_timer_t myTimer_variable;
int timer_IRQ_handling_active = 0;

///////////////////////////

// Actual timekeeping

int clock_tick;		// increment in IRQ_handler()
int clock_has_ticked() { return clock_tick >= Timer_IRQs_per_second; }
int clock_colon = 0;	// flashing colon control is based on "ticks"
// . a "clock tick" update adjusts var clock_tick by subtracting Timer_IRQs_per_second
//   and not by clearing it so that a late update does not introduce error
void clock_tick_update() {
  clock_tick=clock_tick-Timer_IRQs_per_second;
  clock_colon = 1 & (~clock_colon);
}
// . an ability to adjust sub-seconds counts allows for more accurate time sync
void clock_tick_set_fractional_seconds( int count ) {
  if ( (count>=0) && (count < Timer_IRQs_per_second) )
    clock_tick = count;
}
			//
int tIRQ_downcounter;	// decrement in IRQ_handler()
inline void load_ms_for_tIRQ_downcounter( int ms ) { tIRQ_downcounter = ms / Timer_IRQ_Interval_ms; }
inline int ICACHE_FLASH_ATTR
            waiting_for_tIRQ_downcount()           { return tIRQ_downcounter > 0; }
			//
			// initialise vars associated with seconds
inline void clear_timekeeping_visible_vars()       { clock_tick=clock_colon=0; }
			// initialise vars associated with seconds and partial seconds
inline void initialise_timekeeping_all_vars()      { clock_tick=tIRQ_downcounter=clock_colon=0; }

int dt_secs, dt_mins, it_hours, dt_hours;	// daytime keeping vars (it_hours is an internal UTC hours)
int the_time_zone = 0;				// time offset with respect to UTC/GMT (set by a sync)
int dt_summer_offset;				// time offset due to daylight savings (read every second)

// daytime_1second_update() increments the internal time vars and display time vars for each second,
//                          and returns true if any of the displayed hours or mins change
//                          . any change in the summer time setting is noted and applied
// Rapid toggling of summer time switch:
//                          . multiple toggling events within a 10 second sliding window are counted
//                          . if this count is above TOGGLE_THRESHOLD, a user specified event can occur
//                            (currently demonstrated by displaying lower 8bits of IP on 7SEGMENT_DISPLAY)
#define	TOGGLE_TIME_WINDOW 10	// 10 second sliding window
#define TOGGLE_THRESHOLD    3
unsigned char dt_sliding_window = 0;
unsigned char dt_events_in_window = 0;
int get_IP_ls_byte0();

int daytime_1second_update() {
  int original_mins = dt_mins;		// note incoming dt_mins and dt_summer_offset
  int tmp = dt_summer_offset;
  unsigned char redraw = 0;

  update_summer_time(bequiet);
  if (tmp != dt_summer_offset) {	// has the summer time switch toggled?
    Serial.print("Summer offset: ");
    Serial.print(tmp);
    Serial.print("->");
    Serial.println(dt_summer_offset);
    if ( dt_sliding_window == 0 )
      dt_sliding_window = TOGGLE_TIME_WINDOW;
    dt_events_in_window++;
  }

  if (dt_sliding_window>0) {		// is sliding window active?
    if (--dt_sliding_window==0) {
      dt_events_in_window=0;		// reached end of active window so clear flags and redraw time on LED display
      redraw = 1;
    }
    if (dt_events_in_window > TOGGLE_THRESHOLD) {
#if defined( support_7SEGMENT_DISPLAY )
      display_7seg_NdigitCounterInit(3);		// throw up a momentary display of the least sig 8bits of
      display_7seg_NdigitCounter(get_IP_ls_byte0());	// IP on local network (fallback display if PB switch broken)
#endif
    }
  }

  // Update time-of-day
  if (++dt_secs == 60) {
    dt_secs = 0;
    if (++dt_mins == 60) {
      dt_mins = 0;
      if (++it_hours == 24)
	it_hours = 0;
    }
  }
  dt_hours = it_hours + dt_summer_offset;
  if (dt_hours >= 24) dt_hours = 0;
  manage_display_brightness(dt_hours);
    
  return redraw                    ||	// note if redraw of LED main 4 digits needed
         (tmp != dt_summer_offset) ||
         (original_mins != dt_mins);
}

///////////////////////////

int sensorRead_tick;

//
// Time out counter used by NTP connect and sync state machine
#define FSM_OFF (-1)
int fsm_ntp_upcounter=FSM_OFF;	// . set to FSM_OFF when inactive
inline int  ICACHE_FLASH_ATTR fsm_ntp_is_active() { return fsm_ntp_upcounter >= 0; }
inline int  fsm_ntp_get_count() { return fsm_ntp_upcounter; }
inline void fsm_ntp_disable()   { fsm_ntp_upcounter=FSM_OFF; }
inline void fsm_ntp_enable()    { fsm_ntp_upcounter=0; }

///////////////////////////

//
// Configure the ESP8266/FreeRTOS timer interrupt
//
// - this timerCallback updates our real-time variables for timekeeping
void ICACHE_FLASH_ATTR timerCallback(void *pArg) {
  (void)pArg;
  clock_tick++;					// manage time keeping for clock
  sensorRead_tick++;				// manage reading of humidity and temperature sensors
  if ( fsm_ntp_is_active() )
    fsm_ntp_upcounter++;			// upcounter used by finite state machines (idle when < 0)
  if ( waiting_for_tIRQ_downcount() )
    tIRQ_downcounter--;				// manage time delay downcounter           (idle when ==0)
  return;
}
//
// - this ESP timer config functions uses os_timer_setfn to specify that timerCallback() should be
//   called on a "timing event", and uses os_timer_arm() to specify the period these timing events
void ICACHE_FLASH_ATTR setup_timer_interrupt_system()
{
  // attach our time keeping function to ESP's timer call back
  //     os_timer_t *pTimer          ... address of time keeping data structure
  //     os_timer_func_t *pFunction  ... address of function to call with
  //                                     prototype void (*functionName)(void *pArg)
  //     void *pArg                  ... optional args
  //
  // Note: call os_timer_setfn() before os_timer_arm()
  //
  os_timer_setfn( &myTimer_variable, timerCallback, NULL);

  // set up Timer_IRQ_Interval_ms time keeping interrupts
  //     os_timer_t *pTimer          ... time keeping data structure
  //     uint32_t milliseconds       ... millisecs/interrupt
  //     bool repeat                 ... continuous if true
  //
  os_timer_arm( &myTimer_variable, Timer_IRQ_Interval_ms, true);

  timer_IRQ_handling_active = 1;
  return;
}

///////////////////////////////////////////////////////////////////////////

// Time display text

#define time_slen 12          // handles dd:dd:dd\0
char time_str[time_slen];     // latest time string value in 24hour form
char time_s12[time_slen];     // and equivalent time string in 12hour form
//
// update_time_strings() updates the main 24hour time display string
void update_time_strings(int the_colon, int dt_hours, int dt_mins, int dt_secs) {
  if (the_colon) sprintf( time_str, "%2d:%02d:%02d", dt_hours, dt_mins, dt_secs );
  else           sprintf( time_str, "%2d %02d %02d", dt_hours, dt_mins, dt_secs );

						// ensure default 12hour mode has values
  stpcpy(time_s12, time_str);
  if ( is_system_using_12HOUR_display() ) {
    char tmp_char = time_s12[2];
    if (dt_hours>12) sprintf( time_s12, "%2d", dt_hours-12 );	// modify hours if PM indicator in use
    time_s12[2] = tmp_char;
    if (dt_hours>=12)						// embed PM flag 0x80 on tens-of-hours
      time_s12[0] = PM_ENCODE_12HFLAG( time_s12[0] );
  }
}

///////////////////////////////////////////////////////////////////////////

//
// Activity support functions that support network services or time keeping in the background
//

// do_polling_for_active_services() is called regularly to maintain active services such
//                                  as system networking, GPS data retrieval, etc.
//
void do_polling_for_active_services() {
  delay(1);			// system call to support its real-time activities
  tcpip_loop();
  MDNS.update();
  process_network_io();
#if defined( haveGPS )
  background_gps_updates();	// (automatically enables software serial port)
#endif
}

// delay_with_time_keeping(int del) provides a time delay of del ms based on the
//   normal interrupt timerCallback updates if enabled, else uses a system delay().
//   With timer IRQs enabled, this time delay does not suspend time keeping.
// 
void delay_with_time_keeping(int ms) {
  if (ms <= 0) {
    delay(1);		// ensure at least one system call for "zero" delay or invalid -ve ms
    return;
  }
  if (timer_IRQ_handling_active == 0) {		// no user access to timer IRQs:
    delay(ms);					//   use delay() and return

  } else {					// have timer IRQs: use tIRQ_downcounter
    load_ms_for_tIRQ_downcounter( ms );		// set downcounter
    while ( waiting_for_tIRQ_downcount() ) {	// wait until downcounter idle
      do_polling_for_active_services();
    }
  }
  return;
}

///////////////////////////////////////////////////////////////////////////                                                                          

//
// Main tcpip loop provides telnet IO
//

const char *config_ssid = "ClockThing-";
String myNetworkName = "ClockThing-????";

// A tcpip telnet connection has characters read in process_network_io() and
// buffered in queue inBuffer with enqueue and dequeue indices ptr_getB_in and
// ptr_getB_out respectively. The function getchr_telnet() returns a character
// if available, or -1. The most recent result from getchr_telnet() is always
// available via getchr_telnet_last() to avoid any need to separately buffer.

const unsigned char LinBuffer = 128;	// a 2^n buffer length makes it easy to
const unsigned char mask_getB = 127;	// increment and mask the circular indices
char inBuffer[ LinBuffer ];
unsigned char ptr_getB_in = 0;
unsigned char ptr_getB_out = 0;

short int telnet_last_char = 0;
short int getchr_telnet_last() {
  return telnet_last_char;
}

// getchr_telnet() returns the next available user keystroke (via telnet), else returns -1 if no
//                 char available
//
short int getchr_telnet() {
  if (ptr_getB_in != ptr_getB_out ) {
    telnet_last_char = inBuffer[ ptr_getB_out ];// copy next byte for reading
    inBuffer[ ptr_getB_out++ ] = 0;		// clear char so read byes become 0
    ptr_getB_out = mask_getB & ptr_getB_out;	// update circular buffer index
    return telnet_last_char;
  } else
    return -1;
}

// Note: the tcpip/telnet code fragments are based on an example that can handle multiple
//       (MAX_SRV_CLIENTS) clients but we use only one.

// process_network_io() reads any available incoming data and saves it in circular buffer inBuffer
//
void process_network_io() {
  uint8_t i;
  int tmp;
  // check clients for data and save in Q
  for(i = 0; i < MAX_SRV_CLIENTS; i++) {
    if (serverClients[i] && serverClients[i].connected()) {
      while (serverClients[i].available()) {
        // return data from the telnet client and enqueue to inBuffer
	tmp = serverClients[i].read();
	if ( (tmp>=0) && (tmp<=0x7f)) {
	  inBuffer[ ptr_getB_in++ ] = tmp;
	  ptr_getB_in = mask_getB & ptr_getB_in;
	}
      }
    }
  }
}

void tcpip_loop() {
  uint8_t i;

  // check if there are any new clients wanting to connect
  if (server.hasClient()) {
    for(i = 0; i < MAX_SRV_CLIENTS; i++) {
      //find free/disconnected spot
      if (!serverClients[i] || !serverClients[i].connected()) {
        if(serverClients[i])
	  serverClients[i].stop();
        serverClients[i] = server.accept();
        Serial.print("New client: "); Serial.print(i);

	// flush input then welcome user
	while (serverClients[i].available())
	  serverClients[i].read();
	putstr_telnet( (char*) "\n\n\rWelcome to " );
	putstr_telnet( (char*) myNetworkName.c_str() );
	putstr_telnet( (char*) "\n\rPress h ENTER for help\n\n\r" );
        break;
      }
    }
    //no free/disconnected spot so reject
    if ( i == MAX_SRV_CLIENTS) {
      WiFiClient serverClient = server.accept();
      serverClient.stop();
      Serial.println("Connection rejected ");
    }
    Serial.println();
  }

}

///////////////////////////////////////////////////////////////////////////

//
// process_user_enquiry() provides support for a minimal number of user commands
//                        . when a user keystroke is available from either a UART or telnet link, it
//                          is read and decoded as a user command

void setup_LocalOTA_update();
void enable_OTA_update_from_main_server();
void enable_reconfigure_via_wifi(int);
void reconfigure_system_with_user_io();

void daytime_init_from_gps();
void daytime_init_from_ntp_trigger_stage1();

void config_misc_bytes_dump();

void process_user_enquiry() {
  char cmd;
  unsigned char utmp;

  char network_active = 0;
  if (Serial.available() > 0) {		// try for input from both Serial.read() and telnet connection
    cmd = Serial.read();
  } else if (getchr_telnet() >= 0) {
    network_active = 1;			// note telnet input (else default left as serial)
    cmd = getchr_telnet_last();
  } else {
    return;
  }
  if (network_active) io_set_to_telnet();
  else                io_set_to_serial();

#define i2s(x) String((int)x)

  if ((cmd>=' ')&&(cmd<=0x7f)) {	// user types a printable char always triggers a time-of-day status

    MDSTR( (char*) (String("-> ") + cmd).c_str() );

    String msg = String("\n\r");
    msg +=
      "Status: it_hours=" + i2s(it_hours) +
      ", dt_mins="        + i2s(dt_mins)  +
      ", dt_secs="        + i2s(dt_secs)  +
      ", dt_summer_offset=" + i2s(dt_summer_offset) + " (ADC=" + i2s(analogRead(0)) + ")" +
      ", display hr="     + i2s(dt_hours) +
      "\n\rLast time sync: " + Time_of_last_sync +

      "\n\r\n\rNetworking:"   +
      "\n\r  WIFI wlstatus  " + WiFi_statuses[WiFi.status()] +
      "\n\r  IP             " + WiFi.localIP().toString() +
      "\n\r  NTP server     " + timeServerIP.toString();
    if (optional_time_server!=0)
      msg += " (preference was " + String(optional_time_server) + ")";
    msg += "\n\r";

    msg += "\n\rPush button switch is on GPIO input " + ((get_pb_gpio_id() >= 0) ? i2s(get_pb_gpio_id()) : "<disabled>");
    if ( pb_gpio_is_defaultButton() )
      msg += " (default)";
    msg += "\n\rLED 7-seg display brightness = " + i2s(getDisplayBrightness()) + "/15";
    if ( is_system_using_7seg_led() ) msg += "\n\rLED display = 7-seg";
    if ( is_system_using_dots_led() ) msg += "\n\rLED display = dot-matrix";
#if defined( support_DotMatrix_DISPLAY )
    msg += "\n\r  (probe of dot-matrix returns ";
    if (confirm_LED_matrix_present(64)) msg += "true";
    else                                msg += "false";
    msg += ", requires MISO link)";
#endif
    msg += "\n\r  ("                   + String(( is_system_using_12HOUR_display() )?"12":"24") + " hour mode)";
    if ( LEDctl.brightness>=0 )
      msg += "\n\r  (brightness schedule ==> " + i2s(LEDctl.brightness) + "/15 during " + i2s(LEDctl.start_hour) + ".." + i2s(LEDctl.stop_hour) + ")";

    msg += "\n\rMaster time source = " + String(( is_system_using_NTP_source() )?"NTP":"GPS");
    msg += "\n\rSource code is online at " + String(URL_sources);
    msg += "\n\rDocumentation is kept at " + String(URL_homedocs);
    msg += "\n\r" + String(ReleaseMsg);
    msg += "\n";

    msg += "\n\rTime: " + String(time_str);
    msg += "\n";

    console->print( msg );

    utmp = 0xff;
    switch (toupper(cmd)) {		// very basic command decoding
    case 'H':
      msg  = "\n\rh --> this help";
      msg += "\n\r; --> toggle 12/24 hour display mode";
      msg += "\n\rq --> quit from this telnet session";
      msg += "\n\r";
      msg += "\n\rs --> sync using an NTP time server    (needs wifi setup)";
#if defined( haveGPS )
      msg += "\n\rt --> sync using the GPS receiver time (needs u-blox NEO-6M GPS module attached on GPIO input " + i2s(SS_RX) + ")";
#endif
      msg += "\n\r";
      msg += "\n\rc --> configure system via USB connected terminal session (putty, tio, picocom, even kermit!)";
      msg += "\n\rn --> configure system via a network telnet session";
      msg += "\n\ro --> view the lengthy info on configure system options";
      msg += "\n\r";
      msg += "\n\ru --> upgrade firmware to latest from server " + String(OTA_server_name) + ":" + String(OTA_port_number) + String(OTA_file_name);
      msg += "\n\r";
#if defined( haveGPS )
      msg += "\n\rg --> view current GPS module receiver comms buffer (developer use)";
#endif
      msg += "\n\rr --> trigger a system restart";
      msg += "\n\rd --> dump miscellaneous config bytes (developer use)";
      msg += "\n\rl --> perform network upgrade of firmware via web page http:" + IP_as_string + "/update (developer use)";
      msg += "\n\rw --> (EXPERIMENTAL) Web server config at URL http://" + IP_as_string + "/ (crazy developer use)";
      /* msg += "\n\r0..9 [ ] \\ , . / --> LED brightness 0..15"; --disable at present-- */
      msg += "\n\r";
      console->print( msg );
      break;
    case 'L':				// Network upgrade
      setup_LocalOTA_update();
      console->printf("\n\rAlternative URL is http:%s/update\n\n\r", IP_as_string.c_str());
      break;
    case 'U':				// Network upgrade
      console->printf("\n\rContacting server @ Lake Placid for firmware update\n\n\r");
      enable_OTA_update_from_main_server();
      break;
    case 'W':
      console->printf("\n\r(EXPERIMENTAL) Web server config management\n\n\r");
      if ( is_system_doing_time_keeping() ) {
	console->printf("Calling enable_reconfigure_via_wifi() with clock providing a local web server\n\r");
	enable_reconfigure_via_wifi( doing_config_using_wifi_LocalLan );
      } else {
	console->printf("Disabled local web server\n\r");	// ensure time keeping only
	system_switch_to_doing_time_keeping();			// (starves any wifi_server_*.handleClient() of CPU time)
      }
      break;
    case 'C':				// configure via usb-uart link
      if (network_active) break;	// (ignore if request came via network)
      reconfigure_system_with_user_io();
      break;
    case 'N':				// configure via telnet session
      if (! network_active) break;	// (ignore if request came via uart)
      reconfigure_system_with_user_io();
      break;
    case 'O': console->printf( "\n\rSPECIAL OPTIONS WHEN CONFIGURING A SYSTEM\n\r\n%s", special_options_user_info_cstr );
      break;
    case 'R':
      do_full_system_restart((char*) "    restart\n\n\r");	// never return
      break;
    case 'S':				// resync time of day
      console->printf("\n\rTriggering NTP sync (waiting 5 seconds)\n\r");
      delay( 500 );			// (allow no net comms in background)
      daytime_init_from_ntp_trigger_stage1();
      delay( 4500 );
      break;
#if defined( haveGPS )
    case 'T':
      console->printf("\n\rAccessing latest data from GPS\n\r");
      daytime_init_from_gps();
      break;
    case 'G':				// show latest GPS sentence (data packet)
      enable_gps_updates();		// include enable_gps_updates() to ensure serial port enabled
      msg = "\n\rGPS message buffer: "; msg += ssport1_wabuffer; msg += "\n\r";
      console->print( msg );
      if ( UTC_from_GPS_is_ready_to_retrieve() )
	console->printf("\rGPS time => %d:%02d:%02d.%02d\r",
			get_gps_time(0), get_gps_time(1), get_gps_time(2), get_gps_time(3));
      break;
#endif
    case 'D':
      console->printf("\n\rMiscellaneous config bytes:\n\r");
      config_misc_bytes_dump();
      console->println();
      break;
    case ';':				// toggle hours mode
      system_toggle_selected_HOURS_mode();
      console->printf("\n\rHours display --> %s hour mode\n\r", ( is_system_using_24HOUR_display() )?"24":"12" );
      break;
    case 'Q':				// end telnet session
      serverClients[0].stop();
      break;

      /*
       * Use 10 digits { '0' ... '9' } and also the
       *      6 chars  { '[' ']' '\\' ',' '.' '/' }
       * to set brightness utmp to value in 0...15
       */
    case '0': case '1': case '2': case '3':  case '4': case '5': case '6': case '7': case '8': case '9':
                               utmp = cmd - '0';
    // fall through
    case '[':  if (utmp==0xff) utmp = 10;	// a fresh entry into this cascade is marked by utmp==0xff
    // fall through
    case ']':  if (utmp==0xff) utmp = 11;	// " "
    // fall through
    case '\\': if (utmp==0xff) utmp = 12;	// " "
    // fall through
    case ',':  if (utmp==0xff) utmp = 13;	// " "
    // fall through
    case '.':  if (utmp==0xff) utmp = 14;	// " "
    // fall through
    case '/':  if (utmp==0xff) utmp = 15;	// " "
      setDisplayBrightnessP( console, utmp  ); break;

    default:
      console->printf("\n\rUnrecognised command: %c\n\r", char(cmd));
      break;
    }
  }
}

///////////////////////////////////////////////////////////////////////////

//
// Common IO functions allow a single call to update LCD display and Serial port.
// Use of the LED dot matrix display has not been included in these calls.
//

const char * msg_marker = "----------------------------------------";

void allPrint( const char* x ) {
  console->print(x);
  lcd.print(x);
}
void allPrintln( const char* x ) {
  console->println(x);
  lcd.print(x);
}
void allPrint( String x ) {
  console->print(x);
  lcd.print( (char *) x.c_str() );
}
void allPrintln( String x ) {
  console->println(x);
  lcd.print( (char *) x.c_str() );
}
void allPrintN(   int x ) { allPrint(   String(x) ); }
void allPrintlnN( int x ) { allPrintln( String(x) ); }
//
// Basic string reader for serial port or telnet port
//
short int user_input_getchr();
unsigned char user_input_isAvailable();
String my_read_string() {
  String x="";
  while (true) {
    do_polling_for_active_services();
    while ( ! user_input_isAvailable() ) delay(100);
    int tmp = user_input_getchr();
    if (tmp<0) continue;
    char key = tmp;
    if ((key==char(27)) || (key==char(10)) || (key==char(13))) break;
    if ((key<' ') || (key>'~')) {
      console->printf("?");
      continue;
    } else {
      x = x + key;
      console->printf("%c",key);
    }
  }
  return x;
}
  
///////////////////////////////////////////////////////////////////////////

// Notes on sensor reads and sequencing of slow events
//

// There can be a number of additional sensors attached for measurement of temperature and
// humidity. As we do not need high sampling rates for this information, a 10 second cycle is
// employed and individual sensors are accessed at particular seconds-based points in this 10s cycle
// i.e. expect to see say a Dallas Semi 1-wire temperature sensor measured at say the 5 second point
// or a humidity sensor read at the 3 second point. Just as the main time keeping relies on variable
// clock_tick that the timer interrupt callback increments at Timer_IRQs_per_second, the sensor read
// system relies on variable sensorRead_tick which is also incremented at Timer_IRQs_per_second.
//
// After the various sensors are read in each 10 second cycle, variable sensorRead_tick is cleared
// when it reaches 10 seconds and the process repeats.

int new_second;

const int sensorRollover             = 10*Timer_IRQs_per_second;  // sensors are read every 10s

// Define some constant values that, on a match with sensorRead_tick, mean
// that certain actions or events are triggered at certain time instants:
//
const int instant_to_readHumidity    =  3*Timer_IRQs_per_second;  // read H at 3s point
const int instant_to_readTemperature =  6*Timer_IRQs_per_second;  // read T at 6s point
const int instant_to_readDStemp      =  5*Timer_IRQs_per_second;  // read T at 5s point
//
// When a particular time instant is reached and an action triggered, a corresponding "in progress"
// flag is set to prevent further triggering.  There is an X_inProgress flag for each instant_to_X
// defined above.
//
byte ds_temp_read_inProgress, dht_humidity_read_inProgress, dht_temp_read_inProgress;
  
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#include "sensor_data_io.h"
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// Notes on ADC use
//
// A toggle switch controls the voltage fed into ADC channel 0 to provide a binary value that sets
// the binary value dt_summer_offset.
//
// . update_summer_time() reads, sets, and reports on summer time
//
void update_summer_time(enum verbosity verbose) {
  int sensor = analogRead(0);           // read ADC0 and binarise value to get final result
  if (verbose)
    console->print( String("Updating value from ADC0/summer_time input [") + String(sensor) );
  dt_summer_offset = (sensor<50)?0:1;
  if (verbose)
    console->println( String("] ... adjustment = ") + String(dt_summer_offset) );
}

///////////////////////////////////////////////////////////////////////////

// Display driver code
//
const int ts_pads = (lcd_COLS-8)/2;
//
void display_time( char* time_str ) {   // display time string
#if ! defined( haveTempSensor )
  for ( int x=0; x<ts_pads; x++ ) lcd.print(' ');
#else
  lcd.print(' ');
#endif

  lcd.print( time_str );

#if defined( haveTempSensor )
  lcd.print(" T=");
  lcd.print( get_temperature() );
#endif

#if defined( useDHTsensor )
  lcd.setCursor(0,3);                   // move to start of 4th line and overwrite
  lcd.print("          H=");
  lcd.print( get_humidity() );
#endif
}

#if defined( support_DotMatrix_DISPLAY )
// display_dm_String_Counter() is similar to display_7seg_Mchar_NdigCounter()
// in that a 3 digit counter is assumed
void display_dm_String_Counter( char *m, int n ) {
  char tmp[2 * MAX_DEVICES];
  sprintf(tmp, "%s%3d ", m, n);
  mdstr(tmp);
}
#endif
void display_LED_msg_cnt( char *msg, int cnt ) {
#if defined( support_7SEGMENT_DISPLAY )
  display_7seg_Mchar_NdigCounter(cnt,msg[0]);
#endif
#if defined( support_DotMatrix_DISPLAY )
  display_dm_String_Counter( msg, cnt );
#endif
  lcd.setCursor(0,3);
  lcd.print((char*) String(cnt).c_str());
  lcd.print("  ");
}

///////////////////////////////////////////////////////////////////////////

// Non-volatile storage
//
// The config data is kept in eeprom and managed by retrieve_clk_config() and
// save_clk_config(). This includes WiFi SSID and password, and clock operational settings (time
// zone, etc.). The integrity is checked via correct storage of pattern in byte header[] and updates
// variable configuration_is_OK.

bool configuration_is_OK;       // ==true on valid config data retrieval

// The eeprom storage used is defined as EEPROM_nBytesUsed (in bytes).
// It has sections {header ssid_Cstr pass_Cstr misc_bytes}
//
#define EEPROM_nBytesUsed 512
#define WIFI_parms_size 64      // assume this is large enough for ssid and pwd
#define MISC_parms_size 64      // misc. signed bytes e.g. time zone offset, alternative time server
#define n_ConfigPattern  8      // use pairs of complementary bytes as pattern (pairs sum to 0xff)
const byte header[] = {0xa5,0x5a,0x69,0x96,0x55,0xaa,0xff,0};

// Config details retained in eeprom storage are copied to vars {ssid_Cstr pass_Cstr misc_bytes}
// arrays that are located in RAM storage.
//
char ssid_Cstr[WIFI_parms_size];// network SSID (name)
char pass_Cstr[WIFI_parms_size];// network password
static signed char misc_bytes[MISC_parms_size];
				// byte 0 == UTC offset (signed char representing time zone)
				// remaining bytes are allocated as a char* with ' ' delimiters
				// as described in file eeprom_config_io.h
char * get_ssid_char_ptr() { return (char*) ssid_Cstr; }

////////////////////////////////////////
enum option_byte_sequences_id	// (includes original config items {'s','p','o'} for consistency)
  {
   id_wifi_ssid      = 's',	// original
   id_wifi_password  = 'p',	// original
   id_utc_offset     = 'o',	// original
				//          _______|THE FOLLOWING COMMENTS ARE ALSO USED IN GENERATING USER HELP:
   id_TimeServer     = 't',	// misc    |obsi_HLP specify alternative timeserver (e.g. t192.168.1.10 which helps on a standalone network)
   id_pb_switch_gpio = 'g',	// misc    |obsi_HLP specify alternative PB switch input line (e.g. g10 if original g16 input is damaged)
   id_HomeServer     = 'h',	// misc    |obsi_HLP specify the upgrade server (for testing use)
   id_DimSchedule    = 'd',	// misc    |obsi_HLP set dimming with Level 0..15, Start hour, End hour (e.g. d0,22,6 means lights low 10pm-6am)
   id_LED_select     = 'L',	// misc    |obsi_HLP specify LED display type (e.g. L0 for 7segment, L1 for DOT MATRIX, L1x for DOT MATRIX/rotated x=0..3 *90deg; typically leave as-is)
   id_HOURS_mode     = 'H',	// misc    |obsi_HLP select 12 or 24 hour display (e.g. H1 for 12 hour mode, H2 for 24 hour mode)
   id_GPS_time       = 'G'	// misc    |obsi_HLP select GPS time source (e.g. G1 for GPS time using attached GPS receiver; typically leave as-is)

				// additional definitions here can mean change to is_valid_misc_option_id(), and
				// a new an obsi_ help entry plus rerunning script user_help_strings.sh
  };

const String special_options_user_info = String( "Special options comprise an id letter and then some extra characters for a value." )
  + "\n\rUse a '|' or ' ' character to separate options, and enter one '|' to force an empty special option.\n\r" +
// Note: the next char* string is generated by script user_help_strings.sh TEXT PROCESSING THE ABOVE ENUM COMMENTS
#include "user_help_strings_1.h"
// Note: the next char* string contains hard-coded bytes defined in the above enum
  + "\n\rLong form examples:\
\n\rd0,22,6       - choose 8 hours of low level light between 10pm and 6am\
\n\rd0,22,6|L1    - dimming 10pm-6am but now using the DOT MATRIX display\
\n\rd0,22,6|L1|G1 - as above plus enable GPS time sync\
\n\rd0,22,6|L1|H1 - dimming 10pm-6am with DOT MATRIX display using 12 hour mode\
\n\rd0,22,6|L1|H2 - dimming 10pm-6am with DOT MATRIX display using 24 hour mode\
\n\r"
  // Warning: the next const char* help contains hard-coded chars that relate to short cut handling for enum option_byte_sequences_id definitions
  + "\
\n\rShortcut examples (easiest to use, notice a prefix 'G' selects GPS time sync):\
\n\rD  - dimmed LED 10pm-6am\
\n\rGD - dimmed LED 10pm-6am with GPS time source\
\n\rE  - dimmed LED 10pm-6am, DOT MATRIX\
\n\rF  - dimmed LED 10pm-6am, DOT MATRIX in 12 hour mode\
\n\rGF - dimmed LED 10pm-6am, DOT MATRIX in 12 hour mode with GPS time source\
\n\rP  - dimmed LED 10pm-6am, time server = 192.168.1.10\
\n\rQ  - dimmed LED 10pm-6am, time server = 192.168.1.10, DOT MATRIX\
\n\n\r";

////////////////////////////////////////

// is_valid_misc_option_id() is a hand coded function returning true if x is a member of a "misc" subset of
//                           enum option_byte_sequences_id comprising members { id_TimeServer ... }
unsigned char is_valid_misc_option_id( char ch ) {
  return ( (ch == id_TimeServer)     ||
	   (ch == id_pb_switch_gpio) ||
	   (ch == id_HomeServer)     ||
	   (ch == id_DimSchedule)    ||
	   (ch == id_LED_select)     ||
	   (ch == id_HOURS_mode)     ||
	   (ch == id_GPS_time) );
}

#include "eeprom_config_io.h"

///////////////////////////////////////////////////////////////////////////

//
// WiFi for accessing local network
//

const int NTP_PACKET_SIZE = 48;     // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE]; // Buffer to hold incoming and outgoing packets

WiFiUDP udp;                        // udp structure used in udp exchanges

// save full IPv4 number and provide easy access to IP bytes (big endian)
unsigned int  IP_as_uint32 = 0;
void setup_IP_as_uint32() {
  IP_as_uint32 = WiFi.localIP();
  return;
}
/* (using shift&mask instead of char* offset and mask) */
int get_IP_ls_byte0() { return (int) ((IP_as_uint32>>24) & 0xff); }
int get_IP_ls_byte1() { return (int) ((IP_as_uint32>>16) & 0xff); }
int get_IP_ls_byte2() { return (int) ((IP_as_uint32>>8 ) & 0xff); }
int get_IP_ls_byte3() { return (int) ( IP_as_uint32      & 0xff); }

// - display current IP and ntp port numbers
void show_netstat(int row1, int row2) {
  int row;
  if (row1>=0) {
    row=row1 & 0x3; lcd_clear_row(row);
    allPrint("IP: ");
    IP_as_string = WiFi.localIP().toString();
    allPrintln(IP_as_string);
  }
  if (row2>=0) {
    row=row2 & 0x3; lcd_clear_row(row);
    allPrint("UDP local port: ");
    allPrintN(udp.localPort());
  }
  Serial.println();
}

// - connect to network and open a udp port using the normal user SSID but if this is set
//   to wifiDisabled, then only do a single try to connect to the fallback network
//   
void setup_networking_WIFIclient() {
  update_network_status_based_on_ssid();

#if defined( support_DotMatrix_DISPLAY )
  String msg_with_ssid = String(ssid_Cstr);
#define MSG_LIMIT 4			/* display shows at least 4 chars so wrap msg_slide accordingly */
  char* msg_ptr = (char*) msg_with_ssid.c_str();
  int   msg_len = strlen( msg_ptr );
  int msg_slide = 0;
#endif

  lcd_clear_row(0);
  if ( wifi_internet_is_enabled() ) {
    allPrint("Connect: ");
    allPrint(ssid_Cstr);
  } else {
    allPrint("No user SSID" );
  }
    Serial.println();
    lcd_clear_row(1);

#if defined( support_DotMatrix_DISPLAY )
  delay(500);	/* pure delay as reading time for current display */
  mx.clear();
  mdstr( (char*) msg_ptr );
#endif

  WiFi.mode(WIFI_STA);			// ensure STA mode only
  WiFi.setPhyMode(WIFI_PHY_MODE_11G);	// select protocol that is 2.4GHz specific to avoid 5GHz
  
  int wifi_connect_tries = 0;

  //  WiFi.begin(ssid_Cstr, pass_Cstr);		// changed to wifiMulti, was WiFi.status()
  if ( wifi_internet_is_enabled() )			// first ssid choice comes from user
    wifiMulti.addAP(ssid_Cstr, pass_Cstr);
  else
    wifi_connect_tries = WIFI_CONNECT_ATTEMPTS_LIMIT;	// user not requesting wifi so only attempt fallback

  wifiMulti.addAP("localnet", "pass1234");	// second ssid choice ensures fallback for recovery

  while (wifiMulti.run() != WL_CONNECTED) {	// changed to wifiMulti, was WiFi.status() with .500 sec delay
						//                           delay(500);
    delay(1);					// system call to support its real-time activities
    digitalWrite(progressLED, (0x01 & wifi_connect_tries));	// update sync indicator
    allPrint(".");
    wifi_connect_tries++;

    if (( wifi_connect_tries >= WIFI_CONNECT_ATTEMPTS_LIMIT ) &&
	( ! is_internet_access_needed()        )) {
      update_network_status_based_on_connect( wifi_not_enabled );
#if defined( support_DotMatrix_DISPLAY )
      mx.clear();
      mdstr( (char*) "No wifi" );
#endif
      lcd_clear_row(1);
      Serial.println();
      allPrint("No wifi connection");
      Serial.println();
      display_7seg_activity_clear();
      delay(1000);
      return;					// give up, escape loop with IP_as_uint32 = 0
    }

    if ( (wifi_connect_tries % lcd_COLS) == 0 ) {
      lcd_clear_row(1);
      Serial.println();
    }

    display_7seg_progress_update();
#if defined( support_DotMatrix_DISPLAY )
    mx.clear();
    mdstr( (char*) (msg_ptr + msg_slide) );
    if ( ++msg_slide > (msg_len - MSG_LIMIT) ) msg_slide=0;
#endif
  }						// fall out of loop when connected
  update_network_status_based_on_connect( wifi_STA_enabled );

  display_7seg_activity_clear();
  lcd_clear_row(1);
  Serial.println();

  Serial.print("Allocated an ");
  allPrint("IP: ");
  allPrintln(WiFi.localIP().toString());

  setup_IP_as_uint32();				// 
#if defined( support_DotMatrix_DISPLAY )
  String ip_msg = String( get_IP_ls_byte3() ) + '.' + String( get_IP_ls_byte2() );
  msg_ptr = (char*) ip_msg.c_str();
  mx.clear();
  mdstr( msg_ptr );
  delay(1000);	/* pure delay (handles slower ESP-12e and human reading time) */
  ip_msg = String( get_IP_ls_byte1() ) + '.' + String( get_IP_ls_byte0() );
  msg_ptr = (char*) ip_msg.c_str();
  mx.clear();
  mdstr( msg_ptr );
  delay(1000);	/* pure delay (handles slower ESP-12e and human reading time) */
#else
  delay(1000);	/* pure delay */	/* ESP-12e seems slower than ESP-12F so at least 1 second */
#endif

  lcd_clear_row(0);
  allPrintln("Starting UDP");
  udp.begin(localPort); // [can dismantle with udp.stop()]
  delay(1);             // system call to support its real-time activities
  show_netstat(0,1);
}

///////////////////////////////////////////////////////////////////////////

//
// mDNS initialisation
// . from https://www.megunolink.com/documentation/connecting/mdns-browser/
// . updates myNetworkName with actual ssid that includes some MAC bytes

void AdvertiseServices() {
  // create a unique tmp_ssid by appending 4 hex chars from MAC to config_ssid
  // (-pm- #1 duplicated code to remove)
  char tmp_ssid[ strlen(config_ssid)     + 4 + 1 ];

  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);
  //
  sprintf(tmp_ssid, "%s%02x%02x", config_ssid,
          toupper(mac[WL_MAC_ADDR_LENGTH - 2]),
          toupper(mac[WL_MAC_ADDR_LENGTH - 1]));

  myNetworkName = tmp_ssid;
  if (MDNS.begin(myNetworkName.c_str())) {
    Serial.println(F("mDNS responder started"));
    Serial.print(F("Name = "));
    Serial.print(myNetworkName.c_str());
    Serial.println(".local\n\r");
 
    // Add service to MDNS-SD
    MDNS.addService("clockthing", "tcp", 23);
  } else {
    //    while (1) 
    {
      Serial.println(F("Error setting up MDNS responder"));
      delay(1000);
    }
  }
}

///////////////////////////////////////////////////////////////////////////

// Software updates can be handled on the local network (needs user command
// input) or via a web fetch from the "placid" server (after a long PB switch
// press)

#include "perform_OTA.h"

///////////////////////////////////////////////////////////////////////////

//
// Reset of the clock config needs a system restart
// . currently using ESP.reset() which triggers an internal real reset

void do_full_system_restart(char* rmsg) {
#if defined( support_DotMatrix_DISPLAY )
  int cnt=0;
  int len=strlen(rmsg);
  while ( (cnt<len) && (' '==*(rmsg+cnt)) ) cnt++;
  mdstr( (char*) (rmsg+cnt) );
#endif
#if defined( support_7SEGMENT_DISPLAY )
  ticker_7seg_str4( rmsg, 0, 500, 500 );
#endif

  ESP.eraseConfig();	// erase wifi setup so as to force use of our own eeprom config settings

  ESP.reset();		// instead of ESP.restart(), we use ESP.reset() to force a call to
			// __real_system_restart_local() and so avoid reuse of RAM based config settings
			// Refs: https://github.com/esp8266/Arduino/blob/master/cores/esp8266/Esp.cpp#L193
			//       https://github.com/esp8266/Arduino/issues/1494
}

///////////////////////////////////////////////////////////////////////////

// The system can be configured via wifi (using its own access point at startup)
//  or via user IO using UART communications (at startup or in normal operation)

void polling_loop_for_web_configure();
#include "configure_via_wifi_ap.h"
#include "configure_via_uart_or_telnet.h"

///////////////////////////////////////////////////////////////////////////

// Clock synchronisation is performed via udp access to an ntp server or by
// decoding a GPS response using a software serial port attached to a GPS receiver

#include "daytime_retrieval_from_source.h"

///////////////////////////////////////////////////////////////////////////

//
// System Initialisation
//
void setup() {
  // initialise various system-wide message strings
  special_options_user_info_cstr = (char*) special_options_user_info.c_str();

  timer_IRQ_handling_active = 0;

  for (int i=0; i<WIFI_parms_size; i++)		// clear ram copies of config data (before retrieval of EEPROM values)
    ssid_Cstr[i] = pass_Cstr[i] = 0;		// . wifi config
  for (int i=0; i<MISC_parms_size; i++)		// . options
    misc_bytes[i] = 0;				//
  the_time_zone = DefaultTimeZone;		// . time zone TZ

  io_set_to_serial();				// start in Serial comms mode

  Serial.begin(115200);				// 1 enable serial
  delay( 250 );
  Serial.println();
  Serial.println();
  Serial.println(msg_marker);
  Serial.println("*** Initialisation ***");
  Serial.printf("OTA compiled defaults = %s:%d%s\n\r", OTA_server_name, OTA_port_number, OTA_file_name);

  // Handle system transitions (typically unused so undefined)
  DO_SPECIAL_WELCOME
  retrieve_clk_config(bequiet);			// retrieve clock config (set up wifi/options/TZ/pb wiring from EEPROM)

  while (Serial.available() > 0) Serial.read();	//   flush serial input prior to possible read

  Serial.println("\n\rESP8266 pin status:");
  Serial.println(String("  GPIO10=")+String(digitalRead(10))+String(", GPIO16=")+String(digitalRead(16))+String(", ADC=")+String(analogRead(0)));
  // tests show that the pushbutton reads GPIOnn==0 if circuit connected i.e. pulldown resistor present

  update_summer_time(bequiet);			//   (GPIO default mode is input so pin read is OK)

						// 2 initialise I2C
  Wire.setClock( 100000 );
  Serial.println("I2C speed: 100000");
  Wire.begin( I2C_sda, I2C_scl );
  Serial.println("I2C bus initialised");
						// 3 initialise LCD attached to I2C
  lcd.begin(lcd_COLS,lcd_ROWS);
  lcd.clear();
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);       // activate backlight
  Serial.println("LCD initialised (if connected)");	//    (LCD setup so allPrint* functions available)
#if defined( support_DotMatrix_DISPLAY )
  Serial.println("Dot Matrix display driver present");
#endif
  lcd_clear_row(0); allPrintln( "Phillip's Clock" );
  lcd_clear_row(1); allPrintln( "          Thing" );
  lcd_clear_row(2); allPrintln( My_Version );

						// 4 initialise timer IRQ handling and timekeeping
  // clear vars used by timer call back (i.e. interrupt driven code)
  initialise_timekeeping_all_vars();
  // clear daytime keeping vars (i.e. vars related to time display digits)
  dt_clear();		// -pm- probably should remove/suppress NTP msg as this is no longer assured

  update_time_strings( 1, dt_hours,  dt_mins,  dt_secs);

  sensorRead_tick = -1; // let this counter start at the -1/Timer_IRQs_per_second point
                        // so events do not match whole seconds

			// clear some seconds oriented flags
  new_second = 0;
  clr_ssm_index();

						// 5 initialise timer IRQs
  setup_timer_interrupt_system();		// enable timer, can check that clock_tick is counting to verify timer evenrs
  Serial.printf("\n\rInitialised real time interrupts/callback handling, waiting for 5 events: ");
  { int i = clock_tick = 0;
    while (i<5) {
      delay(1);
      if (i == clock_tick) continue;
      i = clock_tick;
      Serial.printf(">%d ",i);
    }
    Serial.println(" OK, continuing");
  }						// delay_with_time_keeping() is now available
  Serial.printf("\n\rTIME SOURCE: ");
  if ( is_system_using_NTP_source() ) {
    Serial.printf("NTP servers\n\r\n\r");
  } else {
    enable_gps_updates();			// RX ss port also uses interrupts for input pin sampling
    Serial.printf("GPS receiver (RX comms enabled)\n\r\n\r");
  }
						// 6 initialise all displays
  lcd.home();					//   set LCD cursor to 0,0

#include "display_init.h"			// all active displays are now initialised

#if defined( useDHTsensor )
  Serial.println("DHT bus driver");
  dht_setup();				// initialise DHT sensor
#endif
#if defined( useDS1820sensor )
  Serial.println("1-wire bus driver");
  ow_setup();				// initialise 1-wire and one DS temp sensor
#endif
  delay_with_time_keeping(100);

  pinMode( get_pb_gpio_id(), INPUT );	// configure user input button
  Serial.print("Push button switch input initialised on gpio ");
  Serial.println( (int) get_pb_gpio_id() );
#if defined( useAnalogueTickTock )
  if ( get_pb_gpio_id() != CLK_TICK_OUTPUT ) {
    Serial.print("Analogue clock pulse output enabled on gpio ");
    Serial.println( (int) CLK_TICK_OUTPUT );
    pinMode( CLK_TICK_OUTPUT, OUTPUT );	// configure user input button
  } else {
    Serial.println("NOTE: Analogue clock pulse output not enabled due to clash with push button switch input");
  }
#endif
  delay_with_time_keeping(100);

  AdvertiseServices();
  Serial.println("mDNS initialised");
  delay_with_time_keeping(100);
  // Start the TCP server
  server.begin();
  server.setNoDelay(true);
  Serial.println("TCP server started");

  // Handle system transitions (typically unused so undefined)
  DO_SPECIAL_SETUP

  //
  // Handle re-entry of WiFi config and timezone config values when either the
  // user reset input (get_pb_gpio_id()) is set or the EEPROM config data is not available
  Serial.println(msg_marker);
  Serial.println("About to check if button pressed to trigger system configuration");
  Serial.println("You have 2 seconds to press any key on your keyboard to trigger a system configuration");

  MDSTR( (char*) "PB ???" );

  delay_with_time_keeping(2000);

  while (true) {	// dummy loop wrapper to allow for init skips from within this if-elseif structure
    delay(1);		// system call to support its real-time activities

    if (Serial.available() > 0) {
      Serial.println(" ... key press detected\n\r\nEntering reconfigure mode using serial comms");
      delay(1000); while (Serial.available() > 0) Serial.read();
      Serial.println("... waiting 5 seconds before entering serial port reconfigure (press '=' to continue to timekeeping)");
      delay(5000);
      if ((Serial.available() > 0) && (Serial.read()=='=')) break;	// skip init
      system_switch_to_doing_config_using_uart();
      io_set_to_serial();		// (assume serial comms as any telnet session accesses the reconfigure facility)
      reconfigure_system_with_user_io();				// never return as system restarts

    } else if (read_pb_input() == ButtonActive) {
      Serial.println("About to reconfigure using access point if push button now released");

      MDSTR((char*)"Release"); display_7seg_str4((char*)"Rel "); delay(1000);
      MDSTR((char*)"PB for");  display_7seg_str4((char*)"PB  "); delay(1000);
      MDSTR((char*)"config");  display_7seg_str4((char*)"for "); delay(1000);
                               display_7seg_str4((char*)"conf"); delay(1000);

      if (read_pb_input() == ButtonActive) {
	Serial.println("Push button still pressed, will disable for now and enter time keeping");
	__set_pb_gpio_id( -1 );		// ignoring reconfigure request as PB not released so disable PB as input line might be broken
	MDSTR((char*)"PBdisable"); ticker_7seg_str4( (char*) "PB off", 0, 250, 0 );
	break;				// and break to continue with normal time keeping
      }

      enable_reconfigure_via_wifi( doing_config_using_wifi_APenabled );
      return;				// return in a new system mode

    } else if (! retrieve_clk_config(beloud)) {
      Serial.println("Entering reconfigure mode using access point (no config retrieved)");
      enable_reconfigure_via_wifi( doing_config_using_wifi_APenabled );
      return;				// return in a new system mode

    }

    break;		// dummy loop end (do not delete)
  }			// fall out of loop means continue startup with existing configuration

  Serial.println();
  Serial.println();
  lcd_clear_row(0);
  allPrint("Time zone: UTC");
  if (the_time_zone > 0)
    allPrint("+");
  if (the_time_zone != 0)
    allPrintN(the_time_zone);

#if defined( support_7SEGMENT_DISPLAY )
  display_7seg_int3( the_time_zone );
#endif
#if defined( support_DotMatrix_DISPLAY )
  mx.clear();
  mdstr( (char*) (String("TZ ") + ((int)the_time_zone)).c_str() );
#endif

  Serial.println();
  Serial.println(msg_marker);
  Serial.println("Connecting to WiFi");
  lcd_clear_row(0);           // move to start of 2nd line
  display_time(time_str);
  setup_networking_WIFIclient();

  reinitialise_daytime_from_source();	// start timekeeping

#if defined( support_DotMatrix_DISPLAY )
  if ( ! is_system_using_dots_led() ) {
    mx.clear();
    mdstr( (char*) "Led off" );
  }
#endif

}

//
// Main user function
//
void loop() {
  // Handle special modes
  // . system upgrade
  if ( is_LocalOTA_active() ) {
    polling_loop_for_LocalOTA();
  }
  // . system config
  if ( is_system_doing_config_using_wifi() ) {
    polling_loop_for_web_configure();
    return;
  }
  // . else we are in normal startup or time keeping mode

  if ( fsm_ntp_is_active() )
    retrieve_UTC_using_NTP_stage2();

//
// Main loop() - time keeping
//

  if (clock_has_ticked()) {
    new_second = 1;
    clock_tick_update();
#if defined( useAnalogueTickTock )
    digitalWrite( CLK_TICK_OUTPUT, clock_colon );
#endif
    if (read_pb_input()==ButtonActive) {		// handle user PB activity
      lcd_clear_row(3);
      allPrintln("     Push button!");

      // Keeping the button pressed momentarily means NTP resync, but
      // keeping it pressed for certain intervals instead is also decoded as
      // a command

      int tenth_sec_count=0;
#define tenth_sec_DELAY 100     // delay count for the following 1/10sec timing loop
#if defined( support_7SEGMENT_DISPLAY )
      display_7seg_NdigitCounterInit(3);
#endif
#define TIMER_IN_SECS_RANGE(tmin, tmax) (((tmin)*10 <= tenth_sec_count) && ((tmax)*10 > tenth_sec_count))
#define TIMER_ODD                       ( 1 == (1 & (tenth_sec_count / 10)) )

      while (read_pb_input()==ButtonActive) {		/* debounce wait with various options */
	delay(1);					// system call to support its real-time activities

#if defined( support_7SEGMENT_DISPLAY ) || defined( support_DotMatrix_DISPLAY )

	if (TIMER_IN_SECS_RANGE(20,30)) {		/* -BUTTONDOWN-20-to-lt-30- */
	  if (tenth_sec_count == 20*10) { lcd_clear_row(3); allPrintln("     Mode flip 12/24"); }
	  if (TIMER_ODD) display_LED_msg_cnt((char*) "2412",tenth_sec_count);
	  else           display_LED_msg_cnt((char*) "FLIP",tenth_sec_count);
	}

	else if (TIMER_IN_SECS_RANGE(10,20)) {		/* -BUTTONDOWN-10-to-lt-20- */
	  if (tenth_sec_count == 10*10) { lcd_clear_row(3); allPrintln("     Release for OTA"); }
	  display_LED_msg_cnt((char*) "OTA:",tenth_sec_count);
	}

	else {						/* -BUTTONDOWN-less-than-10- */
	  if (tenth_sec_count ==     2) { lcd_clear_row(3); allPrintln("     Release to sync"); }
	  if      (TIMER_IN_SECS_RANGE(1,2.5)) display_LED_msg_cnt((char*) "IP ",0xff & (IP_as_uint32    ));	// big endian IP (most sig byte first)
	  else if (TIMER_IN_SECS_RANGE(2.5,4)) display_LED_msg_cnt((char*) "IP ",0xff & (IP_as_uint32>> 8));
	  else if (TIMER_IN_SECS_RANGE(4,5.5)) display_LED_msg_cnt((char*) "IP ",0xff & (IP_as_uint32>>16));
	  else if (TIMER_IN_SECS_RANGE(5.5,7)) display_LED_msg_cnt((char*) "IP ",0xff & (IP_as_uint32>>24));
	  else
	    display_LED_msg_cnt((char*) "Sec",tenth_sec_count);
	}

#endif

	delay_with_time_keeping(tenth_sec_DELAY);	// count time for UI effects
	tenth_sec_count++;
      }

      if ( tenth_sec_count <= 10*10) {		/* -BUTTONDOWN-less-than-10- */
	reinitialise_daytime_from_source();		// (resync clock)
	return;
      }
      if ( tenth_sec_count <= 20*10) {		/* -BUTTONDOWN-10-to-lt-20- */
	enable_OTA_update_from_main_server();		// (update from the main server (this call never returns))
      }
      if ( tenth_sec_count <= 30*10) {		/* -BUTTONDOWN-20-to-lt-30- */
	system_toggle_selected_HOURS_mode();
	return;
      }
    }						// end of sync request or system reconfig/updates

    delay(1);					// system call to support its real-time activities

    int flag_change_in_4digits = daytime_1second_update();

    update_time_strings( clock_colon, dt_hours, dt_mins, dt_secs);
    delay(1);					// system call to support its real-time activities
    lcd.setCursor(0,0);				// move to start of 1st row and overwrite
    delay(1);					// system call to support its real-time activities

    // LCD uses 24 hour mode always
    display_time( time_str );

    // LED options support 12 hour or 24 hour mode

#if defined( support_DotMatrix_DISPLAY )
    if ( is_system_using_dots_led() ) {

      if ( LEDctl.current_value>=0 )
	mx.control(MD_MAX72XX::INTENSITY, ((MAX_INTENSITY/2) * LEDctl.current_value)/max7segBrightness );

      char flagPM;							// will hold display character to flag PM time
      unsigned char tmp = time_s12[0];					// (save time_s12[0] before modification for md_message display)
      flagPM = IS_PM_12HFLAG_SET( tmp )?'"':' ';
      time_s12[0] = PM_REMOVE_12HFLAG( time_s12[0] );
      load_md_message( flagPM, ' ', time_s12, 5 );			// display includes flagPM
      time_s12[0] = tmp;						// (restore time_s12[0])
    }
#endif
#if defined( support_7SEGMENT_DISPLAY )
    if ( is_system_using_7seg_led() ) {
      display_7seg_4digits( time_s12, clock_colon, flag_change_in_4digits );	// display 12/24 hour data
    }
#endif

    if ((dt_hours == TimeSync_hours) &&
	(dt_mins  == TimeSync_mins ) &&
	(dt_secs  == TimeSync_secs )) {
      reinitialise_daytime_from_source();	// daily resync clock
    }
  }

  delay(1);					// system call to support its real-time activities

//
// Main loop() - handle sensor reads at specified intervals in a cycle of length sensorRollover
//
  
// Optional combined temperature and humidity sensor
#if defined( useDHTsensor )
  if (sensorRead_tick == instant_to_readHumidity &&
      dht_humidity_read_inProgress == 0) {
    dht_humidity_read_inProgress = 1;
    update_humidity();
  }
  if (sensorRead_tick == instant_to_readTemperature &&
      dht_temp_read_inProgress == 0) {
    //    lcd.setCursor( 0, 18 );
    //    lcd.print("+");                     // update sensor active indicator
    dht_temp_read_inProgress = 1;
    update_temperature();
  }
#endif

  // Optional temperature sensor
#if defined( useDS1820sensor )
  if (sensorRead_tick == instant_to_readDStemp &&
      ds_temp_read_inProgress == 0) {
    // lcd.setCursor( 0, 18 );
    // lcd.print(" +");                    // update sensor active indicator
    ds_temp_read_inProgress = 1;
    update_ds_temperature();
  }
#endif

  if ((sensorRead_tick < sensorRollover) && (new_second!=0))
    ssm_update_status();		// NOTE: var ssm_index is not set up for DHT sensor

  // Event triggering variable maintenance
  if (sensorRead_tick >= sensorRollover) {
    ssm_clear_status();
    sensorRead_tick = sensorRead_tick - sensorRollover;
    dht_humidity_read_inProgress = dht_temp_read_inProgress =
        ds_temp_read_inProgress = 0;
  }

  new_second = 0;

  // Ensure a pass over do_polling_for_active_services and process_user_enquiry
  //
  do_polling_for_active_services();
  process_user_enquiry();
}

///////////////////////////////////////////////////////////////////////////
