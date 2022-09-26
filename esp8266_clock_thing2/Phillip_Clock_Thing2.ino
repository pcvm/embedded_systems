/* -*- mode: c++; -*- */

/*
 * Phillip's Clock Thing is a world clock application built with off-the-shelf components.
 *										See -pm-
 *   p.musumeci@ieee.org Jan-2016/Jan-2017/June-2019/Feb-2020/Apr-2021
 */

#pragma GCC diagnostic ignored "-Wunused-parameter"

#define ThisRelease "2022 r41"

#include "notes_01_overview.h"	/* user notes */
#include "notes_10_developer.h"	/* circuit diagrams (ascii graphics) and software development notes */

/*
 *   Compile time options:
 *     . use_LED_DISPLAY              -> usually enabled
 *     . useDHTsensor useDS1820sensor -> DS1820 usually enabled
 *     . useAnalogueTickTock          -> enabled if external hardware needs a time signal
 *
 *   Run time options:
 *     . with use_LED_DISPLAY enabled, the variable led_display_is is usually set to
 *       using_7seg_led but the alternative using_dots_led is chosen for LED dot matrix use
 *       and relies on an optional setting of "L1".
 *
 *   TCPIP ports:
 *     . web servers use port 80, telnet uses 23 (MAX_SRV_CLIENTS=1), ntp uses 2390.
 */

// The defines { SPECIAL_WELCOME SPECIAL_SETUP } and file special_modes.h are normally unused
#define SPECIAL_WELCOME
#define SPECIAL_SETUP
// . the SPECIAL_* macros can do things like generate an executable to overwrite eeprom config and/or
//   change pin definitions for the main reset switch (when this sort of chip failure occurs, we have
//   no choice but to reprogram the device and reconfigure it for a workaround).
//   Example: to setup new wifi details, choose a time zone UTC +1, and change the push button wiring
//   to input gpio 10 (due to damage on the esp8266 chip), we can use a define SPECIAL_WELCOME such as
// Serial.printf("\nSpecial config for Fred\n\n"); save_clk_config("wifi_ssid", "wifi_passwd", (signed char) 1, "g10 ", beloud);
//   Fred's real name has been changed so no one knows he blew up the original gpio16 pin input.
//
//#include "special_modes.h"	// can include SPECIAL_macros here

///////////////////////////////////////////////////////////////////////////

/*
 * Select compile time options
 */

#define use_LED_DISPLAY 1	// include code for various types of LED displays

//#define useDHTsensor  1	// temperature/humidity via DHT11
#define useDS1820sensor 1	// temperature via DS1820

#if defined( useDHTsensor )	// (DHT has precedence)
#define TempMethod "dht"
#elif defined( useDS1820sensor )
#define TempMethod "ds"
#else
#define no_temperature_display	1
#define TempMethod " "
#endif

#define useAnalogueTickTock	// 1	// enable/disable analogue clock output pulse
// PIN ASSIGNMENT
#define CLK_TICK_OUTPUT		10	// Note: gpio10 is a fallback alternative PB if gpio16 input damaged

#if defined( use_LED_DISPLAY )
#define support_7SEGMENT_DISPLAY  1		// an Adafruit 7-segment display attached via I2C
#define support_DotMatrix_DISPLAY 1		// an MD dot matrix attached via SPI
#endif

// System id and version message strings (cpp concatenation)
//
#define My_Version    ThisRelease TempMethod	// use rD in place of vD due to 7-seg display
#define My_Hello "    he\to   " My_Version "    ----"
const char* ReleaseMsg = "S/W: " My_Version " :-)";

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
#define defaultButton 16	// gpio16 has a 4k7 pull down resistor and a
#define ButtonActive HIGH	// switch connecting it to 3.3V, press for HIGH
// -- progress flashing indicator
#define progressLED    2	// gpio2 == D4 == LED

// PIN ASSIGNMENT - SPI
#define SPI_miso  12
#define SPI_mosi  13
#define SPI_sck   14    // shift clock
#define SPI_ss    15    // device select

// PIN ASSIGNMENT - I2C
#define I2C_scl    5
#define I2C_sda    4
// - a shared pin is used for temperature and humidity IO, or analogue clock stepper
#define DHTPIN     2    // DHT module uses bidirectional serial protocol.
#define DSPIN      2    // Dallas Semi 1-wire protocol handles temperature sensor.
                        // Alternative IO pins: GPIO0 and if SPI input can be
                        // disabled, GPIO12 might also be an option

///////////////////////////////////////////////////////////////////////////

// System state makes use of the following enum definitions

enum verbosity
  {
    bequiet = 0,
    beloud  = 1
  };

enum system_mode
  {
   doing_time_keeping       = 0,
   doing_config_using_wifi_APenabled,
   doing_config_using_wifi_LocalLan,
   doing_config_using_uart
  };

unsigned char lock_system_mode = doing_time_keeping;	// assume time keeping at reset   

enum led_mode						// run-time selection of LED output
  {
    using_7seg_led = 0,
    using_dots_led = 1
  };
unsigned char led_display_is = using_7seg_led;		// assume LED display is 7seg at reset == 0
bool setup_manages_LED = false;				// used to remember if a config is read with display selection

enum hours_display_mode
  {
    using_24HOUR_display = 0,	// also equal to (1 & '2') i.e. user input char '2' can select
    using_12HOUR_display = 1	// also equal to (1 & '1') i.e. user input char '1' can select
  };
unsigned char HOURS_mode_is = using_12HOUR_display;	// assume hours display mode is 12 hour

///////////////////////////////////////////////////////////////////////////

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>

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
void display_7seg_str4( int, char* );
#include "display_LED7seg.h"
#endif

///////////////////////////////////////////////////////////////////////////

// TCP and NTP

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
// Interactive user communications is provided via the default arduino-style
// UART and via a telnet server, and also via an initial web page configure
// (using an internal wifi access point). A limited form of user IO makes use
// of the push button switch (momentary press or held press with timing
// measured), as well as detection of rapid toggling of the daylight savings
// switch. To support establishing a connection, an mDNS service is run.

// The initial user interactive reconfigure is via the serial port but this
// switches to network (telnet) after clock establishes a tcpip listener.
enum reconfigure_user_io
  {
   is_using_serial = 0,
   is_using_network
  };
enum reconfigure_user_io conf_mode_of_user = is_using_serial;

// mDNS and WiFi access point modes need to create ssid or name strings
// Maximum number of clients that can connect to this device
#define MAX_SRV_CLIENTS 1
WiFiServer server(23);
WiFiClient serverClients[MAX_SRV_CLIENTS];

int using_AP_for_config()  { return lock_system_mode == doing_config_using_wifi_APenabled; }
int using_LAN_for_config() { return lock_system_mode == doing_config_using_wifi_LocalLan; }
// enum lock_system_mode can also = {doing_time_keeping,doing_config_using_uart}

// Programmer note: most general user messages are output via console->print()
// so that simple redirection between local serial comms and telnet can be
// supported via Stream pointer console.
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
void __set_pb_gpio_id( int x ) {	// should only ever be called once
  if ( x>=0 )
    push_button_switch__gpio_line = (0x1f & x);	// limit pin number to 0..31
  else
    push_button_switch__gpio_line = -1;
  return;
}
short int get_pb_gpio_id() {		// obtain current push button switch gpio line
  return push_button_switch__gpio_line;
}
unsigned char read_pb_input() {
  if ( get_pb_gpio_id() >= 0 )
    return digitalRead(get_pb_gpio_id());
  else
    return LOW;
}

///////////////////////////////////////////////////////////////////////////

// Sensor state machines trigger activity displays on the LCD e.g. spinning cursor

int ssm_indexp;	// access via get/clr/inc functions
int get_ssm_index() {                                    return ssm_indexp; }
int clr_ssm_index() { ssm_indexp = 0;                    return 0; }
int inc_ssm_index() { ssm_indexp = 0x7 & (1+ssm_indexp); return ssm_indexp; }
// - define 8 step symbols:           .     o    []     o     .     o    []     o
unsigned char ssm_transitions[] = {0xa5, 0xa1, 0xdb, 0xa1, 0xa5, 0xa1, 0xdb, 0xa1};
// - define 8 spinning cursor chars (use if font version is A02)
//unsigned char ssm_transitions[] = "-\\|/-\\|/";
void ssm_update_status() {
  char status[2] = {0};		// NOTE: var ssm_index is not set up for DHT sensor

  status[0] = (char) ssm_transitions[get_ssm_index()];
  lcd.setCursor( 0, 19 );
  lcd.print(status);
  inc_ssm_index();
}
void ssm_clear_status() {
  lcd.setCursor( 0, 19 );
  lcd.print(" ");	// overprint
  clr_ssm_index();
}

///////////////////////////////////////////////////////////////////////////

// Timer interrupts

// Choose 0.1s second time measurement resolution
#define Timer_IRQ_Interval_ms 100
#define Timer_IRQs_per_second  10

#define ntp_TIMEOUT (8*Timer_IRQs_per_second)  // retry every 8s

extern "C" {
#include "user_interface.h"
}

os_timer_t myTimer_variable;
int timer_IRQ_handling_active = 0;

int clock_tick;		// increment in IRQ_handler()
			// tested/read/cleared in main()
int delay_tick;		// decrement in IRQ_handler()

int clock_colon = 0;				// flashing colon control
int dt_secs, dt_mins, it_hours, dt_hours;	// daytime keeping vars
int the_time_zone = 0;				// time offset with respect to UTC/GMT (set by a sync)
int summer_offset;				// time offset due to daylight savings (read every second)

void dt_initialise( unsigned long epoch ) {
  it_hours = (epoch % 86400L) / 3600; // convert epoch to hr,min,secs and adjust for time zone
  dt_mins  = (epoch % 3600) / 60;
  dt_secs  = epoch % 60;
  update_summer_time(beloud);
  it_hours = it_hours + the_time_zone;
  if (it_hours >= 24)
    it_hours = it_hours - 24;
  dt_hours = it_hours + summer_offset;
  if (dt_hours >= 24) dt_hours = 0;
}
void dt_clear() {
  dt_initialise(0);
}

// dt_1second_update() increments the internal time vars and display time vars for each second
//                     and returns true if any of the displayed hours or mins change
unsigned char sliding_window = 0;
unsigned char events_in_window = 0;
int get_IP_ls_byte0();
void dt_update_redraw(int secs) { sliding_window = 0x07 & secs; };	// schedule future redraw

int dt_1second_update() {
  int original_mins = dt_mins;		// note incoming dt_mins and summer_offset
  int tmp = summer_offset;
  unsigned char redraw = 0;

  update_summer_time(bequiet);
  if (tmp != summer_offset) {		// have detected a change in summer_offset
    Serial.print("summer_offset: ");
    Serial.print(tmp);
    Serial.print("->");
    Serial.println(summer_offset);
    if ( sliding_window == 0 )
      sliding_window = 10;
    events_in_window++;
  }

  if (sliding_window>0) {		// is sliding window active?
    if (--sliding_window==0) {
      events_in_window=0;		// reached end of active window so clear flags and redraw time on LED display
      redraw = 1;
    }
    if (events_in_window>3) {
#if defined( support_7SEGMENT_DISPLAY )
      display_7seg_NdigitCounterInit(3);		// throw up a momentary display of the least sig 8bits of
      display_7seg_NdigitCounter(get_IP_ls_byte0());	// IP on local network (fallback display if PB switch broken)
#endif
    }
  }

  if (++dt_secs == 60) {		// add 1 second to time
    dt_secs = 0;
    if (++dt_mins == 60) {
      dt_mins = 0;
      if (++it_hours == 24)
	it_hours = 0;
    }
  }
  dt_hours = it_hours + summer_offset;
  if (dt_hours >= 24) dt_hours = 0;
  manage_display_brightness(dt_hours);
    
  return redraw                    ||	// note if redraw of LED main 4 digits needed
         (tmp != summer_offset)    ||
         (original_mins != dt_mins);
}

int sensorRead_tick;

//
// Time out counter used by NTP connect and sync state machine
#define FSM_OFF (-1)
int set_clk_upcounter=FSM_OFF;	// . set to FSM_OFF when inactive

///////////////////////////

// Low level interrupt code

// define timerCallback (minimal length)
void timerCallback(void *pArg) {
  (void)pArg;
  clock_tick++;					// manage time keeping for clock
  sensorRead_tick++;				// manage reading of humidity and temperature sensors
  if (set_clk_upcounter >= 0)
    set_clk_upcounter++;			// upcounter used by finite state machines (idle when < 0)
  if (delay_tick > 0)
    delay_tick--;				// manage time delay downcounter           (idle when ==0)
  return;
}

// - calls ESP timer config and interrupt call back setup functions
void setup_timer_interrupt_system()
{
  // attach our time keeping function to ESP's timer call back by calling
  // void os_timer_setfn() with arguments
  //     os_timer_t *pTimer          ... address of time keeping data structure
  //     os_timer_func_t *pFunction  ... address of function to call with
  //                                     prototype void (*functionName)(void *pArg)
  //     void *pArg                  ... optional args
  //
  // Note: call os_timer_setfn() before os_timer_arm()

  os_timer_setfn( &myTimer_variable, timerCallback, NULL);

  // set up Timer_IRQ_Interval_ms time keeping interrupts by calling
  // void os_timer_arm() with arguments
  //     os_timer_t *pTimer          ... time keeping data structure
  //     uint32_t milliseconds       ... millisecs/interrupt
  //     bool repeat                 ... continuous if true

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
  if (HOURS_mode_is == using_12HOUR_display) {
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

// process_network_services() is called regularly to allow system networking
//                            to be maintained
//
void process_network_services() {
  delay(0);
  tcpip_loop();
  MDNS.update();
  process_network_io();
}

// delay_with_time_keeping(int del) provides a time delay of del ms based on the
//   normal interrupt timerCallback updates if enabled, else uses a system delay().
//   With timer IRQs enabled, this time delay does not suspend time keeping.
// 
void delay_with_time_keeping(int ms) {
  if (timer_IRQ_handling_active == 0) {		// no user access to timer IRQs:
    delay(ms);					//   use delay() and return

  } else {					// have timer IRQs: use delay_tick
    delay(0);					// guarantee at least one system call
    if (ms <= 0) return;
    delay_tick = ms / Timer_IRQ_Interval_ms;	// set downcounter
    while (delay_tick > 0) {
      process_network_services();
      delay(0);					// wait until downcounter idle
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

short int getchr_telnet() {
  if (ptr_getB_in != ptr_getB_out ) {
    telnet_last_char = inBuffer[ ptr_getB_out ];// copy next byte for reading
    inBuffer[ ptr_getB_out++ ] = 0;		// clear char so read byes become 0
    ptr_getB_out = mask_getB & ptr_getB_out;	// update circular buffer index
    return telnet_last_char;
  } else
    return -1;
}

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
        serverClients[i] = server.available();
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
      WiFiClient serverClient = server.available();
      serverClient.stop();
      Serial.println("Connection rejected ");
    }
    Serial.println();
  }

}

///////////////////////////////////////////////////////////////////////////

//
// process_user_enquiry() provides support for a minimal number of user
// commands when the clock is running
//

void setup_LocalOTA_update();
void enable_OTA_update_from_main_server();
void enable_reconfigure_via_wifi(int);
void reconfigure_system_with_user_io();

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
      "\n\r        summer_offset=" + i2s(summer_offset) + " (ADC=" + i2s(analogRead(0)) + ")" +
      ", display hr="     + i2s(dt_hours) +
      "\n\rNTP server="     + timeServerIP.toString();
    if (optional_time_server!=0)
      msg += " (preference was " + String(optional_time_server) + ")";
    msg +=   "\n\rTime of last clock synchronisation: " + Time_of_last_sync;
    msg +=   "\n\rPush button switch is on GPIO input " + i2s(get_pb_gpio_id());
    if (get_pb_gpio_id()==defaultButton)
      msg += " (default)";
    msg +=   "\n\rLED 7-seg display brightness = " + i2s(getDisplayBrightness()) + "/15";
    if (led_display_is == using_7seg_led) msg += "\n\rLED display = 7-seg";
    if (led_display_is == using_dots_led) msg += "\n\rLED display = dot-matrix";
#if defined( support_DotMatrix_DISPLAY )
    msg += "\n\r  (probe of dot-matrix returns ";
    if (confirm_LED_matrix_present(64)) msg += "true";
    else                                msg += "false";
    msg += ", requires MISO link)";
#endif
    if (HOURS_mode_is  == using_12HOUR_display) msg += "\n\r  (12 hour mode)";
    if (HOURS_mode_is  == using_24HOUR_display) msg += "\n\r  (24 hour mode)";
    if ( LEDctl.brightness>=0 )
      msg += "\n\r  (brightness schedule ==> " + i2s(LEDctl.brightness) + "/15 during " + i2s(LEDctl.start_hour) + ".." + i2s(LEDctl.stop_hour) + ")";

    msg +=   "\n\rSource code is online at " + String(URL_sources);
    msg +=   "\n\rDocumentation is kept at " + String(URL_homedocs);
    msg +=   "\n\r" + String(ReleaseMsg);
    msg +=   "\n";

    msg +=   "\n\rTime: " + String(time_str);
    msg +=   "\n";

    console->print( msg );

    utmp = 0xff;
    switch (toupper(cmd)) {		// very basic command decoding
    case 'H':
      msg  = "\n\rh --> this help";
      msg += "\n\rs --> sync with a time server now";
      msg += "\n\ru --> upgrade firmware to latest from server " + String(OTA_server_name) + ":" + String(OTA_port_number) + String(OTA_file_name);
      msg += "\n\rn --> configure system via network communications";
      msg += "\n\rc --> configure system via USB connected serial communications";
      msg += "\n\rt --> (EXPERIMENTAL) Web server config at URL http://" + IP_as_string + "/";
      msg += "\n\rr --> trigger a system restart";
      msg += "\n\rl --> perform network upgrade of firmware via web page http:" + IP_as_string + "/update";
      msg += "\n\rd --> dump miscellaneous config bytes";
      msg += "\n\r; --> toggle 12/24 hour display mode";
      msg += "\n\rq --> quit from telnet session";
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
    case 'T':
      console->printf("\n\r(EXPERIMENTAL) Web server config management\n\n\r");
      if ( lock_system_mode == doing_time_keeping ) {
	console->printf("Calling enable_reconfigure_via_wifi() with clock providing a local web server\n\r");
	enable_reconfigure_via_wifi( doing_config_using_wifi_LocalLan );
      } else {
	console->printf("Disabled local web server\n\r");	// ensure time keeping only
	lock_system_mode = doing_time_keeping;			// (starves any wifi_server_*.handleClient() of CPU time)
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
    case 'R':
      do_full_system_restart((char*) "    restart\n\n\r");	// never return
      break;
    case 'S':
      console->printf("\n\rTriggering NTP sync (waiting 5 seconds)\n\r");
      delay( 500 );			// (allow no net comms in background)
      set_clock_part1();
      delay( 4500 );
      break;
    case 'D':
      console->printf("\n\rMiscellaneous config bytes:\n\r");
      config_misc_bytes_dump();
      console->println();
      break;
    case ';':				// toggle hours mode
      HOURS_mode_is = (HOURS_mode_is==using_24HOUR_display)?using_12HOUR_display:using_24HOUR_display;
      console->printf("\n\rHours display --> %s hour mode\n\r", (HOURS_mode_is==using_24HOUR_display)?"24":"12" );
      break;
    case 'Q':				// end telnet session
      serverClients[0].stop();
      break;

      /* Use digits 0..9 and 6 chars "{}\\,./" to represent 16 values for a manual setting of brightness */
    case '0': case '1': case '2': case '3':  case '4': case '5': case '6': case '7': case '8': case '9':
                               utmp = cmd - '0';
    // fall through
    case '[':  if (utmp==0xff) utmp = 10;	// detect utmp!=0xff and save unique value, then fall through
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
// Common IO functions allow a single call to update LCD display and Serial port
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
    process_network_services();
    while ( ! user_input_isAvailable() ) delay(100);
    delay(0);
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

// Sequencing of slow events

// This application manages a number of relatively slow peripherals and
// processes. As the real-time interrupt handler timerCallback() is being
// triggered with a frequency of Timer_IRQs_per_second, the interrupt handler
// increments a variable sensorRead_tick which can then used for trigering
// user chosen actions.  When sensorRead_tick reaches the value sensorRollover
// = 10*Timer_IRQs_per_second, it is reset to 0 i.e. sensorRead_tick restarts
// counting from 0 every 10 seconds.

int new_second;

const int sensorRollover             = 10*Timer_IRQs_per_second;  // sensors are read every 10s

// Define some constant values that, on a match with sensorRead_tick, mean
// that certain actions or events are triggered at certain time instants:
//
const int instant_to_readHumidity    =  3*Timer_IRQs_per_second;  // read H at 3s point
const int instant_to_readTemperature =  6*Timer_IRQs_per_second;  // read T at 6s point
const int instant_to_readDStemp      =  5*Timer_IRQs_per_second;  // read T at 5s point
//
// When a particular time instant is reached and an action triggered, a
// corresponding "in progress" flag is set to prevent further triggering.
// There is an X_inProgress flag for each instant_to_X defined above.
//
byte ds_temp_read_inProgress, dht_humidity_read_inProgress, dht_temp_read_inProgress;
  
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#include "sensor_data_io.h"
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

//
// A toggle switch controls the voltage fed into ADC channel 0 to provide a
// binary value that sets the binary value summer_offset
//
// . update_summer_time() reads, sets, and reports on summer time
//   Notes: - this was only ever called during setup (when timer callbacks might not be active)
//          - as we now read the toggle switch at the timer callback rate, we now also have
//            summer_offset being updated continuously in toggle_switch_update()
//          - it is retained as it also displays ADC value which helps in checking
void update_summer_time(enum verbosity verbose) {
  int sensor = analogRead(0);           // read ADC0 and binarise value to get final result
  if (verbose)
    console->print( String("Updating value from ADC0/summer_time input [") + String(sensor) );
  summer_offset = (sensor<50)?0:1;
  if (verbose)
    console->println( String("] ... adjustment = ") + String(summer_offset) );
}

///////////////////////////////////////////////////////////////////////////
//
const int ts_pads = (lcd_COLS-8)/2;
//
void display_time( char* time_str ) {   // display time string
#if defined( no_temperature_display )
  for ( int x=0; x<ts_pads; x++ ) lcd.print(' ');
#else
  lcd.print(' ');
#endif

  lcd.print( time_str );

#if !defined( no_temperature_display )
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

// The system keeps config data in eeprom as managed by retrieve_clk_config()
// and save_clk_config(). The integrity is checked via correct storage of
// pattern in byte header[] and updates variable configuration_is_OK.

bool configuration_is_OK;       // ==true on valid config data retrieval

// The eeprom storage used is defined as EEPROM_nBytesUsed (in bytes).
// It has sections {header ssid_Cstr pass_Cstr misc_bytes}
//
#define EEPROM_nBytesUsed 512
#define WIFI_parms_size 64      // assume this is large enough for ssid and pwd
#define MISC_parms_size 64      // misc. signed bytes e.g. time zone offset, alternative time server
#define n_ConfigPattern  8      // use pairs of complementary bytes as pattern (pairs sum to 0xff)
const byte header[] = {0xa5,0x5a,0x69,0x96,0x55,0xaa,0xff,0};

char ssid_Cstr[WIFI_parms_size];// network SSID (name)
char pass_Cstr[WIFI_parms_size];// network password
static signed char misc_bytes[MISC_parms_size];
				// byte 0 == UTC offset (signed char representing time zone)
				// remaining bytes are allocated as a char* with ' ' delimiters
				// as described in file eeprom_config_io.h

enum option_byte_sequences_id	// (includes original config items {'s','p','o'} for consistency)
  {
   id_wifi_ssid      = 's',
   id_wifi_password  = 'p',
   id_utc_offset     = 'o',
   id_TimeServer     = 't',
   id_pb_switch_gpio = 'g',
   id_HomeServer     = 'h',
   id_DimSchedule    = 'd',
   id_LED_select     = 'L',
   id_HOURS_mode     = 'H'		/* -pm- add id_lowBW_sensor */
  };

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
    row=row1 & 0x3; clear_lcd_row(row);
    allPrint("IP: ");
    IP_as_string = WiFi.localIP().toString();
    allPrintln(IP_as_string);
  }
  if (row2>=0) {
    row=row2 & 0x3; clear_lcd_row(row);
    allPrint("UDP local port: ");
    allPrintN(udp.localPort());
  }
  Serial.println();
}

// - connect to network and open a udp port
void setup_networking_WIFIclient() {
#if defined( support_DotMatrix_DISPLAY )
  String msg_with_ssid = String("  ") + String(ssid_Cstr);
#define MSG_LIMIT 4			/* display shows at least 4 chars so wrap msg_slide accordingly */
  char* msg_ptr = (char*) msg_with_ssid.c_str();
  int   msg_len = strlen( msg_ptr );
  int msg_slide = 0;
#endif
  clear_lcd_row(0);
  allPrint("Connect: ");
  allPrint(ssid_Cstr);
  Serial.println();
  clear_lcd_row(1);
#if defined( support_DotMatrix_DISPLAY )
  delay(500);	/* pure delay as reading time for current display */
  mx.clear();
  mdstr( (char*) msg_ptr );
#endif

  WiFi.mode(WIFI_STA);			// ensure STA mode only
  WiFi.begin(ssid_Cstr, pass_Cstr);

  int tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);	/* pure delay */
    digitalWrite(progressLED, (0x01 & tries));	// update sync indicator
    allPrint(".");
    if (++tries >= lcd_COLS) {
      tries = 0;
      clear_lcd_row(1);
      Serial.println();
    }
    display_7seg_progress_update();
#if defined( support_DotMatrix_DISPLAY )
    mx.clear();
    mdstr( (char*) (msg_ptr + msg_slide) );
    if ( ++msg_slide > (msg_len - MSG_LIMIT) ) msg_slide=0;
#endif
  }

  display_7seg_activity_clear();
  clear_lcd_row(1);
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

  clear_lcd_row(0);
  allPrintln("Starting UDP");
  udp.begin(localPort); // [can dismantle with udp.stop()]
  delay(0);             // system call to support its real-time activities
  show_netstat(0,1);
}

// send an NTP request to the time server at the given address
void sendNTPpacket( IPAddress &anAddress ) {
  clear_lcd_row(0);
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
    Serial.print(F(" Name = "));
    Serial.println(myNetworkName.c_str());
 
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

//
// Clock synchronisation via udp access to ntp server
//

void set_clock_part1() {
#if defined( support_7SEGMENT_DISPLAY )
  flag_NTP_sync_active;		// (macro to set an indicator LED)
#endif
  set_clk_upcounter = 0;

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

  delay(0);                     // system call to support its real-time activities
  sendNTPpacket(timeServerIP);  // send an NTP packet to a time server
  delay(0);                     // system call to support its real-time activities

  lcd_setCursorSyncFlag;        // mark sync as in process
  timeSyncCounts++;
  allPrint("*");
}

void set_clock_part2() {
  int cb = udp.parsePacket();
  if (!cb) {				// No packet...
    clear_lcd_row(0);
    if (set_clk_upcounter > ntp_TIMEOUT) {
      allPrintln("NTP timeout, retry");
      delay_with_time_keeping(2000);
      set_clock_part1();
    } else {
      allPrintln("No packet yet");
      delay_with_time_keeping(300);	// (was 250)
    }
  } else {				// Got packet so decode
    set_clk_upcounter = FSM_OFF;

    clear_lcd_row(0);
    allPrint("Rx pkt len=");
    allPrintN(cb);
    console->println();

                                            // received a packet so decode
    udp.read(packetBuffer, NTP_PACKET_SIZE);// read packet into the buffer
    delay(0);                               // system call to support its real-time activities

    // Note: the timestamp, as seconds since Jan 1 1900, starts at byte 40 of
    //       the received packet and is encoded as a big-endian 32b word
    //
    unsigned long secsSince1900 = 0;
    for (int i = 0; i < 4; i++)         // merge in 4 bytes, assuming most significant first
      secsSince1900 =
          (secsSince1900 << 8) | (unsigned long)packetBuffer[40 + i];

                                        // process time data in Unix form (seconds since
                                        // 1/1/1970) therefore subtract 70 year offset
    const unsigned long seventyYears = 2208988800UL;

                                        // epoch in Unix form of UTC aka GMT
    unsigned long epoch = secsSince1900 - seventyYears;

    int adjust_ms = dt_mins * 60 + dt_secs;		// save mins/secs as ms in order to report update size
    dt_initialise( epoch );
    adjust_ms =    (dt_mins * 60 + dt_secs) - adjust_ms;// estimate (roughly) the correction
    adjust_ms = adjust_ms % 3600;			// (assume < an hour)

    clear_lcd_row(3);	// erase resync announcement
    clear_lcd_row(1);
    allPrint("Sync@");
    update_time_strings( 1, dt_hours,  dt_mins,  dt_secs);
    Time_of_last_sync = time_str;

    allPrintln(time_str);
#if defined( support_7SEGMENT_DISPLAY )
    display_7seg_4digits(time_s12,clock_colon,1);
    flag_NTP_sync_done;
#endif
#if defined( support_DotMatrix_DISPLAY )
    mx.clear();
    mdstr( (char*) "sync ok" );
#endif

    if (adjust_ms >= 0) lcd.print(" +");
    else                lcd.print(' ');
    lcd.print(adjust_ms);

    lcd_setCursorSyncFlag;      // mark sync as complete
    lcd.print(' ');

    show_netstat( 2,-1 );       // clear extra LCD rows and fill with IP data
    clear_lcd_row(3); allPrintln( String(" ") + ReleaseMsg );

    // clear pending time display updates (irrelevant as new time synchronised)
    clock_tick=clock_colon=0;
  }
}

///////////////////////////////////////////////////////////////////////////

//
// System Initialisation
//
void setup() {
  timer_IRQ_handling_active = 0;

  for (int i=0; i<WIFI_parms_size; i++)		// clear ram copies of config data (before retrieval of EEPROM values)
    ssid_Cstr[i] = pass_Cstr[i] = 0;		// . wifi config
  for (int i=0; i<MISC_parms_size; i++)		// . options
    misc_bytes[i] = 0;				//
  the_time_zone = DefaultTimeZone;		// . time zone TZ

  io_set_to_serial();				// start in Serial comms mode

  Serial.begin(115200);				// 1 enable serial
  delay_with_time_keeping( 250 );
  Serial.println();
  Serial.println();
  Serial.println(msg_marker);
  Serial.println("*** Initialisation ***");
  Serial.printf(" OTA compiled defaults = %s:%d%s\n\r", OTA_server_name, OTA_port_number, OTA_file_name);

  // Handle system transitions (typically unused so undefined)
  SPECIAL_WELCOME
  retrieve_clk_config(bequiet);			// retrieve clock config (set up wifi/options/TZ/pb wiring from EEPROM)

  while (Serial.available() > 0) Serial.read();	//   flush serial input prior to possible read

  Serial.println("\n\rESP8266 pin status:");
  Serial.println(String("  GPIO10=")+String(digitalRead(10))+String(", GPIO16=")+String(digitalRead(16))+String(", ADC=")+String(analogRead(0)));
  // tests show that the pushbutton reads GPIOnn==0 if circuit connected i.e. pulldown resistor present

  update_summer_time(bequiet);			//   (GPIO default mode is input so pin read is OK)

						// 2 initialise I2C
  Wire.begin( I2C_sda, I2C_scl );
  Serial.println(" I2C bus initialised");
  // Can also set transfer speed e.g. Wire.setClock( 100000 )

						// 3 initialise LCD attached to I2C
  lcd.begin(lcd_COLS,lcd_ROWS);
  lcd.clear();
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);       // activate backlight
  Serial.println(" LCD initialised (if connected)");	//    (LCD setup so allPrint* functions available)
#if defined( support_DotMatrix_DISPLAY )
  Serial.println(" Dot Matrix display driver present");
#endif
  clear_lcd_row(0); allPrintln( "Phillip's Clock" );
  clear_lcd_row(1); allPrintln( "          Thing" );
  clear_lcd_row(2); allPrintln( My_Version );

						// 4 initialise timer IRQ handling and timekeeping
  // clear vars used by timer call back (i.e. interrupt driven code)
  clock_tick=delay_tick=clock_colon=0;

  // clear daytime keeping vars (i.e. vars related to time display digits)
  dt_clear();
  update_time_strings( 1, dt_hours,  dt_mins,  dt_secs);

  sensorRead_tick = -1; // let this counter start at the -1/Timer_IRQs_per_second point
                        // so events do not match whole seconds

			// clear some seconds oriented flags
  new_second = 0;
  clr_ssm_index();

						// 5 initialise timer IRQs
  setup_timer_interrupt_system();		//   delay_with_time_keeping() is now available
  Serial.println(" Timer interrupts enabled");

						// 6 initialise all displays
  lcd.home();					//   set LCD cursor to 0,0

#include "display_init.h"
			// all active displays are now initialised

					// The useDHTsensor and useDS1820sensor options are exclusive
#if defined( useDHTsensor )
  Serial.println(" (DHT temperature)");
  dht_setup();				// initialise DHT sensor
#endif
#if defined( useDS1820sensor )
  Serial.println(" 1-wire bus driver");
  ow_setup();				// initialise 1-wire and one DS temp sensor
#endif
  delay_with_time_keeping(100);

  pinMode( get_pb_gpio_id(), INPUT );	// configure user input button
  Serial.print(" Push button switch input initialised on gpio ");
  Serial.println( (int) get_pb_gpio_id() );
#if defined( useAnalogueTickTock )
  if ( get_pb_gpio_id() != CLK_TICK_OUTPUT ) {
    Serial.print(" Analogue clock pulse output enabled on gpio ");
    Serial.println( (int) CLK_TICK_OUTPUT );
    pinMode( CLK_TICK_OUTPUT, OUTPUT );	// configure user input button
  } else {
    Serial.println("NOTE: Analogue clock pulse output not enabled due to clash with push button switch input");
  }
#endif
  delay_with_time_keeping(100);

  AdvertiseServices();
  Serial.println(" mDNS initialised");
  delay_with_time_keeping(100);
  // Start the TCP server
  server.begin();
  server.setNoDelay(true);
  Serial.println("TCP server started");

  // Handle system transitions (typically unused so undefined)
  SPECIAL_SETUP

  //
  // Handle re-entry of WiFi config and timezone config values when either the
  // user reset input (get_pb_gpio_id()) is set or the EEPROM config data is not available
  Serial.println(msg_marker);
  Serial.println("About to check if button pressed to trigger system configuration");
  Serial.println("You have 2 seconds to press any key on your keyboard to trigger a system configuration");

  MDSTR( (char*) "PB ???" );

  delay_with_time_keeping(2000);

  while (true) {	// dummy loop wrapper to allow for init skips from within this if-elseif structure

    if (Serial.available() > 0) {
      Serial.println();
      Serial.println("Entering reconfigure mode using serial comms (key press detected)");
      delay(1000); while (Serial.available() > 0) Serial.read();
      Serial.println("... waiting 5 seconds before starting start serial port reconfigure (press '=' to continue to timekeeping)");
      delay(5000);
      if ((Serial.available() > 0) && (Serial.read()=='=')) break;	// skip init
      lock_system_mode = doing_config_using_uart;
      io_set_to_serial();		// (assume serial comms as any telnet session accesses the reconfigure facility)
      reconfigure_system_with_user_io();				// never return as system restarts

    } else if (read_pb_input() == ButtonActive) {
      Serial.println("About to reconfigure using access point if push button now released");

      MDSTR((char*)"Release"); display_7seg_str4(0,(char*)"Rel "); delay(1000);
      MDSTR((char*)"PB for");  display_7seg_str4(0,(char*)"PB  "); delay(1000);
      MDSTR((char*)"config");  display_7seg_str4(0,(char*)"for "); delay(1000);
                               display_7seg_str4(0,(char*)"conf"); delay(1000);

      if (read_pb_input() == ButtonActive) {
	Serial.println("Push button still pressed, will disable for now and enter time keeping");
	__set_pb_gpio_id( -1 );		// ignoring reconfigure request as PB not released so disable PB as input line might be broken
	MDSTR((char*)"PBdisable"); ticker_7seg_str4( (char*) "PB off", 0, 250, 0 );
	break;				// and break to continue with normal time keeping
      }

      enable_reconfigure_via_wifi( doing_config_using_wifi_APenabled );
      return;				// return with new lock_system_mode

    } else if (! retrieve_clk_config(beloud)) {
      Serial.println("Entering reconfigure mode using access point (no config retrieved)");
      enable_reconfigure_via_wifi( doing_config_using_wifi_APenabled );
      return;				// return with new lock_system_mode

    }

    break;		// dummy loop end (do not delete)
  }

  Serial.println();
  Serial.println();
  clear_lcd_row(0);
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
  clear_lcd_row(0);           // move to start of 2nd line
  display_time(time_str);
  setup_networking_WIFIclient();
  set_clock_part1();          // start clock resync phases
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
  if ((lock_system_mode == doing_config_using_wifi_APenabled) ||
      (lock_system_mode == doing_config_using_wifi_LocalLan )) {
    polling_loop_for_web_configure();
    return;
  }
  // . else we are in normal startup or time keeping mode

  if (set_clk_upcounter != FSM_OFF)
    set_clock_part2();

  if (clock_tick >= Timer_IRQs_per_second) {
      new_second = 1;

      clock_tick=clock_tick-Timer_IRQs_per_second;	// could make this uninterruptable to be really
							// sure of never losing any Timer_IRQs_per_second
							// events but they are slow anyway

      clock_colon = 1 & (~clock_colon);			// update colon boolean
      digitalWrite( CLK_TICK_OUTPUT, clock_colon );
							// handle user sync request or
							// system reconfig via push button
      if (read_pb_input()==ButtonActive) {
        clear_lcd_row(3);
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

        while (read_pb_input()==ButtonActive) {			/* debounce wait with various options */

#if defined( support_7SEGMENT_DISPLAY ) || defined( support_DotMatrix_DISPLAY )

	  if (TIMER_IN_SECS_RANGE(20,30)) {			/* -BUTTONDOWN-20-to-lt-30- */
	    if (tenth_sec_count == 20*10) { clear_lcd_row(3); allPrintln("     Mode flip 12/24"); }
	    if (TIMER_ODD) display_LED_msg_cnt((char*) "2412",tenth_sec_count);
	    else           display_LED_msg_cnt((char*) "FLIP",tenth_sec_count);
	  }

	  else if (TIMER_IN_SECS_RANGE(10,20)) {		/* -BUTTONDOWN-10-to-lt-20- */
	    if (tenth_sec_count == 10*10) { clear_lcd_row(3); allPrintln("     Release for OTA"); }
	    display_LED_msg_cnt((char*) "OTA:",tenth_sec_count);
	  }

	  else {						/* -BUTTONDOWN-less-than-10- */
	    if (tenth_sec_count ==     2) { clear_lcd_row(3); allPrintln("     Release to sync"); }
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

	if ( tenth_sec_count <= 10*10) {	/* -BUTTONDOWN-less-than-10- */
	  set_clock_part1();			// (trigger clock resync phases then return)
	  return;
	}
	if ( tenth_sec_count <= 20*10) {	/* -BUTTONDOWN-10-to-lt-20- */
	  enable_OTA_update_from_main_server();	// (update from the main server (this call never returns))
	}
	if ( tenth_sec_count <= 30*10) {	/* -BUTTONDOWN-20-to-lt-30- */
	  HOURS_mode_is = (HOURS_mode_is==using_24HOUR_display)?using_12HOUR_display:using_24HOUR_display;
	  set_clock_part1();			// (trigger clock resync phases then return, also means display restarts in new mode)
	  return;
	}
      }						// end of sync request or system reconfig/updates

      delay(0);                                     // system call to support its real-time activities

      int flag_change_in_4digits = dt_1second_update();	// add 1 second to time note if HRS/MINS changed

      update_time_strings( clock_colon, dt_hours, dt_mins, dt_secs);
      delay(0);                                     // system call to support its real-time activities
      lcd.setCursor(0,0);                           // move to start of 1st row and overwrite
      delay(0);                                     // system call to support its real-time activities

      // LCD uses 24 hour mode always
      display_time( time_str );

      // LED options support 12 hour or 24 hour mode

#if defined( support_DotMatrix_DISPLAY )
      if (led_display_is == using_dots_led) {

	if ( LEDctl.current_value>=0 )
	  mx.control(MD_MAX72XX::INTENSITY, ((MAX_INTENSITY/2) * LEDctl.current_value)/max7segBrightness );

	char tmp = time_s12[0];						// (ensure any ms bit coding in time_s12[0] is ignored)
	time_s12[0] = PM_REMOVE_12HFLAG( time_s12[0] );
	if ((HOURS_mode_is == using_12HOUR_display) && (dt_hours>12))
	  load_md_message( '"', ' ', time_s12, 5 );			// display includes PM flag '"' with modified hours
	else
	  load_md_message( ' ', ' ', time_s12, 5 );			// display is normal 24 hour mode
	time_s12[0] = tmp;						// (restore time_s12[0])

      }
#endif
#if defined( support_7SEGMENT_DISPLAY )
      if (led_display_is == using_7seg_led) {
	display_7seg_4digits( time_s12, clock_colon, flag_change_in_4digits );	// display 12/24 hour data
      }
#endif

      if ((dt_hours == TimeSync_hours) &&
	  (dt_mins  == TimeSync_mins ) &&
	  (dt_secs  == TimeSync_secs )) {
	set_clock_part1();
      }

      delay(0);                                     // system call to support its real-time activities
    }

//
// handle sensor reads at specified intervals in a cycle of length sensorRollover
//
  
// Optional combined temperature and humiditiy sensor
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

  // Ensure a pass over process_network_services and process_user_enquiry
  //
  process_network_services();
  process_user_enquiry();
}

///////////////////////////////////////////////////////////////////////////
