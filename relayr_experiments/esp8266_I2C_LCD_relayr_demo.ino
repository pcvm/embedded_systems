/* -*- mode: c++; -*-
 *
 * ESP8266 DEMONSTRATION of MQTT reporting of sensor data to relayr, using local I2C and LCD
 *                          and the pubsub library http://pubsubclient.knolleary.net/api.html
 *
 *   p.musumeci@ieee.org Apr-2016
 *
 * NOTE: Section 3.1.1 "General Purpose Input/Output Interface (GPIO)",
 *       p.17 of document 0A-ESP8266__Datasheet__EN_v4.4.pdf states
 *           All digital IO pins are protected from over-voltage with a
 *           snap-back circuit connected between the pad and ground. The
 *           snap back voltage is typically about 6V, and the holding
 *           voltage is 5.8V. This provides protection from over-voltages
 *           and ESD. The output devices are also protected from reversed
 *           voltages with diodes.
 *       That is, the ESP8266 gpio pins appear to be 5V tolerant so in this work, the
 *       peripheral devices such as the PCF8574T (LCD backpack) and the MAX7219 (LED
 *       dot matrix driver) are powered by 5V with I2C and SPI lines directly connected
 *       to the ESP8266 which is powered by 3.3V. In general, the input/output lines are
 *       connected to 3.3V signals.
 *
 * THE HARDWARE USED HERE
 *
 * - An FTDI-232RL based usb-uart adapter is used in 3.3V mode, and connected as described in
 *   Figure "ESP to Serial" at https://github.com/esp8266/Arduino/blob/master/doc/boards.md
 *
 * - The ESP-12F module is on a WHITE daughterboard (0.1" pinouts) with the 3.3V regulator added
 *   on the underside and (very important) the middle bridging 0-Ohm resistor removed on top.
 *   See http://www.electrodragon.com/product/esp8266-smd-adapter-board-wi07-12/
 *
 * - The WHITE daughterboard has R1 and R3 of Figure "ESP to Serial" already installed, so pay
 *   careful attention to adding R2 and R4 as noted below:
 *
 *                 ESP-12 series connections (top view)
 *
 *    FTDI-RTS & add R4=10K    ___________
 *    pullup to 3.3V....RESET o           o GPIO1/RX....FTDI-TX   (top right)
 *                        adc o           o GPIO3/TX....FTDI-RX
 *    (R1 pullup on LHS) CHPD o           o gpio5/I2C_scl
 *                     GPIO16 o           o gpio4/I2C_sda
 *             gpio14/SPI_sck o           o GPIO0.......FTDI-DTR & add R2=10K pullup to 3.3V
 *            gpio12/SPI_miso o           o gpio2.......DHT11 temperature&humidity sensor or One-Wire
 *            gpio13/SPI_mosi o           o gpio15/SPI_ss (R3 pulldown on RHS)
 *    +ve power supply....VCC o           o GND.........FTDI GND & power supply GND
 *                             -----------
 *
 *                           [R1] [0Ohm] [R3]
 *                                  ^--- remove this 0Ohm bridge if using the 3.3V reg
 *
 * - The PCF8574T I2C io expander can be soldered directly on to the LCD module.
 *
 * THE SOFTWARE USED HERE
 *
 * - To simplify use of the display on the I2C "LCD backpack", we take files from
 *   NewliquidCrystal_1.3.4.zip (https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads)
 *   to _replace_ the default library LiquidCrystal (on OS-X, check for a location such as
 *   /Applications/Arduino.app/Contents/Java/libraries/LiquidCrystal into which the files of
 *   NewliquidCrystal are copied).
 *
 *   NOTE: the esp8266 compiler chain fails on SR (shift register) and Software protocol variants
 *         of NewliquidCrystal so just remove or rename files matching {*_SR*, SI2C*, *_SI2C*}.
 *         In sh:  for x in *_SR* SI2C* *_SI2C* ; do echo $x ; mv ${x} ${x}_disabled ; done
 *
 * - The ESP8266 environment provides extern C functions to specify a timer interrupt function
 *   to call back when a software based timeout occurs. This uses an os_timer_t data structure
 *   that is specified via os_timer_setfn() which also specifies the user call back. The timer
 *   interrupt interval is set by a call to os_timer_arm() which in this example sets the timer
 *   mode to continuous operation.
 *
 * - Pin allocations are documented at
 *       http://www.esp8266.com/wiki/doku.php?id=esp8266_gpio_pin_allocations
 *     Booting up in a UART-enabled mode (the default) means GPIO1 provides
 *     uart-TX and GPIO3 provides uart-RX.
 *
 * REFERENCES
 *
 *  1 What works, what doesn't -- https://github.com/esp8266/Arduino/blob/master/doc/reference.md
 *  2 Using newLiquidCrystal from https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home and see also
 *    http://tronixstuff.com/2014/09/24/tutorial-serial-pcf8574-backpacks-hd44780-compatible-lcd-modules-arduino/
 *  3 http://www.switchdoc.com/2015/10/iot-esp8266-timer-tutorial-arduino-ide/
 *  4 Ntp access was inspired by https://github.com/Nurgak/Electricity-usage-monitor (example) but then I read
 *    the NTPClient-Arduino example.
 *  5 Initial access point code (for input of wifi parameters) taken from  WiFiAccessPoint-Arduino code example,
 *    https://learn.sparkfun.com/tutorials/esp8266-thing-hookup-guide/example-sketch-ap-web-server, and
 *    http://www.esp8266.com/viewtopic.php?f=29&t=2153.
 *  6 Persistent wifi config and timezone data added to eeprom (see various Arduino examples).
 *  7 The DHT temperature sensor support comes from Lady Ada @adafruit (install the "non-unified" DHT library).
 *  8 The one-wire DS1820 temperature sensor support comes from the example code at
 *      https://milesburton.com/Dallas_Temperature_Control_Library
 *  9 MQTT pubsub library http://pubsubclient.knolleary.net/api.html
 */

// TODO
//

#define DEBUG_serial 1

///////////////////////////////////////////////////////////////////////////

//
// Application configuration (and includes)
//

// Select addon sensor handling
//
// - temperature/humidity reading and reporting with DHT11
//#define useDHTsensor 1
// - temperature reading and reporting with DS1820
#define useDS1820sensor 1

// Check conditionals to see if a temperature sensor is present
#if defined( useDHTsensor )
#elif defined( useDS1820sensor )
#else
#define no_temperature_display 1
#endif

///////////////////////////////////////////////////////////////////////////

// define IO setup
//
// LCD geometry:
#define lcd_ROWS   4
#define lcd_COLS  20
//
// GPIO:
// - default TX/RX pins
#define uartTXpin  1
#define uartRXpin  3
// - user reset buttin
#define theButton 16    // gpio16 has a 4k7 pull down resistor and a
                        // switch connecting it to 3.3V (press for HIGH)
// - SPI pin assignments
#define SPI_miso  12
#define SPI_mosi  13
#define SPI_sck   14    // shift clock
#define SPI_ss    15    // device select
//- I2C pin assignments
#define I2C_scl    5
#define I2C_sda    4
// - temperature sensor IO pin
#define DHTPIN     2    // DHT module uses bidirectional serial protocol.
#define DSPIN      2    // Dallas Semi 1-wire protocol handles temperature sensor.
                        // Alternative IO pins: GPIO0 and if SPI input can be
                        // disabled, GPIO12 might also be an option

///////////////////////////////////////////////////////////////////////////

// define default time zone and daily synchronisation time
#define DefaultTimeZone (+10)
// (time sync each day at TimeSync_hours:TimeSync_mins:TimeSync_secs)
#define TimeSync_hours (2)
#define TimeSync_mins  (3)
#define TimeSync_secs  (4)

#define lcd_setCursorSyncFlag lcd.setCursor(lcd_COLS-2,1)

// Choose 1/4 second resolution for maintaining daytime
#define Timer_IRQ_Interval_ms 250
#define Timer_IRQs_per_second   4
#define ntp_TIMEOUT (8*Timer_IRQs_per_second)  // retry every 8s

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
//
// Instantiate LCD at I2C backpack address 0x27
// - LCD operates in 4-bit data mode
// - I2C address = PCF8574T base address 0x20 + least significant 3 bits set by user
//
#define BACKLIGHT_PIN  3        // PCF8574 connections to the LCD
#define En_pin         2        //
#define Rw_pin         1        //
#define Rs_pin         0        //
#define D4_pin         4        // b4
#define D5_pin         5        // b5
#define D6_pin         6        // b6
#define D7_pin         7        // b7
//
LiquidCrystal_I2C  lcd(0x27,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);

#include <EEPROM.h>

// mqtt_esp8266 tools from https://github.com/knolleary/pubsubclient/releases/tag/v2.6
#include <PubSubClient.h>

// end of config

///////////////////////////////////////////////////////////////////////////
//

// Cut&paste of setup from relayr Arduino code fragment:
#define DEVICE_ID "__enter id here__"
#define MQTT_USER "__enter id here__"
#define MQTT_PASSWORD "__enter password here__"
#define MQTT_CLIENTID "__enter client id here__" //can be anything else
#define MQTT_TOPIC "__enter topic here__/"
#define MQTT_SERVER "mqtt.relayr.io"

int mqtt_publish_is_OK;	// flag when a publish is OK

struct mqtt_credentials {
  const char *clientId = MQTT_CLIENTID;
  const char *user     = MQTT_USER;
  const char *password = MQTT_PASSWORD;
  const char *topic    = MQTT_TOPIC "data";		// Note: "data" appended to MQTT_TOPIC
  const char *server   = MQTT_SERVER;
  const int  port      = 1883;
  } creds;

void message_callback(char* topic, byte* payload, unsigned int length)
{
  static char msg[lcd_COLS+1];
  unsigned int x = length;

  // handle message arrived
  clear_lcd_row( 2 );           // show first lcd_COLS chars of topic and msg
  strncpy( msg, topic, lcd_COLS-1 );
  for (int i=0; i<lcd_COLS; i++) msg[i] = 0x7f & msg[i];
  lcd.print( msg );

  clear_lcd_row( 3 );
  if (x>lcd_COLS) x = lcd_COLS;
  strncpy( msg, (char*) payload, x-1 );
  for (int i=0; i<lcd_COLS; i++) msg[i] = 0x7f & msg[i];
  lcd.print( msg );
}

WiFiClient espClient;
PubSubClient client(espClient);

void setup_mqtt_system()
{
  mqtt_publish_is_OK = 0;

  client.setServer(creds.server, creds.port);
  client.setCallback(message_callback);
  reconnect();
}

// Reconnect just once if link down
//
void reconnect()
{
  if (!client.connected()) {
#if defined( DEBUG_serial )
    Serial.println("Attempting MQTT connection...");
#endif
    // Attempt to connect
    if (client.connect(creds.clientId, creds.user, creds.password)) {
#if defined( DEBUG_serial )
      Serial.println("connected");
#endif
      //client.publish("outTopic", "hello world");	// pubs
      //client.subscribe("inTopic");			// subs
    } else {
#if defined( DEBUG_serial )
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println();
#endif
    }
  }
}

///////////////////////////////////////////////////////////////////////////
//
// Variable sensorRead_tick is incremented in the timer event handler code
// so that it can then be used as a trigger for timed events such as sensor
// reading. It is reset to 0 when it counts to sensorRollover.

const int sensorRollover             = 10*Timer_IRQs_per_second;  // sensors are read every 10s
int sensorRead_tick;

const int instant_to_readHumidity    =  3*Timer_IRQs_per_second;  // read H at 3s point
const int instant_to_readTemperature =  6*Timer_IRQs_per_second;  // read T at 6s point

const int instant_to_readDStemp      =  5*Timer_IRQs_per_second;  // read T at 5s point

// State information for sensor data acquisition
// - generally these vars are set whenever a transaction is triggered in order
//   to ensure a single trigger per sensorRollover period
// - memory use small so no vars are conditional code, cleared at end of sensorRollover period
byte ds_temp_read_inProgress, dht_humidity_read_inProgress, dht_temp_read_inProgress;
  
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  
#if defined( useDS1820sensor )

// OneWire DS18S20, DS18B20, DS1822 Temperature Example
#include <OneWire.h>
#include <DallasTemperature.h>

OneWire  ds(DSPIN);
DallasTemperature DS18B20(&ds);

#define nCharsTemp 10
char latest_ds_temperature[nCharsTemp]; 
char* get_temperature() { return latest_ds_temperature; }

// Notes:
//   - assumes only one sensor in operation (Index==0)
//   - applies a hard -ve limit so result is >-100
void update_ds_temperature()
{
  DS18B20.requestTemperatures();
  delay(0);
  float temp = DS18B20.getTempCByIndex(0);
  delay(0);

  if (temp <= -100) temp = -99.99;

  String x = String( temp );
  strncpy( latest_ds_temperature, x.c_str(), nCharsTemp-1 );
}

void ow_setup()
{
  ds_temp_read_inProgress = 0;
  for (int i=0; i<nCharsTemp; i++)
    latest_ds_temperature[i] = 0;

  clear_lcd_row( 3 );           // clear screen and storage
  lcd.print("Current Temp=");
  DS18B20.begin();
  update_ds_temperature();
  lcd.print( get_temperature() );
}
#endif

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#if defined( useDHTsensor )

// Temperature reading code is derived from the example testing sketch for various DHT
// humidity/temperature sensors written by ladyada and placed in the public domain.

#include "DHT.h"

// select sensor type
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connections:
//   pin 1 (on the left) of the sensor to +3.3V (esp8266 uses 3.3V)
//   pin 2 of the sensor to whatever your DHTPIN is
//   pin 3 (on the right) of the sensor to GROUND
// Adafruit recommend a 10K pullup on the data line but the current board already has a 4k7 pullup.

// Instantiate DHT sensor
// - initialise with dht.begin()
DHT dht(DHTPIN, DHTTYPE);

#define nCharsResult 10
char latest_temperature[nCharsResult];      // char[] to hold most recent results
char latest_humidity[nCharsResult];
   
void dht_setup() 
{
  dht_humidity_read_inProgress = dht_temp_read_inProgress = 0;

  for (int i=0; i<nCharsResult; i++)
  {
    latest_temperature[i]=latest_humidity[i]=0;
  }
  dht.begin();
}
//
// Reading the temperature or humidity takes about 250 milliseconds so the update
// function calls are scheduled in the main loop. Calling the get_Value() functions
// return the address of the latest values.
// - Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
//
//
char* get_humidity()    { return latest_humidity; }
char* get_temperature() { return latest_temperature; }
//
void update_humidity()
{
  delay(0);
  float h=dht.readHumidity();
  delay(0);

  String x = String( h );
  strncpy( latest_humidity, x.c_str(), nCharsResult-1 );
}
void update_temperature()
{
  delay(0);
  float t=dht.readTemperature();
  delay(0);

  String x = String( t );
  strncpy( latest_temperature, x.c_str(), nCharsResult-1 );
}

#endif

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
//
// Define lcd convenience functions
//
void clear_lcd_row( int r )    // overwrites text with ' ' chars
{
  lcd.setCursor(0,r);
  for ( int i=0; i<lcd_COLS; i++ ) lcd.print(' ');
  lcd.setCursor(0,r);
}
//
const int ts_pads = (lcd_COLS-8)/2;
//
void display_time( char* time_str ) // display time string
{
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
  lcd.setCursor(0,2);                           // move to start of 3nd line and overwrite
  lcd.print("          H=");
  lcd.print( get_humidity() );
#endif
}

///////////////////////////////////////////////////////////////////////////
//
// The system keeps config data in eeprom. When retrieved, it is checked
// and validity determined and stored in variable configuration_is_OK.
// This var is used to control the mode of setup() and loop().
//
bool configuration_is_OK;     // set to true when valid config data retrieved

// The eeprom storage used is EEPROM_nBytesUsed (512) bytes.
// It is allocated as {header ssid_Cstr pass_Cstr misc_bytes}
#define EEPROM_nBytesUsed 512
#define WIFI_parms_size 64    // assume this is large enough for ssid and pwd
#define MISC_parms_size  8    // misc. signed bytes e.g. time zone offset
#define n_ConfigPattern  8    // use pairs of complementary bytes as pattern (pairs sum to 0xff)
const byte header[] = {0xa5,0x5a,0x69,0x96,0x55,0xaa,0xff,0};

char ssid_Cstr[WIFI_parms_size];   // network SSID (name)
char pass_Cstr[WIFI_parms_size];   // network password
signed char misc_bytes[MISC_parms_size];
                                   // misc_bytes[0] == the_time_zone
                                   // misc_bytes[1] == ...

int the_time_zone;            // current time offset with respect to UTC/GMT

bool retrieve_config_OK()
{
  configuration_is_OK = false;

  EEPROM.begin(EEPROM_nBytesUsed);
  delay(10);

  // check integrity (simple byte match)
  int ptr=0;
  for ( int x=0; x<n_ConfigPattern; x=x+2 )
    if ( 0xff != (EEPROM.read(ptr++) + EEPROM.read(ptr++) ) ) return false; // 2 of n_ConfigPattern bytes

  // retrieve strings
  for (int i = 0; i < WIFI_parms_size; ++i)
      ssid_Cstr[i] = char(EEPROM.read(ptr++));
  for (int i = 0; i < WIFI_parms_size; ++i)
      pass_Cstr[i] = char(EEPROM.read(ptr++));

  // retrieve ints
  for (int i = 0; i < MISC_parms_size; ++i)
      misc_bytes[i] = char(EEPROM.read(ptr++));
  the_time_zone = misc_bytes[0];

  clear_lcd_row(0);
  lcd.print("Retrieving config");
  EEPROM.end();
  clear_lcd_row(1); lcd.print(String('(') + ssid_Cstr + ')');
  clear_lcd_row(2); lcd.print(String('(') + pass_Cstr + ')');
  clear_lcd_row(3); lcd.print("(UTC/GMT");
  if (the_time_zone>0)  lcd.print('+');
  if (the_time_zone!=0) lcd.print(the_time_zone);
  lcd.print(')');
  delay(5000);
  configuration_is_OK = true;
  return true;
}

void save_config(char* s, char* p, signed char ttz)
{
  EEPROM.begin(EEPROM_nBytesUsed);
  delay(10);

  //
  // Clear EEPROM region
  //
  clear_lcd_row(0); lcd.print("Clearing memory...");
  for (int i = 0; i < EEPROM_nBytesUsed; ++i)
  {
    EEPROM.write( i, 0 );
    delay(1);
  }

  //
  // Write "magic" patterns to indicate valid data follows
  //
  clear_lcd_row(1); lcd.print("Writing header...");
  int ptr=0;
  for (int x; x<n_ConfigPattern; x++ )
  {
    EEPROM.write(ptr++,header[x]);
    delay(1);
  }

  //
  // Clear and load buffers for string and int data to write
  //
  for (int i = 0; i < WIFI_parms_size; ++i) ssid_Cstr[i]=pass_Cstr[i]=' ';
  strncpy(ssid_Cstr, s, WIFI_parms_size);       // copy network info to fixed length buffers
  strncpy(pass_Cstr, p, WIFI_parms_size);
  for (int i = 0; i < MISC_parms_size; ++i) misc_bytes[i]=0;
  misc_bytes[0] = ttz;                          // copy time zone ttz etc. to fixed length buffer

  //
  // Write...
  //
  clear_lcd_row(2); lcd.print("Writing to eeprom ");
  for (int i = 0; i < WIFI_parms_size; ++i)
  {
    EEPROM.write(ptr++, ssid_Cstr[i]);
    delay(1);
  }
  lcd.print('1');
  for (int i = 0; i < WIFI_parms_size; ++i)
  {
    EEPROM.write(ptr++, pass_Cstr[i]);
    delay(1);
  }
  lcd.print('2');
  for (int i = 0; i < MISC_parms_size; ++i)
  {
    EEPROM.write(ptr++, misc_bytes[i]);
    delay(1);
  }
  lcd.print('3');

  //
  // Commit changes
  //
  EEPROM.end();
  lcd.print('4');
  
  clear_lcd_row(0); lcd.print("Configuration info:");
  clear_lcd_row(1); lcd.print(' '); lcd.print(ssid_Cstr);
  clear_lcd_row(2); lcd.print(' '); lcd.print(pass_Cstr);
  clear_lcd_row(3); lcd.print(' '); lcd.print(int(ttz));
  delay(1000);
}

///////////////////////////////////////////////////////////////////////////
//
// Timer interrupts
//

extern "C" {
#include "user_interface.h"
}

os_timer_t myTimer_variable;

int clock_tick;       // increment in IRQ_handler()
                      // tested/read/cleared in main()
#define FSM_OFF (-1)
int fsm_tick=FSM_OFF; // increment in IRQ_handler()
                      // used by state machines that need timing
                      // set to FSM_OFF when inactive
                  
int clock_colon;                // flashing colon control
int dt_secs, dt_mins, dt_hours; // daytime keeping vars
//
// Variables for time display
#define time_slen 12            // handles dd:dd:dd0
char time_str[time_slen];       // new computed value
char lcd_data[time_slen];       // value currently on LCD

// define timerCallback (minimal length)
void timerCallback(void *pArg)
{ 
  clock_tick++;
  sensorRead_tick++;
  if ( fsm_tick >= 0) fsm_tick++;
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

  // set up 1s time keeping interrupts by calling
  // void os_timer_arm() with arguments
  //     os_timer_t *pTimer          ... time keeping data structure
  //     uint32_t milliseconds       ... millisecs/interrupt
  //     bool repeat                 ... continuous if true

  os_timer_arm( &myTimer_variable, Timer_IRQ_Interval_ms, true);
}

///////////////////////////////////////////////////////////////////////////

//
// WiFi for accessing local network
//

// Connection for time synchronisation
unsigned int localPort = 2390;      // Local port to listen for UDP packets
IPAddress timeServerIP;             // time.nist.gov NTP server address
const char* ntpServerName = "time.nist.gov";
const int NTP_PACKET_SIZE = 48;     // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE]; // Buffer to hold incoming and outgoing packets

WiFiUDP udp;                        // udp structure used in udp exchanges

// - display current IP and ntp port numbers
void show_netstat()
{
  clear_lcd_row(2); 
  lcd.print("IP: ");
  lcd.print(WiFi.localIP());

  clear_lcd_row(3); 
  lcd.print("UDP local port: ");
  lcd.print(udp.localPort());
}

// - connect to network and open a udp port
void setup_networking_WIFIclient()
{
  clear_lcd_row(2);
  lcd.print( "Connecting: " );
  lcd.print( ssid_Cstr );
  clear_lcd_row(3);
  WiFi.begin(ssid_Cstr, pass_Cstr);

  int tries=0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    lcd.print(".");
    if ( ++tries >= lcd_COLS ){ tries=0; clear_lcd_row(3); }
  }
  clear_lcd_row(3);
  lcd.print("IP: ");
  lcd.print(WiFi.localIP());
  delay(1000);              // ESP-12e seems slower than the ESP-12F part (-pm-)

  clear_lcd_row(2);
  lcd.print("Starting UDP");
  udp.begin(localPort);     // [can dismantle with udp.stop()]
  delay(0);                 // system call to support its real-time activities
  show_netstat();
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address)
{
  clear_lcd_row(3);
  lcd.print("Sending NTP packet..");
                            // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
                            // Initialize values needed to form NTP request
                            // (see URL above for details on the packets)
  packetBuffer[ 0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[ 1] = 0;     // Stratum, or type of clock
  packetBuffer[ 2] = 6;     // Polling Interval
  packetBuffer[ 3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
                            // all NTP fields have been initialised, send request
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

///////////////////////////////////////////////////////////////////////////

//
// WiFi for temporary access point for configuration
// - this code had a starting point of example "WiFiAccessPoint" (Feb2016)
//

#include <ESP8266WebServer.h>

// define WiFi access point ssid for configuration
// - currently ignoring a password because this mode of operation only occurs
//   when a user reboots with the config button pressed (or on initial power up)
const char *config_ssid = "ClockThing-";

ESP8266WebServer server(80);

// Two standard resonses are defined
// - provide the setup form
void handleRoot()
{
  server.send(200, "text/html", "<h1>\
<p>Welcome to configuration <br> of ClockThing! <p>\
<p><form action='update' method='get'>\
SSID: <input type='text' name='s'> <p>\
PASSWORD: <input type='text' name='p'> <p>\
UTC/GMT offset: <input type='text' name='o'> <p>\
<input type='submit' value='Submit'>\
</form> </h1>");
}
// - decode the completed form
void handleUpdate() 
{
  String ssid=server.arg("s");
  String pwd =server.arg("p");
  String utco=server.arg("o");
  
  char* ssid_ptr = (char*) ssid.c_str();
  char* pwd_ptr  = (char*) pwd.c_str();
  signed char temp_byte = atoi( utco.c_str() );
  save_config( ssid_ptr, pwd_ptr, temp_byte );

  clear_lcd_row(3); lcd.print("Restart");
  server.send(200, "text/html", "<p> <h1>Thank you!</h1>");
  delay(5000);
  clear_lcd_row(3); lcd.print("Please cycle power");                 // should never reach here
}

void reconfigure_system_via_local_WIFIap()
{
  delay(1000);

  // create tmp_ssid by appending 4 hex chars from MAC to config_ssid
  char tmp_ssid[ strlen( config_ssid ) + 4 ];
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);

  sprintf( tmp_ssid,"%s%02x%02x", config_ssid, toupper(mac[WL_MAC_ADDR_LENGTH - 2]), toupper(mac[WL_MAC_ADDR_LENGTH - 1]) );
  WiFi.softAP(tmp_ssid);
  
  lcd.setCursor(0,0);
  lcd.print("To configure, visit");
  lcd.setCursor(0,1);
  lcd.print("http://");
  IPAddress myIP = WiFi.softAPIP();
  lcd.print(myIP);
  lcd.setCursor(0,2);
  lcd.print("on wifi network");
  lcd.setCursor(0,3);
  lcd.print("  "); lcd.print(tmp_ssid);

  server.on("/", handleRoot);
  server.on("/update", handleUpdate);
  server.begin();
}

void WIFIap_loop()
{
  server.handleClient();
}

///////////////////////////////////////////////////////////////////////////

//
// Clock synchronisation via udp access to ntp server
//

void set_clock_part1()
{
  fsm_tick=0;
                                //get a random server from the pool
  WiFi.hostByName(ntpServerName, timeServerIP); 
  delay(0);                     // system call to support its real-time activities
  sendNTPpacket(timeServerIP);  // send an NTP packet to a time server
  delay(0);                     // system call to support its real-time activities

  lcd_setCursorSyncFlag;        // mark sync as in process
  lcd.print('*');
}

void set_clock_part2()
{
  int cb = udp.parsePacket();
  if (!cb) 
  {
    clear_lcd_row(3);
    if (fsm_tick > ntp_TIMEOUT)
    {
      lcd.print("NTP timeout, retry");
      delay(2000);
      set_clock_part1();
    }
    else
    {
      lcd.print("No packet yet");
      delay(250);
    }
  }
  else 
  {
    fsm_tick=FSM_OFF;
    
    clear_lcd_row(3);
    lcd.print("Rx pkt len=");
    lcd.println(cb);
                                            // received a packet so decode
    udp.read(packetBuffer, NTP_PACKET_SIZE);// read packet into the buffer
    delay(0);                               // system call to support its real-time activities

    // Note: the timestamp, as seconds since Jan 1 1900, starts at byte 40 of
    //       the received packet and is encoded as a big-endian 32b word
    //
    unsigned long secsSince1900 = 0;
    for (int i=0; i<4; i++ )          // merge in 4 bytes, assuming most significant come first
      secsSince1900 = (secsSince1900 << 8) | (unsigned long) packetBuffer[40+i];
                                                        // process the time data assuming Unix form, which
    const unsigned long seventyYears = 2208988800UL;    // is in seconds since Jan 1 1970 i.e. 70 year offset
                                                        // subtract seventy years to obtain
    unsigned long epoch = secsSince1900 - seventyYears; // epoch in Unix form of UTC aka GMT

    int tmp_ms=dt_mins*60+dt_secs;        // save old time in range of an hour for correction measurement

    dt_hours = (epoch  % 86400L) / 3600;  // convert epoch to hr,min,secs and adjust for time zone
    dt_mins  = (epoch  %   3600) / 60;
    dt_secs  = epoch % 60;
    dt_hours = dt_hours + the_time_zone;
    if ( dt_hours >= 24 ) dt_hours = dt_hours - 24;

    clear_lcd_row(3);
    lcd.print("Sync@ ");
    sprintf( time_str, "%2d:%02d:%02d", dt_hours, dt_mins, dt_secs );
    lcd.print( time_str );

    tmp_ms=(dt_mins*60+dt_secs) - tmp_ms; // estimate(!) correction measurement (assume error is a few seconds)
    if ( tmp_ms > 3600 ) tmp_ms = tmp_ms-3600;
    if ( tmp_ms >=0 ) lcd.print(" +");
    else              lcd.print(' ');
    lcd.print(tmp_ms); 

    lcd_setCursorSyncFlag;               // mark sync as complete
    lcd.print(' ');

    clear_lcd_row(2);                    // clear IP display

    // all timed initialisation is now complete
    //
    // -pm- initialise mqtt for live operation
    setup_mqtt_system();
  }
}

///////////////////////////////////////////////////////////////////////////

// - Arduino initialisation

const char* system_id[] PROGMEM = {"Phillip's relayr",
				   "     Temperature",
				   "         IoThing"};
const char* NL10 = "\n\n\n\n\n\n\n\n\n\n";

void setup()
{
  the_time_zone = DefaultTimeZone;

#if defined( DEBUG_serial )
  Serial.begin(115200);		// serial uart for debug messages
  delay(250);
  Serial.println( NL10 );
#endif
  
                                // initialise I2C
  Wire.begin( I2C_sda, I2C_scl );
  // Can also set transfer speed e.g. Wire.setClock( 100000 )

                                // activate LCD module for chosen format and
  lcd.begin(lcd_COLS,lcd_ROWS);
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);       // activate backlight
  for (int i=0;i<3;i++)
    {
      clear_lcd_row(i);
      lcd.print( system_id[i] );
#if defined( DEBUG_serial )
      Serial.println( system_id[i] );
#endif
    }

  clock_tick=clock_colon=0;
  dt_secs=dt_mins=dt_hours=0;   // clear daytime keeping vars
  sprintf( time_str, "%2d:%02d:%02d", dt_hours, dt_mins, dt_secs );

  sensorRead_tick = -1; // let this counter start at the -1/Timer_IRQs_per_second point
                        // so events do not match whole seconds
  setup_timer_interrupt_system();

                                // initialise time display
  lcd.home();                   // set cursor to 0,0

#if defined( useDHTsensor )
  dht_setup();                  // initialise DHT sensor
#endif
#if defined( useDS1820sensor )
  ow_setup();                   // initialise 1-wire and one DS temp sensor
#endif
  delay(5000);

  pinMode( theButton, INPUT );  // configure user input button
  delay(250);
  //
  // Handle re-entry of WiFi config and timezone config values when either the
  // user reset input (theButton) is set or the EEPROM config data is not available
  if ((digitalRead(theButton) == HIGH) || !retrieve_config_OK())
    reconfigure_system_via_local_WIFIap();
  else
    {
      clear_lcd_row(0);
      lcd.print("Lake Placid: UTC");
      if (the_time_zone>0)  lcd.print('+');
      if (the_time_zone!=0) lcd.print(the_time_zone);
      clear_lcd_row(1);         // move to start of 2nd line
      display_time( time_str );
      setup_networking_WIFIclient();
      set_clock_part1();        // start clock resync phases
    }
}

//
// main user function
//
void loop()
{
  if (!configuration_is_OK)
  {
    WIFIap_loop();
    return;
  }

  if ( fsm_tick != FSM_OFF )
    set_clock_part2();

  if ( clock_tick>=Timer_IRQs_per_second )
    {
      clock_tick=clock_tick-Timer_IRQs_per_second;  // could make this uninterruptable to be really
                                                    // sure of never losing any Timer_IRQs_per_second
                                                    // events but they are slow anyway

      clock_colon = 1 & (++clock_colon);            // update colon boolean

      // handle user input (yes, the clock has a user command input button...)
      if ( digitalRead(theButton) == HIGH )
        {
          clear_lcd_row(3);
          lcd.print("Resyncing...");
          while ( digitalRead(theButton) == HIGH )  // debounce wait
            delay(0);                               //               on button release
          set_clock_part1();                        // trigger clock resync phases
          return;                                   //               as last action of "loop"
        }

      delay(0);                                     // system call to support its real-time activities

      if (++dt_secs==60)                            // add 1 second to time
              {
                dt_secs=0;
                if (++dt_mins==60)
                  {
                    dt_mins=0;
                    if (++dt_hours==24)
                        dt_hours=0;
                  }
              }


      if (clock_colon) sprintf( time_str, "%2d:%02d:%02d", dt_hours, dt_mins, dt_secs );
      else             sprintf( time_str, "%2d %02d %02d", dt_hours, dt_mins, dt_secs );
      delay(0);                                     // system call to support its real-time activities
      lcd.setCursor(0,1);                           // move to start of 2nd line and overwrite
      delay(0);                                     // system call to support its real-time activities
      display_time( time_str );

      if ( (dt_hours==TimeSync_hours) &&
           (dt_mins==TimeSync_mins  ) &&
           (dt_secs==TimeSync_secs  ) )
           {
              set_clock_part1();
           }

      delay(0);                                     // system call to support its real-time activities
    }

//
// MQTT: handle any pending publish actions
//

  if (0 != mqtt_publish_is_OK)
    {
      String tmpS = String("{\"meaning\":\"Temperature (int)\", \"value\":\"");
      tmpS += get_temperature();
      tmpS += "\"}";

      reconnect();
      int pStatus=client.publish( creds.topic, tmpS.c_str() );
      mqtt_publish_is_OK=0;             // clear data to publish flag
					// (not retrying if transfer fails)
#if defined( DEBUG_serial )
      Serial.print("Tx: ");
      Serial.println(creds.topic);
      Serial.print("    ");
      Serial.print(tmpS.c_str());
      if (pStatus==0)  Serial.println(" -> failed");
      else             Serial.println(" -> OK");
#endif
    }

//
// handle sensor reads at specified intervals in a cycle of length sensorRollover
//

#if defined( useDHTsensor )
  if ( sensorRead_tick == instant_to_readHumidity && dht_humidity_read_inProgress == 0 )
  {
    dht_humidity_read_inProgress = 1;
    update_humidity();
  }
  if ( sensorRead_tick == instant_to_readTemperature && dht_temp_read_inProgress == 0 )
  {
    lcd.print(" +");                    // update sensor active indicator
    dht_temp_read_inProgress = 1;
    update_temperature();
  }
#endif

#if defined( useDS1820sensor )
  if ( sensorRead_tick == instant_to_readDStemp && ds_temp_read_inProgress == 0 )
  {
    lcd.print(" +");                    // update sensor active indicator
    ds_temp_read_inProgress = 1;
    update_ds_temperature();
  }
#endif

  if ( sensorRead_tick >= sensorRollover )
  {
    lcd.print("  ");                    // clear sensor active indicator
    sensorRead_tick = sensorRead_tick-sensorRollover;
    dht_humidity_read_inProgress = dht_temp_read_inProgress = ds_temp_read_inProgress = 0;

    // test -pm-
    mqtt_publish_is_OK++;               // indicate fresh data available (once/10secs)
  }

//
// MQTT: handle any pending publish actions
//

  reconnect();
  client.loop();
}

///////////////////////////////////////////////////////////////////////////
