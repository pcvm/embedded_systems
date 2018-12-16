/* -*- mode: c++; -*-
 *
 * ESP8266 DEMONSTRATION of I2C and LCD, 1-bit LED output, and Serial communications.
 *         Application is a time-of-day clock using timer interrupts with ntp
 *         client calls to synchronise time daily and WiFi access point configuration
 *         with previous config stored in EEPROM.
 *
 *   p.musumeci@ieee.org Jan-2016
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
 *   NOTE: the circuit at https://github.com/esp8266/Arduino/blob/master/doc/ESP_to_serial.png
 *         shows direct connection between the 3.3V supply from the USB-uart interface and the
 *         3.3V from the ESP8266 board - you should keep these supplies separate (unless your
 *         USB-uart power source really is strong enough to drive everything).
 *
 * - The ESP-12E module is on a WHITE daughterboard (0.1" pinouts) with the 3.3V regulator added
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
 *    theButton........GPIO16 o           o gpio4/I2C_sda
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
 * + An optional LED driven by GPIO16 can be selected in order to free up the line used for
 *   serial output. Feed GPIO16 via a 1K resistor to an LED and define LEDon16 below.
 *   (this line is used later for a user input switch)
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
 * 1 What works, what doesn't -- https://github.com/esp8266/Arduino/blob/master/doc/reference.md
 * 2 Using newLiquidCrystal from https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home and see also
 *   http://tronixstuff.com/2014/09/24/tutorial-serial-pcf8574-backpacks-hd44780-compatible-lcd-modules-arduino/
 * 3 http://www.switchdoc.com/2015/10/iot-esp8266-timer-tutorial-arduino-ide/
 * 4 Ntp access was inspired by https://github.com/Nurgak/Electricity-usage-monitor (example) but then I read
 *   the NTPClient-Arduino example.
 * 5 Initial access point code (for input of wifi parameters) taken from  WiFiAccessPoint-Arduino code example,
 *   https://learn.sparkfun.com/tutorials/esp8266-thing-hookup-guide/example-sketch-ap-web-server, and
 *   http://www.esp8266.com/viewtopic.php?f=29&t=2153.
 * 6 Persistent wifi config and timezone data added to eeprom (see various Arduino examples).
 *
 * + Possible display info https://code.google.com/archive/p/arudino-maxmatrix-library/
 */

// TODO
//
// 1 add a retry limit on the sync etc.

//
// Application configuration (and includes)
//

// define IO setup
#define lcd_ROWS 4
#define lcd_COLS 20
#define LEDon16 1

// define default time zone and daily synchronisation time
#define Tzone (+10)
// schedule ntp time sync each day at, say, 2:03:04 ...
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

// Choose LED to flash
#if defined(__AVR__)
#define LED 13  // Atmel/AVR
#else
#if defined( LEDon16 )
#define LED 16  // Use free gpio pin to drive an LED, which allows Serial to be used
#else
#define LED  2  // Default ESP8266 LED on the TX line (LED use interferes with Serial port use)
#endif
#endif

#include <EEPROM.h>

// end of config

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
void display_time( char* time_str ) // display time string
{
  for ( int x=0; x<ts_pads; x++ ) lcd.print(' ');
  lcd.print( time_str );
}

///////////////////////////////////////////////////////////////////////////
//
// The system keeps config data in eeprom. When retrieved, it is checked
// and validity determined and stored in variable configuration_is_OK.
// This var is used to control the mode of setup() and loop().
//
bool configuration_is_OK;     // set to true when config data valid

#define EEPROM_used 512
#define WIFI_parms_size 64    // assume this is large enough for ssid and pwd
#define n_ConfigPattern  8    // use pairs of complementary bytes as pattern (pairs sum to 0xff)
const byte header[] = {0xa5,0x5a,0x69,0x96,0x55,0xaa,0xff,0};

char ssid[WIFI_parms_size];   // network SSID (name)
char pass[WIFI_parms_size];   // network password

bool retrieve_config()
{
  EEPROM.begin(EEPROM_used);
  delay(10);

  // check integrity (simple byte match)
  int ptr=0;
  for ( int x=0; x<n_ConfigPattern; x=x+2 )
    if ( 0xff != (EEPROM.read(ptr++) + EEPROM.read(ptr++) ) ) return false; // 2 of n_ConfigPattern bytes

  for (int i = 0; i < WIFI_parms_size; ++i)
      ssid[i] = char(EEPROM.read(ptr++));
  for (int i = 0; i < WIFI_parms_size; ++i)
      pass[i] = char(EEPROM.read(ptr++));

  clear_lcd_row(0);
  lcd.print("Retrieving config");
  EEPROM.end();
  clear_lcd_row(1);
  lcd.print(String('(') + ssid + ')');
  clear_lcd_row(2);
  lcd.print(String('(') + pass + ')');
  delay(5000);
  return true;
}

void save_config(char* s, char* p)
{
  EEPROM.begin(EEPROM_used);
  delay(10);

  clear_lcd_row(0); lcd.print("Clearing memory...");
  for (int i = 0; i < EEPROM_used; ++i)
  {
    EEPROM.write( i, 0 );
    delay(1);
  }

  // write complementary patterns
  clear_lcd_row(1); lcd.print("Writing header...");
  int ptr=0;
  for (int x; x<n_ConfigPattern; x++ )
  {
    EEPROM.write(ptr++,header[x]);
    delay(1);
  }

  for (int i = 0; i < WIFI_parms_size; ++i) ssid[i]=pass[i]=' ';
  strncpy(ssid, s, WIFI_parms_size);        // copy network info to fixed length buffers
  strncpy(pass, p, WIFI_parms_size);

  clear_lcd_row(2); lcd.print("Writing to eeprom ");
  
  for (int i = 0; i < WIFI_parms_size; ++i)
  {
    EEPROM.write(ptr++, ssid[i]);
    delay(1);
  }
  lcd.print('1');
  
  for (int i = 0; i < WIFI_parms_size; ++i)
  {
    EEPROM.write(ptr++, pass[i]);
    delay(1);
  }
  lcd.print('2');

  EEPROM.end();
  lcd.print('3');
  
  clear_lcd_row(0); lcd.print("Configuration info:");
  clear_lcd_row(1); lcd.print(' '); lcd.print(ssid);
  clear_lcd_row(2); lcd.print(' '); lcd.print(pass);
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
                  
int clock_colon;                        // flashing colon control
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
  lcd.print( ssid );
  clear_lcd_row(3);
  WiFi.begin(ssid, pass);

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
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
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
<input type='submit' value='Submit'>\
</form> </h1>");
}
// - decode the completed form
void handleUpdate() 
{
  String ssid=server.arg("s");
  String pwd =server.arg("p");
  
  char* ssid_ptr = (char*) ssid.c_str();
  char* pwd_ptr  = (char*) pwd.c_str();
  save_config( ssid_ptr, pwd_ptr );
  
  clear_lcd_row(3); lcd.print("Restart");
  server.send(200, "text/html", "<p> <h1>Thank you!</h1>");
  delay(5000);
  clear_lcd_row(3); lcd.print("Please cycle power");                 // should never reach here
}

void setup_networking_WIFIap()
{
  delay(1000);

  // create tmp_ssid by appending 4 hex chars from MAC to config_ssid
  char tmp_ssid[ strlen( config_ssid ) + 4 ];
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);

  sprintf( tmp_ssid,"%s%02x%02x", config_ssid, toupper(mac[WL_MAC_ADDR_LENGTH - 2]), toupper(mac[WL_MAC_ADDR_LENGTH - 1]) );
  WiFi.softAP(tmp_ssid);
  
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
                                    
    clear_lcd_row(3);
    lcd.print("ES=" );
    lcd.print(secsSince1900);
                                      // process the time data assuming Unix form, which
                                      // is in seconds since Jan 1 1970 i.e. 70 years later
    const unsigned long seventyYears = 2208988800UL;
                                                        // subtract seventy years and get
    unsigned long epoch = secsSince1900 - seventyYears; // epoch in Unix form of UTC aka GMT

    clear_lcd_row(3);
    lcd.print("Last sync @ ");

    dt_hours = (epoch  % 86400L) / 3600;  // convert epoch to hr,min,secs
    dt_mins  = (epoch  %   3600) / 60;
    dt_secs  = epoch % 60;
    dt_hours = (dt_hours + Tzone) % 24;   // display local time

    sprintf( time_str, "%2d:%02d:%02d", dt_hours, dt_mins, dt_secs );
    lcd.print( time_str );

    lcd_setCursorSyncFlag;               // mark sync as complete
    lcd.print(' ');
  }
}

///////////////////////////////////////////////////////////////////////////

// - Arduino initialisation
void setup()
{
                                // initialise I2C
  Wire.begin( 4,5 );            // specify SDA==>GPIO4, SCLK==>GPIO5
  // Can also set speed e.g. Wire.setClock( 100000 )

                                // initialise serial comms and LED pin
#if defined( LEDon16 )
  Serial.begin(115200);
#endif
  pinMode(LED, OUTPUT);
                                // activate LCD module for chosen format and
  lcd.begin(lcd_COLS,lcd_ROWS);
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);       // activate backlight

  clock_tick=clock_colon=0;
  dt_secs=dt_mins=dt_hours=0;   // clear daytime keeping vars
  sprintf( time_str, "%2d:%02d:%02d", dt_hours, dt_mins, dt_secs );

  setup_timer_interrupt_system();

                                // initialise time display
  lcd.home();                   // set cursor to 0,0

  if ( (configuration_is_OK=retrieve_config()) )
  {
    clear_lcd_row(0);
    lcd.print("Lake Placid: UTC");
    if ( Tzone>0 ) lcd.print('+');
    lcd.print(Tzone);
    clear_lcd_row(1);           // move to start of 2nd line
    display_time( time_str );
    setup_networking_WIFIclient();
    set_clock_part1();
  } else {
    setup_networking_WIFIap();
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

      clock_colon = 1 & (++clock_colon);            // flash colon
      digitalWrite(LED, clock_colon );

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
      sprintf( time_str, "%2d:%02d:%02d", dt_hours, dt_mins, dt_secs );
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
}

///////////////////////////////////////////////////////////////////////////
