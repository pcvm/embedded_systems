/* -*- mode: c++; -*- */

// WiFi for temporary access point for configuration
// - this code had a starting point of example "WiFiAccessPoint" (Feb2016)
// - later, example code for BearSSL class was included

//#define enable_HTTPS_code 1

#if defined( enable_HTTPS_code )
BearSSL::ESP8266WebServerSecure wifi_server_main(443);	// our main server is on port 443
ESP8266WebServer wifi_server_other(80);
#else
ESP8266WebServer wifi_server_main(80);			// our main server is on port 80
#endif

#if defined( enable_HTTPS_code )
// Keys generated for 100 years so can ignore knowing current time
//
// Using: cat keys_https/ca.crt
static const char serverCert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDijCCAnICCQCIyzuaSwtR2zANBgkqhkiG9w0BAQsFADCBhTELMAkGA1UEBhMC
QVUxDDAKBgNVBAgMA1FMRDEPMA0GA1UEBwwGQ2Fpcm5zMRMwEQYDVQQKDApDbG9j
a1RoaW5nMQ0wCwYDVQQLDARyb290MQ8wDQYDVQQDDAZnaW90dG8xIjAgBgkqhkiG
9w0BCQEWE3AubXVzdW1lY2lAaWVlZS5vcmcwIBcNMjIwNzMwMTEwOTA5WhgPMjI0
MTA4MTExMTA5MDlaMIGFMQswCQYDVQQGEwJBVTEMMAoGA1UECAwDUUxEMQ8wDQYD
VQQHDAZDYWlybnMxEzARBgNVBAoMCkNsb2NrVGhpbmcxDTALBgNVBAsMBHJvb3Qx
DzANBgNVBAMMBmdpb3R0bzEiMCAGCSqGSIb3DQEJARYTcC5tdXN1bWVjaUBpZWVl
Lm9yZzCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALwT/lg/yhds+YLZ
YFxqYKLHrwjFgEm3SQsbR1hoyQWKfCnsJ2DL2Eu1VCyqQ+TB6rg08/0rR31WF8Vn
frmmRnMBrizyzt5WiYsf6tl6xzwIq7/UVd6kZ1CL46K0LCw66+35oxFFJgq9WG9q
uyUBEQN3IjjHQr47yQS2edYUZnKcHij/FhIeZsQwNmbVvPtQ7nW95zAHTQoso1L1
pvkpc2YNLO2VSBNCI44jOs5YrW5ql6fKhR5ZYwls/D5xn4g/gijHCki0dsQX1DeC
9b3KEBqUrVWFOCyKTAJmAC7I06UG19INNANAtZiHQm+sqRAmqpz6QH0/HWnwKGsH
MF4bNJsCAwEAATANBgkqhkiG9w0BAQsFAAOCAQEAX23yC323rBYmIE953WkGvJWz
oi+PSIuUAnX8wUpw9LsHHlJHrO5uFQBfeHCGBvb16KPEOnF1xazU7wLeoD549z9q
5m1hze5I1jJxb9pwLulsDqUgJEyfdrNtg+LmRfgSKEMPOj57iGvmG8laKRLS19Cz
hEW2VqvTYwywue2xW2H4rF5D0M/lgcRicpAhDrm/q4zThHagawyoxCcfRKaiXA90
Rw2kXtYRH3mJtRIwqsjfgko43zpbu9SCwffUxisTBZsCL47TZa+BQo1WbJmSJwRe
6TxDDw1XmD6tq5YuGM9P+zxVt3cwm8Koa6J4jw1HnMhYf86CFY2SN9ffX3bi+A==
-----END CERTIFICATE-----
)EOF";
//
// Using: cat keys_https/ca.key
static const char serverKey[] PROGMEM =  R"EOF(
-----BEGIN PRIVATE KEY-----
MIIEvwIBADANBgkqhkiG9w0BAQEFAASCBKkwggSlAgEAAoIBAQC8E/5YP8oXbPmC
2WBcamCix68IxYBJt0kLG0dYaMkFinwp7Cdgy9hLtVQsqkPkweq4NPP9K0d9VhfF
Z365pkZzAa4s8s7eVomLH+rZesc8CKu/1FXepGdQi+OitCwsOuvt+aMRRSYKvVhv
arslAREDdyI4x0K+O8kEtnnWFGZynB4o/xYSHmbEMDZm1bz7UO51vecwB00KLKNS
9ab5KXNmDSztlUgTQiOOIzrOWK1uapenyoUeWWMJbPw+cZ+IP4IoxwpItHbEF9Q3
gvW9yhAalK1VhTgsikwCZgAuyNOlBtfSDTQDQLWYh0JvrKkQJqqc+kB9Px1p8Chr
BzBeGzSbAgMBAAECggEBAIwwkvuQwMBpWCpygWMhX9q14QA2xwMrfWSr9qgtovlg
WmCdSFTcXONBy/ZnC6ht6tlXjy58nhMMozdfh5Q2HnkHHZ8fdZdBBdEP/2wQGoXj
KLIaQc9s2B2vyAO1LN/jI40E8Mmks1B6Bns2ITzZ6KaenIGZ9WP7CEFdCr/KAPsX
nxLjG3EjQ4EOFGLW95RVhuO2w++gOrfKLdMexLCxx0lafEuE5o7f2VAwoPAqjPYm
WqBocPW7jcesgW3IfJfII398WdpDonNcuv+13+7FmOiZADeJUkR4litJNAUwutyy
WttpCY5DtHZgRfWfLTnzaciIe483NHpKua1f3/BgrtECgYEA7sFPwKG8zyFQQsQQ
whPzeHByKGTYp83+M8Qj/ohWKH/PioRqeecNfKtIYh/rL0wTOe6+GlL7i5yBY+6j
DwIm1Z2W6bLCq5k1GPDEKTDobofdmL9zUACxI7WlqJJ6fN6MwsIvRht92K0ogBWM
7VLg88lWcSFETR1TKwcBUloZaH0CgYEAyamkK3CpFz3UPJ8CmJhVAcIahH72idvC
6Z0R8ycOou80CR1fU1ikrCsck+aDukVUpb6meJ42IZ1PU7Yvtijv1UO9R3z7NlzV
DUcjljk8BYJQEXhz7/W1h2fZIjJuzxyFIbQQVpKoqBAfloqv8BjKJNC22oXxIXpN
67rG6+w2NPcCgYEA6XVHGbJNIDCP/akgRdYiZWt2hP8PRw5K+a1aEOKZdOJ9OxH9
15NRgc0uaxyQ6N+3kRdc9fqck0iG0QOgl+VfudNtpLiyV/7oh6mt9iUnsiTSTzEt
fVYf4C4XfT013nHzK+GbPi9VoBC5oHzyH4HCCz+dceO4mLsK/vDLnw4Q+DUCgYEA
tctAPAjr5g/O7HdMF9P7jgI1dYwfJFgAdikqrVBgWXHaLv1TmgcnLC4RLSN5TMqD
bIdZ+xJlKhhuEsECceyliFPwpil9LZyyU6313s7p1O2M0LMNkRyn8NElZTahvgOv
4Jo6wFClyhv+3e9Ye9FdKaMdtBFbNxrSc9eEy9m9ZDECgYB5f43AbbKMVDaXn3td
V3wavcZAZu/G+vJ307OH7Z7BShaKO7Fa7Z38icUIwewGOI7Pw4sCNw+4ng3vlgv1
+O+2Xkg7XBediBIaOD60r8jW+6QyU8yMrBFqHSGoY7q4Z9vQ6u9T6McokVO+zIY2
ZhXWKLrKBdpwr0VhTgTTMcozPA==
-----END PRIVATE KEY-----
)EOF";
#endif

String configure_IP_str;		// local pointer to IP used by server

//void secureRedirect() {
//  wifi_server_other.sendHeader("Location", String("https://") + configure_IP_str, true);
//  wifi_server_other.send(301, "text/plain", "");
//}

// Two standard responses are defined
// - output a setup form when the browser requests "/"
// - decode the web form details when the browser requests "/update"
// - use option_byte_sequences_id character enum values as unique name= values here
// - not (yet?) preloading default/current values via            value="current value"

// This macro converts a single char input to a 1-char C string constant, as used to generate the
// HTML form sent to the user and in selecting a value by wifi_server_main.arg() received by this
// server i.e. we ensure that the key sent in the form matches the key used to select a value
#define c2css(x) String(x).c_str()

void handleRoot() {
  wifi_server_main.send( 200,
		    "text/html",
		    (
  String( "<h3>Web configuration <br> of ClockThing! <br>\
<form action='update' method='get'>WiFi Config <br>\
&nbsp;SSID:                      <input type='text' name='" ) + c2css(id_wifi_ssid)      + String( "'> <br>\
&nbsp;PASSWORD:                  <input type='text' name='" ) + c2css(id_wifi_password)  + String( "'> <br>\
&nbsp;TimeZone (0=GMT,10=EST):   <input type='text' name='" ) + c2css(id_utc_offset)     + String( "'> <br>\
&nbsp; <br>\
Options (blank OK) <br>\
&nbsp;Display dim schedule (D means dim 10pm-6am):      <input type='text' name='" )     + c2css(id_DimSchedule) + String( "'> <br>\
&nbsp;Alt. ntp server:           <input type='text' name='" ) + c2css(id_TimeServer)     + String( "'> <br>\
&nbsp;Alt. pb gpio n:            <input type='text' name='" ) + c2css(id_pb_switch_gpio) + String( "'> <br>\
&nbsp;Test LED (ignore):         <input type='text' name='" ) + c2css(id_LED_select)     + String( "'> <br>\
&nbsp;12 Hour mode:              <input type='text' name='" ) + c2css(id_HOURS_mode)     + String( "'> <br>\
<input type='submit' value='Submit'>\
</form> </h3> <br>\
Suggested dimming schedule is D <br>\
&nbsp; <br>\
Press Submit when ready." ) ).c_str()
		    );
}

// - decode the completed form
void handleUpdate() {
  String ssid           = wifi_server_main.arg(c2css(id_wifi_ssid));
  String pwd            = wifi_server_main.arg(c2css(id_wifi_password));
  String utco           = wifi_server_main.arg(c2css(id_utc_offset));
  String opt_TimeServer = wifi_server_main.arg(c2css(id_TimeServer));
  String opt_GpioN_pb   = wifi_server_main.arg(c2css(id_pb_switch_gpio));
  String opt_Dim_sched  = wifi_server_main.arg(c2css(id_DimSchedule));
  String opt_LED_select = wifi_server_main.arg(c2css(id_LED_select));
  String opt_HOURS_mode = wifi_server_main.arg(c2css(id_HOURS_mode));

  if ((ssid.length()==0) || (pwd.length()==0)) {	// ensure minimum viable details
    allPrintln("Require ssid & pwd!");
    return;
  }

  // Handle option abbreviations
  if ( (opt_Dim_sched=="D") || (opt_Dim_sched=="d") ) opt_Dim_sched = "0,22,6";

  char *ssid_ptr = (char *)ssid.c_str();
  char *pwd_ptr = (char *)pwd.c_str();

  signed char byte__utc_offset = atoi(utco.c_str());

  // Option handling:
  // . group together option string values as ' ' separated tokens
  String multiple_options = "";
  if (opt_TimeServer.length()>0) multiple_options += id_TimeServer     + opt_TimeServer + ' ';
  if (opt_GpioN_pb.length()>0)   multiple_options += id_pb_switch_gpio + opt_GpioN_pb   + ' ';

  if (opt_Dim_sched.length()>0)  multiple_options += id_DimSchedule    + opt_Dim_sched  + ' ';
  if (opt_LED_select.length()>0) multiple_options += id_LED_select     + opt_LED_select + ' ';
  if (opt_HOURS_mode.length()>0) multiple_options += id_HOURS_mode     + opt_HOURS_mode + ' ';

							// save config (multiple_options checked in save_clk_config())
  save_clk_config(ssid_ptr, pwd_ptr, byte__utc_offset, (char*) multiple_options.c_str(), beloud);

  clear_lcd_row(0);
  allPrintln("Restart");
  wifi_server_main.send(200, "text/html", "<p> <h1>Thank you!</h1><p> <h1>Please cycle power now</h1>");
  DelSec( 5 );
  clear_lcd_row(0);
  allPrintln("Restarting");

  // From
  //   https://stackoverflow.com/questions/39688410/how-to-switch-to-normal-wifi-mode-to-access-point-mode-esp8266
  // - need to remove AP mode otherwise the access point reappears even
  //   when connecting as a client (this fix might no longer be needed)
  WiFi.softAPdisconnect();	//
  WiFi.disconnect();		// shut down AP and
  WiFi.mode(WIFI_STA);		// also select STA mode

#if defined( support_DotMatrix_DISPLAY )
  // WARNING: modifying pointers { ssid_ptr,pwd_ptr } while displaying before reset
  int cnt = strlen(ssid_ptr);
  while (cnt>6) { mdstr( (char*) ssid_ptr ); delay(1000); cnt-=6; ssid_ptr+=6; }
  cnt = strlen(pwd_ptr);
  while (cnt>6) { mdstr( (char*) pwd_ptr  ); delay(1000); cnt-=6; pwd_ptr+=6; }
#endif
  do_full_system_restart((char*) "    conF OK  ");	// never return
}

// Setup a web server on a wifi access point using a temp SSID at startup, or
// enable a web server on the existing wifi network, in order to initialisation
//
char ssid_number_AP[ 4 + 1 ];
//
// Provide ticker tape status message
String config_help;
//
void enable_reconfigure_via_wifi( int new_mode ) {

  lock_system_mode = new_mode;		// configures main polling loop to
					// support using LAN/or/AP for config

  if (using_LAN_for_config()) configure_IP_str = IP_as_string;
  else                        configure_IP_str = "192.168.4.1";

  config_help = String("    Config at http://") + configure_IP_str + "/ ";
  console->printf( "\n\rSet message string = [%d][", config_help.length() );
  console->print( config_help );
  console->print( "]\n\r" );

  DelSec( 1 );

  // create a unique tmp_ssid by appending 4 hex chars from MAC to config_ssid
  char tmp_ssid[ strlen(config_ssid)     + 4 + 1 ];

  if (using_AP_for_config()) {
    // create a unique tmp_ssid by appending 4 hex chars from MAC to config_ssid
    // (-pm- #1 duplicated code to remove)
    uint8_t mac[WL_MAC_ADDR_LENGTH];
    WiFi.softAPmacAddress(mac);
    //
    sprintf(ssid_number_AP, "%02x%02x",
	    toupper(mac[WL_MAC_ADDR_LENGTH - 2]),
            toupper(mac[WL_MAC_ADDR_LENGTH - 1]));
    sprintf(tmp_ssid, "%s%s", config_ssid, ssid_number_AP);
    WiFi.softAP(tmp_ssid);

    lcd.setCursor(0, 0);
    allPrintln("To configure, visit");
    lcd.setCursor(0, 1);
    allPrint("https://");
    IPAddress myIP = WiFi.softAPIP();
    allPrint(myIP.toString());
    Serial.println();
    lcd.setCursor(0, 2);
    allPrint("on wifi network");
    lcd.setCursor(0, 3);
    allPrint("  ");
    allPrint(tmp_ssid);
    display_7seg_str4(0,(char*) "Conf");

    MDNS.begin("clock");			// broadcast presence of "clock" on new AP
    MDNS.addService("clock", "tcp", 23);	// . for AP mode, need to advertise the telnet server
  }
  MDNS.addService("clock", "tcp", 443);		// . for AP and normal mode, need to advertise the web server
  MDNS.addService("clock", "tcp", 80);		//

#if defined( enable_HTTPS_code )
  /* wifi_server_other.on("/", secureRedirect); we choose to accept an insecure update instead of doing a redirect */
  wifi_server_other.on("/", handleRoot);
  wifi_server_other.on("/update", handleUpdate);
  wifi_server_other.begin();

  wifi_server_main.getServer().setRSACert(new BearSSL::X509List(serverCert), new BearSSL::PrivateKey(serverKey));
#endif

  wifi_server_main.on("/", handleRoot);		// specify function called on first access "/"
  wifi_server_main.on("/update", handleUpdate);	// specify function called on action       "/update"
  wifi_server_main.begin();

  Serial.println();
}

// polling_loop_for_web_configure() is the polling loop for web services when reconfiguring the system
//
int client_loop_count = 0;
#if defined( support_DotMatrix_DISPLAY )
int client_loop_count2 = 0;
#endif
//
void polling_loop_for_web_configure() {
#if defined( enable_HTTPS_code )
  wifi_server_main.handleClient();
  wifi_server_other.handleClient();
#else
  wifi_server_main.handleClient();
#endif

  delay(500);		// loop activity rate is 1/2 Hz
  allPrint(">");
  if (++client_loop_count >= lcd_COLS) {
    client_loop_count = 0;
    clear_lcd_row(1);
    Serial.println();
  }

  if (using_AP_for_config()) {
    if (4 & client_loop_count)	// switch display updates at 4/2 Hz
      display_7seg_str4(0,ssid_number_AP);
    else
      display_7seg_str4(0,(char*) "Conf");
  }

#if defined( support_DotMatrix_DISPLAY )
  if (++client_loop_count2 >= (int) config_help.length()) {
    client_loop_count2 = 0;
    console->print( "\n\r" );
  }
  mdstr( (char*) config_help.c_str() + client_loop_count2 );
#endif

  process_network_services();
  process_user_enquiry();
}

///////////////////////////////////////////////////////////////////////////
