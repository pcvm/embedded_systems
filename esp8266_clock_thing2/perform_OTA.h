/* -*- mode: c++; -*- */

// Over The Air updates
//
// . setup_LocalOTA_update() starts a web server so that a user can access a "/update" page,
//                           specify a new system binary, and upload and flash.
//                           Relies on the system being connected to the home wifi network that
//                           developer uses to access httpServerLocalOTA.
// . enable_OTA_update_from_main_server() performs a system binary download and then flashes the system.
//                           Relies on having already connected to the home wifi network and internet, so
//                           that it can access the update server via ESPhttpUpdate.

unsigned char LocalOTA_active = 0;	// set for OTA active
unsigned char is_LocalOTA_active() { return 0 != LocalOTA_active; }

#include <WiFiClient.h>
#include <ESP8266HTTPUpdateServer.h>

ESP8266WebServer httpServerLocalOTA(80);
ESP8266HTTPUpdateServer httpUpdater;

void setup_LocalOTA_update() {
  LocalOTA_active = 1;
  clear_lcd_row( 1 );
  allPrint("Local net upgrade ");
  clear_lcd_row( 2 );
  lcd.print("http://");
  lcd.print((char*) myNetworkName.c_str());
  lcd.print(".local/update");

#if defined( support_7SEGMENT_DISPLAY )
  display_7seg_str4(0,(char*) " OTA");
#endif
#if defined( support_DotMatrix_DISPLAY )
  mdstr((char*) "Upgrade");
#endif

  httpUpdater.setup(&httpServerLocalOTA);
  httpServerLocalOTA.begin();

  MDNS.addService("http", "tcp", 80);	// MDNS already active

  console->printf("HTTPUpdateServer ready! Open http://%s.local/update in your browser\n", myNetworkName.c_str());
}

void polling_loop_for_LocalOTA() {
  httpServerLocalOTA.handleClient();
  MDNS.update();
}

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#include <ESP8266httpUpdate.h>

void ota_show_progress(int cur, int total) {
  int frac = (100 * cur) / total;
#if defined( support_7SEGMENT_DISPLAY )
  display_7seg_NdigitCounter(frac);	// 7seg led or serial progress
  display_7seg_progress_update();
#else
  Serial.printf("Progress %d%%\r", frac);
#endif
  if (frac<5) {
    clear_lcd_row( 0 );			// LCD progress
    lcd.print("OTA update: ");
  } else
    lcd.setCursor(12,0);
  lcd.print((char*) String(frac).c_str());
  lcd.print("%");
  if (frac>=100) {
    clear_lcd_row( 1 );
    lcd.print("Download complete");
    clear_lcd_row( 2 );
    clear_lcd_row( 3 );
  }

#if defined( support_DotMatrix_DISPLAY )
  char fracs[16];
  sprintf(fracs," -> %3d   ", frac);	// dot matrix led progress
  mdstr(fracs);
#endif

  digitalWrite(progressLED, (0x01 & (frac/10)));	// update download indicator
}

void enable_OTA_update_from_main_server() {
  WiFiClient client;				// [2018 changed use of update()]
  ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW);	// [2018 changed use of update()]

#if defined( support_7SEGMENT_DISPLAY )
  ticker_7seg_str4( (char*) "Update", 'U', 250, 1000 );
#endif
#if defined( support_DotMatrix_DISPLAY )
  mdstr((char*) "OTA");
#endif
  clear_lcd_row( 1 );			// LCD progress
  lcd.print("OTA enabled");
  clear_lcd_row( 2 );
  clear_lcd_row( 3 );
  console->printf("Fetching %s:%d%s\n\n",
		  OTA_server_name, OTA_port_number, OTA_file_name);
  // This OTA kills all existing network services so allow time for message transfer to user
  process_network_services();
  delay(1000);
  process_network_services();

				// PIN ASSIGNMENT 
  pinMode(2, OUTPUT);		// use GPIO2 pin as output for download indicator

#if defined( support_7SEGMENT_DISPLAY )
  display_7seg_NdigitCounterInit(3);
  display_7seg_activity_clear();
#endif

  ESPhttpUpdate.onProgress(ota_show_progress);
  ESPhttpUpdate.update( client, OTA_server_name, OTA_port_number, OTA_file_name );	// [2018 changed use of update()]
}

///////////////////////////////////////////////////////////////////////////
