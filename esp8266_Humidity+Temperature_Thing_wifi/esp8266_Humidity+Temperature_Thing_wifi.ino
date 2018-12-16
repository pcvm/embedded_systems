// My Temperature and Humidity Thing
//
// Using NodeMCU https://github.com/nodemcu/nodemcu-devkit-v1.0
// Using Adafruit DHT-sensor-library for DHT22/AM2302
// Using AM2302 on a NodeMCU ESP-12 Development Kit
//       - from https://learn.adafruit.com/dht/connecting-to-a-dhtxx-sensor, the AM2302 is
//         a wired version of the DHT22
//       - at https://www.adafruit.com/product/393:
//         . "AM2302 is a wired version of the DHT22"
//         . has a 5.1K resistor pullup connecting VCC and DATA so external pullup not needed
//         . connect pin 1 to 3.3V, pin 2 to GPIO 17, pin3 is NC, and pin 4 is GND
//       - top view with mounting surface facing down and bolt hole upper:
//         . pins 1234 are left to right as VCC, DATA, NC, GND
//
// To use: enter wifi info for nw_ssid and nw_passwd, rebuild and upload to target hardware.
//

//
// Options
#define LED_flashOnRead 1

//
// GPIO
#define DHTportN 4	// connect DHT sensor to GPIO 4
#define LEDportN 2

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

float humidity = 0.0;
float temperature = 0.0;

#if defined( LED_flashOnRead )
int LED_state=0;
#endif

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

// Uncomment the type of sensor in use:
//#define DHTTYPE           DHT11     // DHT 11 
#define DHTTYPE           DHT22     // DHT 22 (AM2302)
//#define DHTTYPE           DHT21     // DHT 21 (AM2301)

DHT_Unified dht(DHTportN, DHTTYPE);

uint32_t delayMS;

///////////////////////////////////////////////////////////////////////////

#include <ESP8266WiFi.h>

const char* nw_ssid   = "my_wifi_ssid";
const char* nw_passwd = "my_wifi_password";

// Set web server port number to 80
WiFiServer server(80);

// Retain IP address as a string
String the_ipaddress;

void network_setup() {
  Serial.print("Connecting to ");
  Serial.print(nw_ssid);
  Serial.print("/");
  Serial.println(nw_passwd);

  WiFi.mode(WIFI_STA);			// ensure STA mode only
  WiFi.begin(nw_ssid, nw_passwd);
  int cnt=0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (++cnt >= 40 ) {
      cnt=0;
      Serial.println();
    }
  }
  // display IP address and start web server
  Serial.println();
  the_ipaddress = WiFi.localIP().toString();
  Serial.println("Allocated an IP = " + the_ipaddress);

  delay(500);
  server.begin();
}

void info_web_server(){
  WiFiClient client = server.available();	// listen for incoming clients

  if (client) {					// have new connection
    Serial.println("New connection");
    if (client.connected()) {
      // send HTTP header
      client.println("HTTP/1.1 200 OK");
      client.println("Content-type:text/html");
      client.println("Connection: close");
      client.println();
            
      // display HTML web page
      client.println("<!DOCTYPE html><html>");
      client.println("<body><h3> Server " + the_ipaddress + " </h3>");
      // date
      client.print("<p> Humidity = ");
      client.print(humidity);
      client.println("% </p>");

      client.print("<p> Temperature = ");
      client.print(temperature);
      client.println("C </p>");

      client.println("</body></html>");
            
      // HTTP response ends with another blank line
      client.println();
      // close the connection
      client.stop();
    }
  }
}

///////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println();

  Serial.println("Phillip's Humidity&Temperature Things");

  Serial.println("DHT initialisation using adafruit/DHT-sensor-library");
  Serial.println("DHT22/AM2302 driver");

  dht.begin();
  sensor_t sensor;
					// display temperature sensor details
  dht.temperature().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");  
  Serial.println("------------------------------------");
					// display humidity sensor details
  dht.humidity().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");  
  Serial.println("------------------------------------");
					// set min delay between sensor readings based on sensor details
  delayMS = sensor.min_delay / 1000;

#if defined( LED_flashOnRead )
  pinMode( LEDportN, OUTPUT );
#endif

  Serial.println("Network initialisation");
  network_setup();

  Serial.println();
  Serial.println("All systems go...");
}

void loop()
{
  humidity = temperature = -1.0;	// default values before update

  delay(delayMS);			// observe delay between measurements
  
  sensors_event_t event;  		// read T and H and display
					// update temperature
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature");
  } else {
    Serial.print("Temperature = ");
    Serial.print(event.temperature);
    temperature = event.temperature;
    Serial.print("C");
  }
					// update humidity
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity");
  } else {
    Serial.print(", Humidity = ");
    Serial.print(event.relative_humidity);
    humidity = event.relative_humidity;
    Serial.println("%");
  }
#if defined( LED_flashOnRead )
  LED_state = !LED_state;
  digitalWrite( LEDportN, LED_state );
#endif

  info_web_server();			// serve out latest valid data (or -1 values for sensor read error)
}

///////////////////////////////////////////////////////////////////////////
