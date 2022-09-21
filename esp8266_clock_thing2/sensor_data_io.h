/* -*- mode: c++; -*- */

//
// Support code for various temperature and/or humidity sensors
//
  
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
void update_ds_temperature() {
  DS18B20.requestTemperatures();
  delay(0);
  float temp = DS18B20.getTempCByIndex(0);
  delay(0);

  if (temp <= -100) {
    strncpy(latest_ds_temperature, "?", 2);
  } else {
    String x = String(temp);
    strncpy(latest_ds_temperature, x.c_str(), nCharsTemp - 1);
  }
}

void ow_setup() {
  ds_temp_read_inProgress = 0;
  for (int i = 0; i < nCharsTemp; i++)
    latest_ds_temperature[i] = 0;

  clear_lcd_row(3);             // clear screen and storage
  allPrint("Current Temp=");
  DS18B20.begin();
  update_ds_temperature();
  allPrintln(get_temperature());
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
   
void dht_setup() {
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
void update_humidity() {
  delay(0);
  float h = dht.readHumidity();
  delay(0);

  String x = String(h);
  strncpy(latest_humidity, x.c_str(), nCharsResult - 1);
}
void update_temperature() {
  delay(0);
  float t = dht.readTemperature();
  delay(0);

  String x = String(t);
  strncpy(latest_temperature, x.c_str(), nCharsResult - 1);
}

#endif
