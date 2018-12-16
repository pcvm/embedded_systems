# emonTx_energy_monitor

* emonTxV3_4_continuous_kwhtotals_Vrms_Temp.ino

  - This is a fork of
    https://github.com/openenergymonitor/emonTxFirmware/tree/master/emonTxV3/RFM/emonTxV3.4/emonTxV3_4_continuous_kwhtotals
  - Added literate programming comments to describe calibration. See symbols: <br> Measured_Vrms_mains <br> Measured_Vrms_secondary <br> CombinedStepDown <br> voltageScaling <br> powerCal_CT*
  - Added calculation and transmission of V_rms (allows remote calibration check).
  - Added Dallas Semi DS18xxx temperature sensor reading and transmission.
  - Corrected slight accumulative error in sampling time (see new variable lastsendtime).
  - Notes:
    - A minimalist python program to display energy flow (and temperature) on a Raspberry Pi using an AdaFruit touch panel is available. This is a portable program using pipes for communication with standard builtin FreeBSD/linux utilities that handle serial port communications to the radio data link.
    - NOTE: As I'm am not completely confident with the use of a single ADC for data collection in the open energy monitor station, and the subsequent need to apply phase correction to sine wave data samples, I have frozen this work. Any further effort will involve use of separate and synchronised ADC input channels attached to an SPI bus of an ESP8266 board, which I believe could be installed in place of the battery compartment in the current hardware.

  - Extract from file comments:
<pre>
// Addition of Vrms, Temperature, literate programming comments for calibration,
//          adjust lastsendtime based on ***: Phillip Musumeci
</pre>
<pre>
// The one-wire DS1820 temperature sensor support is based on the example code at
//       https://milesburton.com/Dallas_Temperature_Control_Library
// Note: Dallas Semi application note 162
//       https://www.maximintegrated.com/en/app-notes/index.mvp/id/162
//       "Interfacing the DS18X20/DS1822 1-Wire Temperature Sensor in a Microcontroller Environment"
//       suggests each read requires approx. 750us to reset the bus (given 1 sensor)
//       and then approx. 120us/bit of data. For say 12bit data, that is of the order of 2.5ms.
#include <OneWire.h>
#include <DallasTemperature.h>
#define  W1PIN                  5       // 1-Wire pin for temperature
OneWire  ds(W1PIN);
DallasTemperature DS18B20(&ds);
</pre>
