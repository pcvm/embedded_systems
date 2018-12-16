# relayr_experiments

* esp8266_I2C_LCD_relayr_demo.ino

  - This demo program senses temperature data and sends it to a "no model" display on the relayr.io MQTT control panel.
  - esp8266_I2C_LCD_relayr_demo_esp8266.jpg shows components used.

* pdisplay_relayr_demo.py

  - This demo program is a quick hack of emonTx_energy_monitor/src_RPi/pdisplay.py so that the home energy use console now also logs 3 channels of data (export power, locally generated power, and local temperature) to 3 "no model" displays on  the relayr.io MQTT control panel.
  - pdisplay_relayr_demo_RPi.jpg shows the system in use.
