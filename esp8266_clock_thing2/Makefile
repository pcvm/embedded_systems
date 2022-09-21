#
# Phillip Clock Thing
#

###########################################################################

# NOTES
# 0) Need the 'make clean' before every compile in case arduino IDE updates libs
# 1) Currently leaving output dir unspecified so default is used (look in the
#    ./build/ directory which is found automatically by the uploader)
#SetOutputDir=--output-dir ./build/
SetOutputDir=
#
# 2) Assume the device is on the highest numbered SLAB_USBtoUARTn port (macOS)
PORT := $(shell TERM= ls /dev/tty.SLAB_USBtoUART* 2>/dev/null|tail -1)
#
# 3) Need target board id
##   Looked up nodemcu_v1.0 symbols in gui build and found "nodemcuv2"
#
## 3.1) Try a board ID mechanism to probe actual hardware
##
# $ arduino-cli board list
#|Port                            Type              Board Name FQBN Core
#|/dev/cu.Bluetooth-Incoming-Port Serial Port       Unknown
#|/dev/cu.HT1-BluetoothSerialPort Serial Port       Unknown
#|/dev/cu.SLAB_USBtoUART          Serial Port (USB) Unknown
#|/dev/cu.SLAB_USBtoUART7         Serial Port (USB) Unknown             
# \_ no result
#
## 3.2) Fall back to listing all boards and search for ttgo
##
#| $ giotto $ arduino-cli board listall | grep nodemcu
#| NodeMCU 0.9 (ESP-12 Module)      esp8266:esp8266:nodemcu                    
#| NodeMCU 1.0 (ESP-12E Module)     esp8266:esp8266:nodemcuv2                  
#| NodeMCU-32S                      esp32:esp32:nodemcu-32s                    
# \_ suggests entry esp8266:esp8266:nodemcuv2                  
#

###########################################################################

all:
	@echo "Target   - Purpose"
	@echo "b        - clean compiler cache then build"
	@echo "d        - make b and copy binary to desktop"
	@echo "bc       - diff new binary against most recent published binary (a test for BINARY UNCHANGED)"
	@echo "warnings - pedantic compile"
	@echo "pub      - publish binary (default using arduino-cli)"
	@echo
	@echo "tags     - update TAGS file"
	@echo "git      - update local github files"
	@echo "tools    - show extra help about tools and libraries"
	@echo

tools:
	@echo "Library maintenance"
	@echo ". (July2022) arduino-cli is to become the main Arduino IDE engine"
	@echo ". some functionality still unique to Arduino IDE"
	@echo ". from https://forum.arduino.cc/t/find-what-libraries-used-so-a-new-install-can-replicate-those-libraries/984643/7"
	@printf "\nThere are 3 locations for library files:\n\nSketchbook: {sketchbook folder}/libraries\n  The sketchbook folder location can be found (or changed) at File > Preferences > Sketchbook location:.\n  This is the location of libraries installed via Library Manager (Sketch > Include Library > Manage Libraries...) or Sketch > Include Library > Add .ZIP Library.\nIDE built-in: {Arduino IDE install folder}/libraries\n  This is for the libraries included pre-installed with the Arduino IDE 1.x installation (Arduino IDE 2.x and Arduino CLI do not come with pre-installed libraries).\nPlatform bundled: This is dependent on the selection in the Tools > Board Arduino IDE menu. Location will depend on if you're using a boards platform bundled with the IDE, installed via Boards Manager, or manually installed to {sketchbook folder}/hardware.\n  Platform bundled libraries are used by boards platform authors to distribute architecture-specific libraries along with the other board support files.\n"
	@echo ""
	@echo "Libraries used"
	@echo ". see build message"
	@echo ""
	@echo "Libraries out-of-sync"
	@echo ". (July2022) manually removed all Adafruit libraries"
	@echo ". use Arduino IDE to reload until app builds"
	@echo ""
	@echo "Library search (example)"
	@echo ". arduino-cli lib search webserver"
	@echo ". locate existing source code (useful to find examples"
	@echo "    find ~/Library/Arduino15 -name ESP8266WiFi"
	@echo "    find ~/micros/Local_libraries+sources -name ESP8266WiFi"
	@echo ""
	@echo "References"
	@echo ". arduino-cli    https://www.devdungeon.com/content/arduino-cli-tutorial"
	@echo "                 https://arduino.github.io/arduino-cli/latest/getting-started/"
	@echo "                 https://github.com/arduino/arduino-cli"

b:	clean
	@make sysinfo
	arduino-cli compile --warnings more --fqbn esp8266:esp8266:nodemcuv2 ./Phillip_Clock_Thing2.ino --output-dir `pwd`/build/esp8266.esp8266.nodemcuv2/
	ls -al build/esp8266.esp8266.nodemcuv2/

d:
	make b
	cp -p build/esp8266.esp8266.nodemcuv2/Phillip_Clock_Thing2.ino.bin ~/Desktop/

bc:
	ls -l build/esp8266.esp8266.nodemcuv2/Phillip_Clock_Thing2.ino.bin /usr/local/var/www/Clock_Thing2.nodemcu.bin
	@echo
	diff build/esp8266.esp8266.nodemcuv2/Phillip_Clock_Thing2.ino.bin /usr/local/var/www/Clock_Thing2.nodemcu.bin && echo MATCH

pub:
	@echo "Copying from build/esp8266.esp8266.nodemcuv2 to /usr/local/var/www"
	cp -p build/esp8266.esp8266.nodemcuv2/Phillip_Clock_Thing2.ino.bin /usr/local/var/www/Clock_Thing2.nodemcu.bin
	chmod a+r /usr/local/var/www/*Clock_Thing2.ino.bin
	@echo
	ls -l     build/esp8266.esp8266.nodemcuv2/Phillip_Clock_Thing2.ino.bin
	@echo
	ls -l     /usr/local/var/www/*.bin
	@echo

warnings:	clean
	@make sysinfo
	arduino-cli compile --warnings all --fqbn esp8266:esp8266:nodemcuv2 ./Phillip_Clock_Thing2.ino --output-dir `pwd`/build/esp8266.esp8266.nodemcuv2/
	ls -al build/esp8266.esp8266.nodemcuv2/

###########################################################################

clean:
	@echo "Doing 'arduino-cli cache clean' (mandatory if libraries are manually manipulated)"
	arduino-cli cache clean

sysinfo:
	@echo '----------------------------------------------------------------------------------------'
	@echo
	@echo "If arduino-cli mentions a board setup error, it might need a (new) module initialisation"
	@echo "i.e.    arduino-cli core install esp8266:esp8266"
	@echo
	@echo '----------------------------------------------------------------------------------------'

show:
	arduino-cli config dump

up:
	@echo Using port $(PORT)
	arduino-cli upload --port $(PORT) --fqbn esp8266:esp8266:nodemcuv2 $(SetOutputDir)Phillip_Clock_Thing2

tags:	Makefile Phillip_Clock_Thing2.ino *.h
	etags Phillip_Clock_Thing2.ino *.h
	ls -l TAGS

git:
	mkdir -p ~/github/embedded_systems/esp8266_clock_thing2
	rsync -av --delete *.ino *.h Makefile README* ~/github/embedded_systems/esp8266_clock_thing2/
	ls -al ~/github/embedded_systems/esp8266_clock_thing2/

###########################################################################
