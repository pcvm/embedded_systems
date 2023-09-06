#
# Phillip Clock Thing
#

# NOTES
#
# 0) Need the 'make clean' before every compile in case arduino IDE updates libs
#
# 1) Need an output dir for build output
#    . Use ./build/ directory which is the default used by compiler and uploader
#      - leaving unspecified is the same as SetOutputDir=--output-dir ./build/
SetOutputDir=
#
# 2) Assume the device is on the highest numbered SLAB_USBtoUARTn port (macOS)
#    . See $(arduino-cli board list) for possible ports
#      - suggested /dev/tty.SLAB_USBtoUART for first port
#
PORT := $(shell TERM= ls /dev/tty.SLAB_USBtoUART* 2>/dev/null|tail -1)
#
# 3) Need target board id
#    . Looked up nodemcu_v1.0 symbols in gui build and found "nodemcuv2"
#    . See $(arduino-cli board listall | grep nodemcu) for possible board id
#      - suggested esp8266:esp8266:nodemcuv2                  
#

###########################################################################

all:
	@echo "Target     Purpose"
	@echo "------     ------------------------"
	@echo "b        - clean compiler cache then build"
	@echo "d        - do a copy of current binary to desktop"
	@echo "bd       - make b then d"
	@echo "c        - compare current binary against most recent published binary (a test for BINARY UNCHANGED)"
	@echo "warnings - pedantic compile"
	@echo "pub      - publish binary (default using arduino-cli)"
	@echo
	@echo "tags     - update TAGS file"
	@echo "git      - update local github files"
	@echo "tools    - show extra help about tools and libraries"
	@echo
	@echo "list     - list connected boards and choose one"
	@echo "upload   - upload .bin file to last board detected in 'list'"
	@echo "up       - upload .bin file to board at $(PORT)"
	@echo

###########################################################################

# have also tried filter: egrep "^/dev/cu.*usb.*"
# also tried:             egrep "^/dev/tty.SLAB_USBtoUART*"
# currently:              egrep "^/dev/cu.SLAB_USBtoUART*"

list:
	@echo "Listing:"
	sh -c 'arduino-cli board list | tee -a /dev/stderr | egrep "^/dev/cu.SLAB_USBtoUART*" | while read x y z ; do echo $$x > _board.list ; done'
	@echo "Saved board:"
	@cat _board.list

upload:
	arduino-cli upload --port $(shell cat _board.list) --fqbn esp8266:esp8266:nodemcuv2 -i build/esp8266.esp8266.nodemcuv2/Phillip_Clock_Thing2.ino.bin
	tio -b 115200 $(shell cat _board.list)

###########################################################################

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
	bash ./user_help_strings.sh
	@make sysinfo
	arduino-cli compile --libraries ~/micros/Local_libraries+sources/Arduino/libraries --warnings more --fqbn esp8266:esp8266:nodemcuv2 ./Phillip_Clock_Thing2.ino --output-dir `pwd`/build/esp8266.esp8266.nodemcuv2/
	ls -al build/esp8266.esp8266.nodemcuv2/

d:
	cp -p build/esp8266.esp8266.nodemcuv2/Phillip_Clock_Thing2.ino.bin ~/Desktop/

bd:
	@make b
	@make d

c:
	ls -l build/esp8266.esp8266.nodemcuv2/Phillip_Clock_Thing2.ino.bin /usr/local/var/www/Clock_Thing2.nodemcu.bin
	@echo
	@sh -c "echo 'Comparing ... ' ; diff -q build/esp8266.esp8266.nodemcuv2/Phillip_Clock_Thing2.ino.bin /usr/local/var/www/Clock_Thing2.nodemcu.bin && echo MATCH || echo ERROR"

pub:
	@echo "Copying from build/esp8266.esp8266.nodemcuv2 to /usr/local/var/www"
	cp -p build/esp8266.esp8266.nodemcuv2/Phillip_Clock_Thing2.ino.bin /usr/local/var/www/Clock_Thing2.nodemcu.bin
	chmod a+r /usr/local/var/www/*Clock_Thing2.ino.bin
	@echo
	ls -l     build/esp8266.esp8266.nodemcuv2/Phillip_Clock_Thing2.ino.bin
	@echo
	ls -l     /usr/local/var/www/*.bin
	@echo
	@echo "See also: make checkdocs"
	@echo

unpub:
	@echo "Removing /usr/local/var/www/Clock_Thing2.nodemcu.bin"
	rm -f           /usr/local/var/www/Clock_Thing2.nodemcu.bin
	ls -l /usr/local/var/www/*.bin

checkdocs:
	@echo Current docs:
	@echo $(realpath ClockThing_main_documentation)
	@sh -c "cd ClockThing_main_documentation ; chmod a+r *.pdf ; ls -l *.pdf"
	@echo
	@echo After any update, export the PDF using the \"/System/Applications/Utilities/ColorSync Utility.app\"
	@echo filter \"Reduce File Size 600dpi+betterJPEG \(docs 1/3\)\" which appears as a preview export option.
	@echo
	@echo Published docs:
	@echo '  enable read access: chmod a+r /usr/local/var/www/docs/*.pdf'
	@echo '  ls -l /usr/local/var/www/docs'
	@ls -l /usr/local/var/www/docs
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
