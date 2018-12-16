#
# Display power information from an openenergy.org emonTxV3_4 basestation
# connected to an RFM12Pi receiver.
#
# Software requirements: python, the cu terminal utility.
#
# Local Hardware Setup
#
#                 emonTxV3_4 running modified emonTxV3_4_continuous_kwhtotals
#                   |
#                   |(radio link)
#                   V
#                 RFM12Pi --> usb-uart --> linux/*bsd system
#
# Overall system information:
#
#  - py-serial may not be easily installed because it may not be compatible
#    with current OS releases. The workaround employed here is to run the
#    terminal emulator "cu" is a thread, accessing the RPI12Pi via a serial
#    port such as /dev/cuaU0 or /dev/ttyUSB0;
#  - this program assumes that the remote emonTx is running the continuous
#    analysis software "emonTxV3_4_continuous_kwhtotals.ino" but with Vrms
#    now computed and transmitted as an aid in calibration and checking;
#  - this program assumes that the local RFM12Pi has been set to the appropriate
#    frequency (for Australia, we choose 433MHz via command '4b' and then
#    lock it via command '123x').
#  - to disable screen blanking so there is no need to wake the system, use
#      xset s reset && xset dpms force off
#    (verified when using window sized at needScreenSize)
#

import datetime
import time

from subprocess import Popen, PIPE, STDOUT

import threading	# run RFM12Pi data retrieve&display as background task
import sys

# Common serial port names:
#    /dev/cuaU0   FreeBSD
#    /dev/ttyUSB0 Linux
serial_port='/dev/ttyUSB0'

unset_temperature = -321

###########################################################################

import json
#import time

import paho.mqtt.client as mqtt


# mqtt credentials
creds = {
    'clientId': '__enter client id here__',
    'user':     '__enter user here__',
    'password': '__enter password here__',
    'topic':    '__enter topic here__',
    'server':   'mqtt.relayr.io',
    'port':     1883
}


# ATTENTION !!!
# DO NOT try to set values under 200 ms of the server
# will kick you out
publishing_period = 1000*2	# try free running 2s period since eMonTx sampling interval = 10s


class MqttDelegate(object):
    "A delegate class providing callbacks for an MQTT client."

    def __init__(self, client, credentials):
        self.client = client
        self.credentials = credentials

    def on_connect(self, client, userdata, flags, rc):
        print('Connected.')
        # self.client.subscribe(self.credentials['topic'].encode('utf-8'))
        self.client.subscribe(self.credentials['topic'] + 'cmd')

    def on_message(self, client, userdata, msg):
        print('Command received: %s' % msg.payload)

    def on_publish(self, client, userdata, mid):
        pass
#        print('Message published.')


def main(credentials, publishing_period):

    global energyReportThread_quit_ack
    global watts_totalImports
    global watts_totalGeneration
    global temp_1
    global watts_totalImports_lastReported
    global watts_totalGeneration_lastReported
    global temp_1_lastReported


    client = mqtt.Client(client_id=credentials['clientId'])
    delegate = MqttDelegate(client, creds)
    client.on_connect = delegate.on_connect
    client.on_message = delegate.on_message
    client.on_publish = delegate.on_publish
    user, password = credentials['user'], credentials['password']
    client.username_pw_set(user, password)
    # client.tls_set(cafile)
    # client.tls_insecure_set(False)
    try:
        print('Connecting to mqtt server.')
        server, port = credentials['server'], credentials['port']
        client.connect(server, port=port, keepalive=60)
    except:
        print('Connection failed, check your credentials!')
        return

    # set 200 ms as minimum publishing period
    if publishing_period < 200:
        publishing_period = 200

    while True:
        if energyReportThread_quit_request: break

        client.loop()

        # publish data
        if temp_1_lastReported != temp_1:
            message = {
                'meaning' : 'Temperature (external)',
                'value'   : temp_1
            }
            (result, mid) = client.publish(credentials['topic'] +'data', json.dumps(message))
            temp_1_lastReported = temp_1
        #
        if watts_totalImports_lastReported != watts_totalImports:
            message = {
                'meaning' : 'Power Exports',
                'value'   : (-watts_totalImports)
            }
            (result, mid) = client.publish(credentials['topic'] +'data', json.dumps(message))
            watts_totalImports_lastReported = watts_totalImports
        #
        if watts_totalGeneration_lastReported != watts_totalGeneration:
            message = {
                'meaning' : 'Power Generation',
                'value'   : watts_totalGeneration
            }
            (result, mid) = client.publish(credentials['topic'] +'data', json.dumps(message))
            watts_totalGeneration_lastReported = watts_totalGeneration

        time.sleep(publishing_period / 1000.)


#if __name__ == '__main__':
#    main(creds, publishing_period)
def relayr_reporting_thread():
    main(creds, publishing_period)

###########################################################################
#
# define functions to decode next data packet element
#

# Decode int8 and int16 values
def get_int8( b_array, ptr ):
  return (int(b_array[ptr]),ptr+1)

def get_int16( b_array, ptr ):
  return (int(b_array[ptr])+(256*int(b_array[ptr+1])),ptr+2)

def get_int32( b_array, ptr ):
  (sum1,ptr) = get_int16( b_array, ptr )
  (sum2,ptr) = get_int16( b_array, ptr )
  return (sum1 + pow(2,16)*sum2, ptr)

#def get_int64( b_array, ptr ):
#  (sum1,ptr) = get_int32( b_array, ptr )
#  (sum2,ptr) = get_int32( b_array, ptr )
#  return (sum1 + pow(2,32)*sum2, ptr)

def decode2sComp( x, n_bits ):
  global binary_powers
  if x > binary_powers[n_bits]:
    print "\n? calling decode2sComp( %d, %d )" % (x, n_bits)
  if x > binary_powers[n_bits-1]-1:
    return x-binary_powers[n_bits]
  else:
    return x

def get_real16( b_array, ptr ):
  return (int(b_array[ptr])+(256*int(b_array[ptr+1])),ptr+2)

def is_data_packet( dp ):
  return ( dp[1]>='0' and dp[1]<='9' and dp[2]>='0' and dp[2]<='9' )

###########################################################################

#
# data_collection_thread() is a thread that waits for and inputs text lines
#                          from a subprocess cu task attached to a uart and
#                          the RFM12Pi.
#                          It terminates when energyReportThread_quit_request
#                          is found set before for the next RFM12Pi data input.
#                          The data format depends on AVR program
#                          emonTxV3_4_continuous_kwhtotals.ino (modified to
#                          include Vrms (calibration) and temperature sensing).

def data_collection_thread():
  print 'Data collection thread starting'
  global energyReportThread_quit_request
  global energyReportThread_quit_ack
  global energyReportVerbose

  global watts_totalImports
  global watts_totalGeneration
  global temp_1

  global T_summary
  global power_summary

  the_display.clear()
  position = (0, 1*36)
  text_surface = font.render(' '*25, True, (0,0,0))		# overwrite with black
  text_surface = font.render('Starting...', True, (0,0,0))	# overwrite with black
  the_display.screen.blit(text_surface, position)

  dp_p  = [0.]		# dp_* arrays hold most recent data
  dp_wh = [0.]
  if N_sensors<1:
    print '??? error, N_sensors is too small ???'
  if N_sensors>1:
    for x in range(1,N_sensors):
      dp_p.append(0.)
      dp_wh.append(0.)

  msgNumber = 0
  msgNumber_increment = 5	# emonTxV3_4_continuous_kwhtotals pkt ID increment
  total_missed_packets = 0
  startup = True

  LED=True

  cu_task.stdout.flush()
  while True:
    if energyReportThread_quit_request: break
    dp = cu_task.stdout.readline()	# decode text lines in 'data packet' dp

    if (dp.find('> 0l')==0) or (dp.find('> 1l')==0):
      if energyReportDebug: print 'LED toggled'
      continue	# skip LED commands

    current_time = datetime.datetime.now().time()
    dp = dp.rstrip()
  
    if len(dp) < 3: continue
    if is_data_packet( dp ):
      if energyReportDebug:
        print 'At %s, received:        (%d missed packets)' % (current_time.isoformat(),total_missed_packets)
        print dp
    else:  
      print dp
      continue
  
    dp = dp.strip()		# packet passes data test so remove unwanted spacing

    if LED: cu_task.stdin.write("1l")
    else:   cu_task.stdin.write("0l")
    LED = not LED

    # decode power data and print
    dp_n = dp.split(" ")
  				# extract various numeric values,
  				# assuming 2s complement as appropriate
    ptr=0
    (dp_ch,ptr)     = get_int8(dp_n,ptr)
    mn_tmp = msgNumber
    (msgNumber,ptr) = get_int32(dp_n,ptr)
    status="%2d/%d" % (dp_ch,msgNumber)
  
    if startup:
      print '*** First valid packet received'
      startup=False
    else:
      n_missed_packets = (msgNumber-mn_tmp)/msgNumber_increment-1	## integer maths
      if n_missed_packets>0:
        total_missed_packets = total_missed_packets + n_missed_packets
        print '*** Missed %d packets' % n_missed_packets
  
  				# Vrms provides a sanity check for magnitudes
    (Vrms,ptr)      = get_int16(dp_n,ptr)
    status=status+" Vrms=%d" % Vrms
  
  				# Note that powers and WattHr values are signed
    powers='; Powers:'
    for x in range(N_sensors):
      (dp_p[x],ptr) = get_int16(dp_n,ptr)
      dp_p[x] = decode2sComp( dp_p[x],16 )
      if x < N_sensors_active: powers=powers + "%d, " % dp_p[x]
    powers=powers[0:-2]
  
    WattHr='; WHr:'
    for x in range(N_sensors):
      (dp_wh[x],ptr) = get_int32(dp_n,ptr)
      dp_wh[x] = decode2sComp( dp_wh[x],32 )
      if x < N_sensors_active: WattHr=WattHr + "%d, " % dp_wh[x]
    WattHr=WattHr[0:-2]

    T_summary=''
    temp_temperature=unset_temperature
    if N_temperature_active>0:
      for Tsensor_ID in range(N_temperature_active):
        (Tsensor_Temp,ptr) = get_int16(dp_n,ptr)	# Note: temp data scaled by 100
        if temp_temperature == unset_temperature:
          temp_temperature = Tsensor_Temp/100.0
        T_summary=T_summary+ " T%d=%5.2fC" % (Tsensor_ID+1,Tsensor_Temp/100.0)
      status=status+T_summary

    watts_totalImports    = dp_p[ id_totalImports ]	# update globals
    watts_totalGeneration = dp_p[ id_totalGeneration ]
    temp_1 = temp_temperature

    power_summary = '%s: %s; Pkts missed: %d' % (current_time.isoformat()[0:10],status + powers + WattHr, total_missed_packets)
    if energyReportVerbose:
      print power_summary
    if writeStatusFile:
      sf = open( StatusFile, "w" )
      sf.write("%s\n" % power_summary)
      sf.close()

    # update display
    current_time = datetime.datetime.now().time()
    the_time = 'Updated at %s' % current_time.isoformat()[0:8]

    # Display a timestamp, then total import/export power value as
    # red/green text and finally current generated power as sea-green
    # text, all on black background. Each text line is written at
    # multiples of 36 pixel down the screen.
    #
    the_display.clear()
    #
    position = (0, 0*36)
    text_surface = font.render(the_time, True, (255, 255, 255))	# RGB for white timestamp
    the_display.screen.blit(text_surface, position)
    #
    position = (0, 1*36)
    text_surface = font.render(' '*25, True, (0,0,0))		# overwrite with black
    the_display.screen.blit(text_surface, position)
    if watts_totalImports<0:
      status = "Exporting %dW" % (-watts_totalImports)
      text_surface = font.render(status, True, (0, 255, 0))	# RGB for green, exporting
    else:
      status = "Importing %dW" % watts_totalImports
      text_surface = font.render(status, True, (255, 0, 0))	# RGB for red, importing
    the_display.screen.blit(text_surface, position)
    #
    position = (0, 2*36)
    text_surface = font.render(' '*25, True, (0,0,0))		# overwrite with black
    the_display.screen.blit(text_surface, position)
    status = "(generating %dW)" % watts_totalGeneration
    text_surface = font.render(status, True, (0, 128, 255))	# RGB for sea-green, generation
    the_display.screen.blit(text_surface, position)
    #
    position = (0, 3*36)
    text_surface = font.render(' '*25, True, (0,0,0))		# overwrite with black
    the_display.screen.blit(text_surface, position)
    status = T_summary
    text_surface = font.render(status, True, (0, 128, 255))	# RGB for sea-green, generation
    the_display.screen.blit(text_surface, position)
    #
    # Update the display
    pygame.display.update()


  # cleanup
  cu_task.stdout.close()
  cu_task.stdin.close()

  energyReportThread_quit_ack=True
  print '*** Data collection thread has finished'

###########################################################################

import os
import pygame

class adafruit_display:
    screen = None;
    
    def __init__(self):
        "Ininitializes a new pygame screen using the framebuffer"
        # Based on "Python GUI in Linux frame buffer"
        # http://www.karoltomala.com/blog/?p=679
        disp_no = os.getenv("DISPLAY")
        if disp_no:
            print "Found X display = {0}".format(disp_no)
        
        # Check which frame buffer drivers are available
        # Start with fbcon since directfb hangs with composite output
        drivers = ['fbcon', 'directfb', 'svgalib']
        found = False
        for driver in drivers:
            # Make sure that SDL_VIDEODRIVER is set
            if not os.getenv('SDL_VIDEODRIVER'):
                os.putenv('SDL_VIDEODRIVER', driver)
            try:
                pygame.display.init()
            except pygame.error:
                print 'Driver: {0} failed.'.format(driver)
                continue
            found = True
            break
    
        if not found:
            raise Exception('No suitable video driver found!')
        
        size = (pygame.display.Info().current_w, pygame.display.Info().current_h)
        print " Framebuffer size: %d x %d" % (size[0], size[1])
        #
        # Use FULLSCREEN or ...
        #
        #self.screen = pygame.display.set_mode(size, pygame.FULLSCREEN)
        #
        # ... use specified size window with NOFRAME
        #
        tmpSize=needScreenSize
        if tmpSize[0] > size[0]: tmpSize[0] = size[0]
        if tmpSize[1] > size[1]: tmpSize[1] = size[1]
        size=tmpSize
        print " Using size: %d x %d" % (size[0], size[1])
        #
        self.screen = pygame.display.set_mode(size, pygame.NOFRAME)
        # Clear the screen to start
        self.screen.fill((0, 0, 0))        
        # Initialise font support
        pygame.font.init()
        # Render the screen
        pygame.display.update()

    def clear(self):			# (fast enough for small screens)
        self.screen.fill((0, 0, 0))
        pygame.display.update()

    def __del__(self):
        "Destructor to make sure pygame shuts down, etc."

    def test(self):
        # Fill the screen with red (255, 0, 0)
        red = (255, 0, 0)
        self.screen.fill(red)
        # Update the display
        pygame.display.update()

#-    def drawGraticule(self):
#-        "Renders an empty graticule"
#-        # The graticule is divided into 10 columns x 8 rows
#-        # Each cell is 50x40 pixels large, with 5 subdivisions per
#-        # cell, meaning 10x8 pixels each.  Subdivision lines are
#-        # displayed on the central X and Y axis
#-        # Active area = 10,30 to 510,350 (500x320 pixels)
#-        borderColor = (255, 255, 255)
#-        lineColor = (64, 64, 64)
#-        subDividerColor = (128, 128, 128)
#-        # Outer border: 2 pixels wide
#-        pygame.draw.rect(self.screen, borderColor, (8,28,504,324), 2)
#-        # Horizontal lines (40 pixels apart)
#-        for i in range(0, 7):
#-            y = 70+i*40
#-            pygame.draw.line(self.screen, lineColor, (10, y), (510, y))
#-        # Vertical lines (50 pixels apart)
#-        for i in range(0, 9):
#-            x = 60+i*50
#-            pygame.draw.line(self.screen, lineColor, (x, 30), (x, 350))
#-        # Vertical sub-divisions (8 pixels apart)
#-        for i in range(1, 40):
#-            y = 30+i*8
#-            pygame.draw.line(self.screen, subDividerColor, (258, y), (262, y))
#-        # Horizontal sub-divisions (10 pixels apart)
#-        for i in range(1, 50):
#-            x = 10+i*10
#-            pygame.draw.line(self.screen, subDividerColor, (x, 188), (x, 192))


###########################################################################

# RPi/adafruit screen has framebuffer size: 320 x 240

needScreenSize = [320, 232]

# Lookup constants for binary powers of 2 for 0..64
binary_powers = [0] * 65
for x in range(65):
  binary_powers[x] = 2**x

# Runtime options (set defaults, allow override)
energyReportVerbose = True
energyReportDebug   = False
#
envSymbol = os.getenv("POWER_VERBOSE")
if envSymbol != None: energyReportVerbose = envSymbol != 'NO'
envSymbol = os.getenv("POWER_DEBUG")
if envSymbol != None: energyReportDebug   = envSymbol != 'NO'
#
# The periodically update power status information should be stored on
# a ram disk tmpfs e.g. a file in area $XDG_RUNTIME_DIR. Retrieve the
# StatusFile as environment var $POWER_STATUS and adjust to be in tmpfs
# if no directory is specified.
StatusFile = os.getenv("POWER_STATUS")
if StatusFile == '' or StatusFile == None:
  StatusFile='None'
  writeStatusFile = False
else:
  if StatusFile[0] != '/': StatusFile = os.getenv("XDG_RUNTIME_DIR") + '/' + StatusFile
  writeStatusFile = True

# Connection information
N_sensors=4		  # emonTxV3_4_continuous_kwhtotals.ino handles 4 I inputs
N_sensors_active=2	  # enter number of attached current sensors
#
N_temperature_active=1    # enter number of attached temperature sensors
T_summary=''

#
id_totalImports=0	  # identify grid connection current sensor
id_totalGeneration=1	  # identify solar panel generation connection current sensor

watts_totalImports = 0	  					# initialise global vars
watts_totalGeneration = 0 					#
temp_1 = unset_temperature					#

watts_totalImports_lastReported = watts_totalImports-1		# initialise global vars, and
watts_totalGeneration_lastReported = watts_totalGeneration-1	# set to NOT match default values above
temp_1_lastReported = temp_1-1


# Initialise/clear the thread control global flags
energyReportThread_quit_request=False
energyReportThread_quit_ack=False
shutdown_wait_seconds=15

#
# Task: serial comms via popen and util cu
#

# The cu task provides a bidirectional connection to the 'serial_port'
# and the attached RFM12Pi
cu_task= Popen('/usr/bin/cu -l '+serial_port, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, universal_newlines=True, close_fds=True)

# Display preparation
# - Create an instance of the adafruit_display class
the_display = adafruit_display()
# - Get a reference to the system font, size 48
font = pygame.font.Font(None, 48)


# Start main
print
the_options = 'Modes:'
if energyReportVerbose: the_options = the_options + ' Verbose=YES'
else:                   the_options = the_options + ' Verbose=NO'
if energyReportDebug:   the_options = the_options + ' Debug=YES'
else:                   the_options = the_options + ' Debug=NO'
the_options = the_options + ' StatusFile=%s' % StatusFile

print the_options
print

#
# Task: data collection thread
#

thread = threading.Thread(target = data_collection_thread)
power_summary = ''
thread.start()

#
# Task: relayr reporting
#

thread2 = threading.Thread(target = relayr_reporting_thread)
thread2.start()

#
# Main:
#

print '''

Local commands can be sent to the RFM12Pi (as text followed by ENTER)
with responses displayed here.
Local commands: q ENTER == quit; p ENTER == latest powers

'''

while True:

  input = raw_input()

  if input[0] == 'q':
    print '*** Shutdown has commenced (will complete after next update)'
    energyReportThread_quit_request=True
    time_out=shutdown_wait_seconds
    while time_out>0:
      if energyReportThread_quit_ack: break
      time.sleep( 1 )
      time_out=time_out-1
      if time_out < 5: print " %d " % time_out
    thread.join()
    break		#sys.exit(0)
  if input[0] == 'v':
    print 'Status display sent'
    cu_task.stdin.write("v")
  if input[0] == 'V':
    energyReportVerbose=not energyReportVerbose
  if input[0] == 'p':
    print power_summary

thread2.join()		# better behaved thread, do simple join
the_time = datetime.datetime.now().time()
print '%s: normal exit' % the_time.isoformat()[0:10]
