from RPi import GPIO
from time import sleep
import logging
import sys
import time
import statistics as stats
import math
import csv
sys.path.append('/home/pi/Adafruit_Python_BNO055')
from Adafruit_BNO055 import BNO055

#pin setups
bno = BNO055.BNO055(serial_port='/dev/serial0',rst=18)
clk = 17
dt = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup(clk, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dt, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
clkLastState = GPIO.input(clk)

#Initalizing variables
headingoffset=0
samplerate=.5
distanceperencoder=1
pi=math.pi
xloc=0
yloc=0
zloc=0
counter = 0
headinglist=[]
rolllist=[]
pitchlist=[]

with open ('calibration.csv') as caldat:
    readcsv=csv.reader(caldat, delimiter=',')#,quoting=csv.QUOTE_NONNUMERIC)
    f=list(readcsv)
    f=f[0]
    f=list(map(int, f))
    print(f)
    



# Enable verbose debug logging if -v is passed as a parameter.
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
            logging.basicConfig(level=logging.DEBUG)

    # Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

#calibrationdata=bno.get_calibration()
#print(calibrationdata)
#bno.set_calibration(calibrationdata)

# Print system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))

# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')

# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
print('Software version:   {0}'.format(sw))
print('Bootloader version: {0}'.format(bl))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

timevar=time.time()
sampletimevar=time.time()
lastencodertime=0

print('Reading BNO055 data, press Ctrl-C to quit...')
try:
#    sys, gyro, accel, mag = bno.get_calibration_status()
#    while sys<2:
#        sys, gyro, accel, mag = bno.get_calibration_status()
#        print(sys)
#        if time.time()-timevar>1:
#            print =('calibrating')
#            timevar=time.time()
    bno.set_calibration(f)
    while True:
        clkState = GPIO.input(clk)
        dtState = GPIO.input(dt)
        if clkState != clkLastState and time.time()-lastencodertime>samplerate:
            meanheading=stats.mean(headinglist)-headingoffset
            meanroll=stats.mean(rolllist)
            meanpitch=stats.mean(pitchlist)
            headinglist.clear()
            rolllist.clear()
            pitchlist.clear()
#            if dtState != clkState:
#                counter += 1
#            else:
#                counter -= 1
            print('Mean Heading={0:0.2F} Mean Roll={1:0.2F} Mean Pitch={2:0.2F}'.format(
             meanheading, meanroll, meanpitch))
#            print(math.cos(meanheading*pi/180),math.cos(meanpitch*pi/180))
            xloc=xloc+distanceperencoder*math.cos(meanheading*pi/180)*math.cos(meanpitch*pi/180)
            yloc=yloc+distanceperencoder*math.sin(meanheading*pi/180)*math.cos(meanpitch*pi/180)
            zloc=zloc+distanceperencoder*math.sin(meanpitch*pi/180)
            print('Xloc={0:0.2F} Yloc={1:0.2F} Zloc={2:0.2F}'.format(
             xloc,yloc,zloc))
            
            clkLastState = clkState
            sleep(0.001)
            lastencodertime=time.time()
            
        elif clkState != clkLastState and time.time()-lastencodertime<=samplerate:
            clkLastState = clkState
        
    
        # Read the Euler angles for heading, roll, pitch (all in degrees).
        if time.time()-sampletimevar>samplerate:
            heading, roll, pitch = bno.read_euler()
            headinglist.append(heading)
            rolllist.append(roll)
            pitchlist.append(pitch)
            # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
            sys, gyro, accel, mag = bno.get_calibration_status()
            sampletimevar=time.time()
        
        # Print everything out.
#        if time.time()-timevar>1:
#            print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
#              heading, roll, pitch, sys, gyro, accel, mag))
##            print(pitchlist)
#            timevar=time.time()
        # Other values you can optionally read:
        # Linear acceleration data (i.e. acceleration from movement, not gravity--
        # returned in meters per second squared):
        #x,y,z = bno.read_linear_acceleration()
        # Gravity acceleration data (i.e. acceleration just from gravity--returned
        # in meters per second squared):
        #x,y,z = bno.read_gravity()
        # Sleep for a second until the next reading.
        #time.sleep(1)
finally:
        GPIO.cleanup()








