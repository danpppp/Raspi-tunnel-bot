from RPi import GPIO
from time import sleep
import logging
import sys
import time
import statistics as stats
import math
import csv
import datetime as dt
sys.path.append('/home/pi/.local/lib/python3.5/site-packages')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
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
samplerate=0
calibrationtime=1.5
pi=math.pi
distanceperencoder=1.08*pi/40
xloccurrent=0
yloccurrent=0
zloccurrent=0
xloclist=[0]
yloclist=[0]
zloclist=[0]
counter = 0
headinglist=[]
rolllist=[]
pitchlist=[]

#Create figure for plotting
plt.ion()
fig = plt.figure()
ax=fig.add_subplot(111)
def newfig(fig,xloclist,yloclist):
    line1, = ax.plot(xloclist,yloclist,'-r')
    fig.canvas.draw()
    fig.canvas.flush_events()

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
    bno.set_calibration(f)
    bno.set_mode(0X0C)
    
    while time.time()-timevar<calibrationtime:
        heading, roll, pitch = bno.read_euler()
    headingoffset=heading
    print(headingoffset)
    
    while True:
        clkState = GPIO.input(clk)
        dtState = GPIO.input(dt)
        if clkState != clkLastState:
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
            
            xloccurrent=xloccurrent+distanceperencoder*math.cos(meanheading*pi/180)*math.cos(meanpitch*pi/180)
            yloccurrent=yloccurrent+distanceperencoder*math.sin(meanheading*pi/180)*math.cos(meanpitch*pi/180)
            zloccurrent=zloccurrent+distanceperencoder*math.sin(meanpitch*pi/180)
            print('Xloc={0:0.2F} Yloc={1:0.2F} Zloc={2:0.2F}'.format(
             xloccurrent,yloccurrent,zloccurrent))
            xloclist.append(xloccurrent)
            yloclist.append(yloccurrent)
            zloclist.append(zloccurrent)
            #print(xloclist,yloclist,zloclist)
            newfig(fig,xloclist,yloclist)
            
            
            clkLastState = clkState
            
            lastencodertime=time.time()
            
    
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
        if time.time()-timevar>1:
#            print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
#              heading-headingoffset, roll, pitch, sys, gyro, accel, mag))
            #print(pitchlist)
            timevar=time.time()
        #ani=animation.FuncAnimation(fig,
finally:
        GPIO.cleanup()









