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
from multiprocessing import Process, Queue

# this function controls the graphing of the program
def newfig(fig,q):
    while True:
        loclist=q.get(block=True)
        if q.empty()==True:
            savelist=loclist
        while not q.empty():
            savelist=loclist
            loclist=q.get(block=True)
        line1, = ax.plot(savelist[0],savelist[1],'-r')
        ax.set_title(u"Z={}".format(round(savelist[2],3)))
        
        fig.canvas.draw()
        fig.canvas.flush_events()
       
    
# this function controls the reading of the BNO055 and assigns a distance traveled for a given step    
def readbno(q,headingoffset, samplerate):    
        global timevar
        global xloclist
        global yloclist
        global zloclist
        global fig
        global xloccurrent
        global yloccurrent
        global zloccurrent
        global clkState
        global dtState
        global clkLastState
        global headinglist
        global rolllist
        global pitchlist
        global lastencodertime
        global sampletimevar
        clkState = GPIO.input(clk)
        dtState = GPIO.input(dt)
        
        
        # Read the Euler angles for heading, roll, pitch (all in degrees).
        if time.time()-sampletimevar>samplerate:
            heading, pitch, roll = bno.read_euler()
            if abs(roll) >135:
                heading=heading-180
            headinglist.append(heading)
            rolllist.append(roll)
            pitchlist.append(pitch)
            # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
            sys, gyro, accel, mag = bno.get_calibration_status()
            sampletimevar=time.time()
            
            
        if time.time()-timevar>1:#clkState != clkLastState:
            meanheading=stats.mean(headinglist)-headingoffset
            meanroll=stats.mean(rolllist)
            meanpitch=stats.mean(pitchlist)
            headinglist.clear()
            rolllist.clear()
            pitchlist.clear()
            
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
            
            q.put([xloclist,yloclist,zloclist[-1]])
            
            clkLastState = clkState
            
            lastencodertime=time.time()
            
            
        # Print everything out.
        if time.time()-timevar>1:
            print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
                  heading-headingoffset, roll, pitch, sys, gyro, accel, mag))
            #print(pitchlist)
            timevar=time.time()
        #ani=animation.FuncAnimation(fig,





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
timevar=time.time()
sampletimevar=time.time()
lastencodertime=0



#Create figure for plotting
plt.ion()
fig = plt.figure()
ax=fig.add_subplot(111)


#getting calibration data from calibration file
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
#bno._operation_mode(0X01)

while True:
    try:
        if not bno.begin(0X0B):
            raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
        status,self_test,error = bno.get_system_status()
        break
    except Exception as e:
        print("Got error: {}".format(e))
        print("sleeping .5s before retrying")
        time.sleep(.5)

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



print('Reading BNO055 data, press Ctrl-C to quit...')

try:
    #setting calibration of the BNO055
    bno.set_calibration(f)
    
    
    calibstart=time.time()
    while time.time()-calibstart<calibrationtime:
        heading, roll, pitch = bno.read_euler()
    headingoffset=heading
    print(headingoffset)
    
    clkState = GPIO.input(clk)
    dtState = GPIO.input(dt)
        
    q=Queue()
    p=Process(target=newfig,args=(fig, q))
    p.start()
    while True:
        readbno(q,headingoffset,samplerate)
                
finally:
    GPIO.cleanup()
    p.terminate()
    p.join()
    print('done')














