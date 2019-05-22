from RPi import GPIO
from time import sleep
import logging
import sys
import time
import statistics as stats
import math
import numpy
import csv
import datetime as dt
sys.path.append('/home/pi/.local/lib/python3.5/site-packages')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Ellipse
sys.path.append('/home/pi/Adafruit_Python_BNO055')
from Adafruit_BNO055 import BNO055
from multiprocessing import Process, Queue

# this function controls the graphing of the program
def newfig(fig,q):
    while True:
        
        # making sure that only the most recent addition to the queue is used
        loclist=q.get(block=True)
        if q.empty()==True:
            saveloclist=loclist[0:3]
            figxerror=loclist[3]
            figyerror=loclist[4]
            figzerror=loclist[5]
        while not q.empty():
            saveloclist=loclist[0:3]
            figxerror=loclist[3]
            figyerror=loclist[4]
            figzerror=loclist[5]
            loclist=q.get(block=True)
            
        # initializing plotting
        line1, = ax.plot(saveloclist[0],saveloclist[1],'-r')
        line2, =ax2.plot(saveloclist[0],saveloclist[2],'-r')
        
        ax.set_title('x vs y')
        ax2.set_title('x vs z')
        ax.axis('equal')
        ax2.axis('equal')
        
        # checking if an ellipse exists, if it does not, create one
        # if it does exist, delete the old one and create a new one
        try:
            ellipse
        except NameError:
            ellipse=Ellipse((saveloclist[0][-1],saveloclist[1][-1]),figxerror,figyerror,0)
            ellipse.set_clip_box(ax.bbox)
            ellipse.set_alpha(.44)
            ax.add_artist(ellipse)
        else:
            ellipse.remove()
            ellipse=Ellipse((saveloclist[0][-1],saveloclist[1][-1]),figxerror,figyerror,0)
            ellipse.set_clip_box(ax.bbox)
            ellipse.set_alpha(.44)
            ax.add_artist(ellipse)
            
        try:
            ellipse2
        except NameError:
            ellipse2=Ellipse((saveloclist[0][-1],saveloclist[2][-1]),figxerror,figzerror,0)
            ellipse2.set_clip_box(ax2.bbox)
            ellipse2.set_alpha(.44)
            ax2.add_artist(ellipse2)
        else:
            ellipse2.remove()
            ellipse2=Ellipse((saveloclist[0][-1],saveloclist[2][-1]),figxerror,figzerror,0)
            ellipse2.set_clip_box(ax2.bbox)
            ellipse2.set_alpha(.44)
            ax2.add_artist(ellipse2)
            
        fig.canvas.draw()
        fig.canvas.flush_events()
       
    
# this function controls the reading of the BNO055 and assigns a
# distance traveled for a given step    
def readbno(q2,headingoffset, samplerate,errormeanheadingrad,errormeanpitchrad):    
    global clkState
    global dtState
    global clkLastState
    global headinglist
    global rolllist
    global pitchlist
    global counter
    global countermax
    global nforspeed
    clkState = GPIO.input(clk)
    dtState = GPIO.input(dt)
    
    # Read the Euler angles for heading, roll, pitch (all in degrees).
    #if time.time()-sampletimevar>samplerate:
    heading, pitch, roll = bno.read_euler()
    
    # correct for error in BNO055 code which inverts the heading if
    # roll angle is greater than 135 degrees
    if abs(roll) >135:
        heading=heading-180
        
    # making sure that all angles within the average list are of the same
    # sign. if they are not (as it passes from 360 to 0 degrees) the averaged
    # value will not reflect the true heading
    if len(headinglist)>0 and headinglist[0]>180 and heading<90:
        heading=heading+360
    elif len(headinglist)>0 and headinglist[0]<180 and heading>270:
        heading=heading-360
#        heading=0
#        roll=0
#        pitch=0
    if len(headinglist)<30:
        nforspeed=1
        addtolist=0
        headinglist.append(heading)
        rolllist.append(roll)
        pitchlist.append(pitch)
    elif len(headinglist)>100:
        nforspeed=+nforspeed
        headinglist=headinglist[0::nforspeed]
        rolllist=rolllist[0::nforspeed]
        pitchlist=pitchlist[0::nforspeed]
    if nforspeed>1:
        if addtolist==nforspeed-1:
            headinglist.append(heading)
            rolllist.append(roll)
            pitchlist.append(pitch)
            addtolist=0
        elif addtolist<nforspeed:
            addtolist=+addtolist
    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    #sys, gyro, accel, mag = bno.get_calibration_status()
    #sampletimevar=time.time()
        
    # Making sure that the encoder only counts when the encoder is turned in
    # in the positive dirrection
    if clkState != clkLastState:
        if dtState != clkState:
            counter += 1
        else:
            counter -= 1
        if counter>countermax:
            countermax=counter
            print(countermax)
            
            #print(len(headinglist))
            meanheading=stats.mean(headinglist)-headingoffset
            meanroll=stats.mean(rolllist)
            meanpitch=stats.mean(pitchlist)
            
            q2.put([meanheading,meanroll,meanpitch])
            
            headinglist.clear()
            rolllist.clear()
            pitchlist.clear()
        
        clkLastState = clkState
            
            
        # Print everything out.
#        if time.time()-timevar>1:
#            print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
#                  heading-headingoffset, roll, pitch, sys, gyro, accel, mag))
            #print(pitchlist)
#            timevar=time.time()
        #ani=animation.FuncAnimation(fig,

def mathfun(q,q2):
    global xloccurrent
    global yloccurrent
    global zloccurrent
    global xerror
    global yerror
    global zerror
    while True:
        
        # making sure that only the most recent addition to the queue is used
        loclist2=q2.get(block=True)
        meanheadingm=loclist2[0]
        meanrollm=loclist2[1]
        meanpitchm=loclist2[2]
        
        
        

    #                print('Mean Heading={0:0.2F} Mean Roll={1:0.2F} Mean Pitch={2:0.2F}'.format(
    #                 meanheading, meanroll, meanpitch))         
        meanheadingrad=meanheadingm*pi/180
        meanpitchrad=meanpitchm*pi/180
        
        # updates the x y and z locations based on trig using dead reckoning
        xloccurrent=xloccurrent+distanceperencoder*math.cos(meanheadingrad)*math.cos(meanpitchrad)
        yloccurrent=yloccurrent-distanceperencoder*math.sin(meanheadingrad)*math.cos(meanpitchrad)
        zloccurrent=zloccurrent+distanceperencoder*math.sin(meanpitchrad)
        print('Xloc={0:0.2F} Yloc={1:0.2F} Zloc={2:0.2F}'.format(
         xloccurrent,yloccurrent,zloccurrent))
        xloclist.append(xloccurrent)
        yloclist.append(yloccurrent)
        zloclist.append(zloccurrent)
        
        # determines error according to error propagation formula. assuming worst
        # case error
        xerror=xerror+math.sqrt((math.cos(meanheadingrad)*math.cos(meanpitchrad)*disterror)**2+
                        (distanceperencoder*math.sin(meanheadingrad)*math.cos(meanpitchrad)*errormeanheadingrad)**2+
                        (distanceperencoder*math.cos(meanheadingrad)*math.sin(meanpitchrad)*errormeanpitchrad)**2)
        yerror=yerror+math.sqrt((math.sin(meanheadingrad)*math.cos(meanpitchrad)*disterror)**2+
                        (distanceperencoder*math.cos(meanheadingrad)*math.cos(meanpitchrad)*errormeanheadingrad)**2+
                        (distanceperencoder*math.sin(meanheadingrad)*math.sin(meanpitchrad)*errormeanpitchrad)**2)
        zerror=zerror+math.sqrt((math.sin(meanpitchrad)*disterror)**2+
                                (distanceperencoder*math.cos(meanpitchrad)*errormeanpitchrad)**2)
        
        
#        print('Xerror={0:0.2F} Yerror={1:0.2F} Zerror={2:0.2F}'.format(
#         xerror,yerror,zerror))
        
        # imputing information into the queue
        q.put([xloclist,yloclist,zloclist,xerror,yerror,zerror])


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
distanceperencoder=1.035*pi/40
xloccurrent=0
yloccurrent=0
zloccurrent=0
xloclist=[0]
yloclist=[0]
zloclist=[0]
counter = 0
countermax=0
headinglist=[]
rolllist=[]
pitchlist=[]
timevar=time.time()
sampletimevar=time.time()
xerror=0
yerror=0
zerror=0
disterror=distanceperencoder*.25/12
errormeanpitchrad=4*pi/180
errormeanheadingrad=4*pi/180
nforspeed=0



#Create figure for plotting
plt.ion()
fig = plt.figure(figsize=(5,8))
ax=plt.subplot2grid((8,5),(0,0), colspan=5,rowspan=5)
ax2=plt.subplot2grid((8,5),(6,0), colspan=5,rowspan=2)

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
#    headingoffset=0
    print(headingoffset)
    
    clkState = GPIO.input(clk)
    dtState = GPIO.input(dt)
        
    q=Queue()
    p=Process(target=newfig,args=(fig, q))
    p.start()
    
    q2=Queue()
    p2=Process(target=mathfun,args=(q, q2))
    p2.start()
    
    while True:
        readbno(q2,headingoffset,samplerate,errormeanheadingrad,errormeanpitchrad)
                
finally:
    GPIO.cleanup()
    p.terminate()
    p.join()
    p2.terminate()
    p2.join()
    






