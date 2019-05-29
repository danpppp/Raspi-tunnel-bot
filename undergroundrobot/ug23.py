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
from multiprocessing import Process, Queue, Pipe
import datetime
from email.mime.multipart  import MIMEMultipart
from email.mime.text import MIMEText
import smtplib
import mimetypes
import email
import email.mime.application
import sys
 
 
 # Create a text/plain message
msg = email.mime.multipart.MIMEMultipart()
msg['Subject'] = 'Test 2'
msg['From'] = 'undergroundtest@gmail.com'
recipients=['danpekin@gmail.com','hstapletonmcms@gmail.com']

 
# The main body is just another attachment
body = email.mime.text.MIMEText("""2 turns""")
msg.attach(body)

 




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
    global quat0list
    global quatxlist
    global quatylist
    global quatzlist
    global counter
    global countermax
    global nforspeed
    global firstiteration
    global meanquat0
    global meanquatx
    global meanquaty
    global meanquatz
    global addtolist
    global timevar
    #global firstpast135
    global correctheading
    global headingcorrection
    clkState = GPIO.input(clk)
    dtState = GPIO.input(dt)
    
    if clkState != clkLastState and firstiteration==0:
        if dtState != clkState:
            counter += 1
        else:
            counter -= 1
        if counter>countermax:
            countermax=counter
            #print(countermax)
            #print("alsdkjhasldkjfalksdjf")
            q2.put([meanquat0,meanquatx,meanquaty,meanquatz])
        
        clkLastState = clkState
        
    quat0,quatz,quaty,quatx=bno.read_quaternion()

    quat0list.append(quat0)
    quatzlist.append(quatz)
    quatylist.append(quaty)
    quatxlist.append(quatx)
        
    firstiteration=1
    
    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    sys, gyro, accel, mag = bno.get_calibration_status()
        
    # Making sure that the encoder only counts when the encoder is turned in
    # in the positive dirrection
    if clkState != clkLastState:
        if dtState != clkState:
            counter += 1
        else:
            counter -= 1
        if counter>countermax:
            countermax=counter
            #print(countermax)
            
            if len(quat0list)>200:
                nforquickness=round(len(quat0list)/100)
                quat0list=quat0list[0::nforquickness]
                quatxlist=quatxlist[0::nforquickness]
                quatylist=quatylist[0::nforquickness]
             
            meanquat0=stats.mean(quat0list)
            meanquatx=stats.mean(quatxlist)
            meanquaty=stats.mean(quatylist)
            meanquatz=stats.mean(quatzlist)
            stdquat0=stats.stdev(quat0list)
            stdquatx=stats.stdev(quatxlist)
            stdquaty=stats.stdev(quatylist)
            stdquatz=stats.stdev(quatzlist)
            
            q2.put([meanquat0,meanquatx,meanquaty,meanquatz,stdquat0,stdquatx,stdquaty,stdquatz])
            
            quat0list.clear()
            quatxlist.clear()
            quatylist.clear()
            quatzlist.clear()
            
            firstiteration=0
        
        clkLastState = clkState
            
            
    # Print everything out.
    if time.time()-timevar>1:
        
#        print(quat0)
#        print(quatx)
#        print(quaty)
#        print(quatz)
        xvec=1-2*(quaty**2+quatz**2)
        yvec=2*(quatx*quaty+quatz*quat0)
        zvec=2*(quatx*quatz-quaty*quat0)

        xvecnorm=xvec*math.cos(headingoffset*pi/180)-yvec*math.sin(headingoffset*pi/180)
        yvecnorm=xvec*math.sin(headingoffset*pi/180)+yvec*math.cos(headingoffset*pi/180)



        print('xvec={0:0.2F} yvec={1:0.4F} zvec={2:0.2F}'.format(
              xvecnorm, yvecnorm, zvec))
        timevar=time.time()

def mathfun(q,q2,q3):
    global xloccurrent
    global yloccurrent
    global zloccurrent
    global xloclist
    global yloclist
    global zloclist
    global xerror
    global yerror
    global zerror
    global quat0listm
    global quatxlistm
    global quatylistm
    global quatzlistm
    global stdquat0list
    global stdquatxlist
    global stdquatylist
    global stdquatzlist
    while True:
        
        # making sure that only the most recent addition to the queue is used
        loclist2=q2.get(block=True)
        meanquat0m=loclist2[0]
        meanquatxm=loclist2[1]
        meanquatym=loclist2[2]
        meanquatzm=loclist2[3]
        
        stdquat0m=loclist2[4]
        stdquatxm=loclist2[5]
        stdquatym=loclist2[6]
        stdquatzm=loclist2[7]
        
        xvec=1-2*(meanquatym**2+meanquatzm**2)
        yvec=2*(meanquatxm*meanquatym+meanquatzm*meanquat0m)
        zvec=2*(meanquatxm*meanquatzm-meanquatym*meanquat0m)

        xvecnorm=xvec*math.cos(headingoffset*pi/180)-yvec*math.sin(headingoffset*pi/180)
        yvecnorm=xvec*math.sin(headingoffset*pi/180)+yvec*math.cos(headingoffset*pi/180)
        
        # updates the x y and z locations based on trig using dead reckoning
        xloccurrent=xloccurrent+distanceperencoder*xvecnorm
        yloccurrent=yloccurrent+distanceperencoder*yvecnorm
        zloccurrent=zloccurrent+distanceperencoder*zvec
        print('Xloc={0:0.2F} Yloc={1:0.2F} Zloc={2:0.2F}'.format(
         xloccurrent,yloccurrent,zloccurrent))
        
        xloclist.append(xloccurrent)
        yloclist.append(yloccurrent)
        zloclist.append(zloccurrent)
        
        # determines error according to error propagation formula. assuming worst
        # case error
        xerror=0
        yerror=0
        zerror=0
        
        xerrorlist.append(xerror)
        yerrorlist.append(yerror)
        zerrorlist.append(zerror)
        
        quat0listm.append(meanquat0m)
        quatxlistm.append(meanquatxm)
        quatylistm.append(meanquatym)
        quatzlistm.append(meanquatzm)
        
        stdquat0list.append(stdquat0m)
        stdquatxlist.append(stdquatxm)
        stdquatylist.append(stdquatym)
        stdquatzlist.append(stdquatzm)
        
        print('Xerror={0:0.2F} Yerror={1:0.2F} Zerror={2:0.2F}'.format(
         xerror,yerror,zerror))
        
        # imputing information into the queue
        q.put([xloclist,yloclist,zloclist,xerror,yerror,zerror])
        q3.put([xloclist,yloclist,zloclist,xerrorlist,yerrorlist,zerrorlist,
                quat0listm,quatxlistm,quatylistm,quatzlistm,
                stdquat0list,stdquatxlist,stdquatylist,stdquatzlist])
#        mathpipe.close()
        
#pin setups
bno = BNO055.BNO055(serial_port='/dev/serial0',rst=18)
clk = 17
dt = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup(clk, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dt, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(21,GPIO.IN,pull_up_down=GPIO.PUD_UP)
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
quat0list=[]
quatxlist=[]
quatylist=[]
quatzlist=[]
timevar=time.time()
sampletimevar=time.time()
xerror=0
yerror=0
zerror=0
xerrorlist=[0]
yerrorlist=[0]
zerrorlist=[0]
quat0listm=[0]
quatxlistm=[0]
quatylistm=[0]
quatzlistm=[0]
stdquat0list=[0]
stdquatxlist=[0]
stdquatylist=[0]
stdquatzlist=[0]
disterror=distanceperencoder*3/60
errormeanpitchrad=3*pi/180
errormeanheadingrad=3*pi/180
nforspeed=0
firstiteration=1
lengthofpipevar=0
firstpast135=0




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
    
    q3=Queue()
    q2=Queue()
    p2=Process(target=mathfun,args=(q, q2, q3))
    p2.start()
    
    while True:
        readbno(q2,headingoffset,samplerate,errormeanheadingrad,errormeanpitchrad)
        input_state = GPIO.input(21)
        if input_state==False:
            print('button press')
            loclist=q3.get(block=True)
            if q3.empty()==True:
                xyzpos=loclist[0:3]
                xerrorlist=loclist[3]
                yerrorlist=loclist[4]
                zerrorlist=loclist[5]
                quat0=loclist[6]
                quatx=loclist[7]
                quaty=loclist[8]
                quatz=loclist[9]
                stdq0=loclist[10]
                stdqx=loclist[11]
                stdqy=loclist[12]
                stdqz=loclist[13]
            while not q3.empty():
                xyzpos=loclist[0:3]
                xerrorlist=loclist[3]
                yerrorlist=loclist[4]
                zerrorlist=loclist[5]
                quat0=loclist[6]
                quatx=loclist[7]
                quaty=loclist[8]
                quatz=loclist[9]
                stdq0=loclist[10]
                stdqx=loclist[11]
                stdqy=loclist[12]
                stdqz=loclist[13]
                loclist=q3.get(block=True)
            
            deletingfile=open('testdata.csv',"w+")
            deletingfile.close()
            with open('testdata.csv','w') as datfile:
                wr=csv.writer(datfile, quoting=csv.QUOTE_ALL)
                wr.writerow(xyzpos[0])
                wr.writerow(xyzpos[1])
                wr.writerow(xyzpos[2])
                wr.writerow(xerrorlist)
                wr.writerow(yerrorlist)
                wr.writerow(zerrorlist)
                wr.writerow(quat0)
                wr.writerow(quatx)
                wr.writerow(quaty)
                wr.writerow(quatz)
                wr.writerow(stdq0)
                wr.writerow(stdqx)
                wr.writerow(stdqy)
                wr.writerow(stdqz)
            
            
            
            #directory=sys.argv[1]
            directory = 'testdata.csv'
             
            # Split de directory into fields separated by / to substract filename
            spl_dir=directory.split('/')
             
            # We attach the name of the file to filename by taking the last
            # position of the fragmented string, which is, indeed, the name
            # of the file we've selected
             
            filename=spl_dir[len(spl_dir)-1]
             
            # We'll do the same but this time to extract the file format (pdf, epub, docx...)
            spl_type=directory.split('.')
            type=spl_type[len(spl_type)-1]
             
            fp=open(directory,'rb')
            att = email.mime.application.MIMEApplication(fp.read(),_subtype=type)
            fp.close()
            att.add_header('Content-Disposition','attachment',filename=filename)
            msg.attach(att)
            
            msg['To'] = ", ".join(recipients)
            s = smtplib.SMTP('smtp.gmail.com:587')
            s.starttls()
            s.login('undergroundrobottest@gmail.com','Hawkeslab')
            s.sendmail('undergroundrobottest@gmail.com',recipients, msg.as_string())
            s.quit()
            print('email sent')
            
            p.terminate()
            p.join()
            p2.terminate()
            p2.join()
            sys.exit()
finally:
    GPIO.cleanup()
    p.terminate()
    p.join()
    p2.terminate()
    p2.join()
    
















