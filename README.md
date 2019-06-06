# Raspi-tunnel-bot
Necessary Dependencies
		“Sudo apt-get install -y build-essential python-dev python-smbus python-pip git”
	Download BNO055 python module
cd ~
git clone https://github.com/adafruit/Adafruit_Python_BNO055.git
cd Adafruit_Python_BNO055
sudo python setup.py install
	Disable kernel serial port to allow for connecting to BNO055 as serial UART
		Sudo raspi-config 
		Go to interface options ---> serial ---> disable
	Get GPIO from adafruit
		git clone https://github.com/adafruit/Adafruit_Python_GPIO.git
cd Adafruit_Python_GPIO
sudo python setup.py install
Change give code for BNO055 to reference correct dirrectory
	sys.path.append(‘/home/pi/Adafruit_Python_BNO055’) to simpletest.py
	
	Import sys
sys.path.append(/home/pi/Adafruit_Python_GPIO’) 
to line 222 of BNO055.py
Set static IP
	In /etc/dhcpcd.conf
	Add lines
	interface wlan0
static ip_address=192.168.##.##
static routers=192.168.##.##
static domain_name_servers=##
As stated by https://electrondust.com/2017/11/25/setting-raspberry-pi-wifi-static-ip-raspbian-stretch-lite/
 
To get ip_adress type “route -ne”. This will give you the default gateway. This number will be exactly the static routers. The ip_adress will be this number with the very last digit changed. 
 
To get domain_name servers, type “cat /etc/resolv.conf”. Then type all numbers into this field with each number being separated by a single space.
 
	Set up live plotting for x and y coordinates
This causes some issues with the program as it is a slow process. I will need to look into multiprocessing so that the plotting is a separate problem.
	Set up email capabilites
		Sudo apt-get ssmtp
		Sudo apt-get mailutils
