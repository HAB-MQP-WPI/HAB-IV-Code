#https://www.circuitbasics.com/raspberry-pi-ds18b20-temperature-sensor-tutorial/

#python3 -m pip install --upgrade pynmeagps
#^ Used to update pynmeagps if necessary

#NOTE: Use ls /dev/tty* in terminal to detect Arduino and radio port!

import serial #Python serial communication library (PySerial)
import time #Python time module
import pynmeagps #Python GPS library
 
stream = serial.Serial('/dev/ttyUSB1',9600) #USB GPS device setup - 9600 baud rate
nmr = pynmeagps.NMEAReader(stream)
GPStime = '' #GPS time
GPSlat = 0 #GPS latitude
GPSlong = 0 #GPS longitude
GPSalt = 0 #GPS altitude
GPSsats = 0 #GPS # of satellites detected
 
def read_GGA(): #Reads a GGA message from GPS module and updates GPS data globals
    for x in range(24): #Wait for GPS to output a GGA message 
        try:
            (raw_data, parsed_dat) = nmr.read() #Read GPS message output
            if parsed_dat.msgID =="GGA": #Once a GGA message is received, parse and use data
                GPSlat1 = 0 #Lat, long, and alt will return 0 if data is not valid
                GPSlong1 = 0
                GPSalt1 = 0
                if parsed_dat.quality > 0: #Update GPS coords if data is valid
                    GPSlat1 = parsed_dat.lat
                    GPSlong1 = parsed_dat.lon
                    GPSalt1 = parsed_dat.alt
                return parsed_dat.time, parsed_dat.numSV, GPSlat1, GPSlong1, GPSalt1 #Return values upon successful data update
        except Exception as e:
            print(e)
    return 0,0,0,0,0 #Return 0 for data values in error case
 
while True:
    GPStime, GPSsats, GPSlat, GPSlong, GPSalt = read_GGA() #Fetch GPS data
    print(GPStime)
    LOCtimeSTR = time.strftime("%m/%d/%Y %H:%M:%S", time.localtime()) #Get local time and turn into string
 
    #Print GPS data
    print("Time {} UTC {} Lat {:10.8f} Long {:10.8f} Altitude {:8.1f} #Sats {:2d}".format(LOCtimeSTR, GPStime, GPSlat, GPSlong, GPSalt, GPSsats))
