#!/usr/bin/env python
from time import sleep
from threading import Thread #Library used in making threads
import time
import serial
import pynmeagps #Python GPS library
import RPi.GPIO as GPIO
#import math
from picamera import PiCamera
from subprocess import call


#NOTE: Use ls /dev/tty* in terminal to detect Arduino and radio port!

#Globals
arduino_data = ''
GPS_data = ''
GPSlat = 0.0
GPSlong = 0.0
GPSlatStart = 0.0 #Starting point latitude
GPSlongStart = 0.0 #Starting point longitude
xRadEllipse = .000417 #X axis radius of ellipse travel boundary - For ellipse bound mode
yRadEllipse = .000139 #Y axis radius of ellipse travel boundary
#Variables for Image Transmission
packet_list = []
imgCount = 0
imgFlag = 0 #Not sure if being used


#Packet Class
class packet():
    def __init__(self, pcktNum, imgId, pcktTotal, content):
        self.pcktNum = pcktNum
        self.imgId = imgId
        self.pcktTotal = pcktTotal
        self.content = content

arduino = serial.Serial( #Set up Arduino port
    port='/dev/ttyACM0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

radioModule =serial.Serial( #Set up radio module port
    port='/dev/ttyUSB1',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

stream = serial.Serial('/dev/ttyUSB1',9600) #Set up GPS Port
nmr = pynmeagps.NMEAReader(stream) #Set up NMEA reader

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

def arduinoLoop(): #Code from Pi_Arduino_Test
    print('Arduino Thread Begin\n')
    global arduino_data
    arduino_buffer = '' #Buffer used to hold sensor data while fetching
    while 1:
        
        firstLine = arduino.readline() #Read serial lines until first line of package is found
        if firstLine[0:11] == b'BMP180 Temp':
            arduino_buffer += str(firstLine) + '\n' #Add first line of package to buffer
            for line in range(10): #Add rest of package lines to buffer
                currentLine = arduino.readline() #Read a line of serial text from Arduino
                #radioModule.write(bytes((str(x)+ '\n'), 'utf-8')) #Turn serial data into string and send to radio module
                myFile = open("HABIVSensorData.txt",'a') #Open text file with append permission - add to existing content
                localTime = time.strftime("%m/%d/%Y %H:%M:%S", time.localtime()) #Fetches current time and turns into string
                myFile.write(localTime + ' ' + str(currentLine) + '\n') #Write serial data (converted to string) to text file
                myFile.close() #Close file being written to (save the data)
                arduino_buffer += str(currentLine) + '\n'
            
            arduino_data = arduino_buffer #Pass data from buffer to global
            arduino_buffer = '' #Reset buffer after data passed to global
        
            #print(arduino_data) #Print serial data (automatically converted to string) to screen

def piLoop():
    print('GPS Thread Begin\n')
    global GPS_data, GPSlat, GPSlong
    GPStime = ''
    GPSlat = GPSlong = GPSalt = GPSsats = 0
    GPS_F=open("HABIVGPSData.txt","w") #Open text file with write permission - REPLACES EXISTING CONTENT
    GPS_F.write('Start of GPS Data\n') #Write header text to text file
    while 1:
        GPStime,GPSsats,GPSlat,GPSlong,GPSalt = read_GGA() #Fetch GPS data
        LOCtimeSTR = time.strftime("%m/%d/%Y %H:%M:%S", time.localtime()) #Fetches current time and turns into string

        GPS_F.close()    
        GPS_F=open("HABIVGPSData.txt",'a')
        GPS_F.write(str("Time {} UTC {} Lat {:10.8f} Long {:10.8f} Altitude {:8.1f} #Sats {:2d}".format(LOCtimeSTR, GPStime, GPSlat, GPSlong, GPSalt, GPSsats))+'\n')
        
        GPS_data = str("LAT: {:10.6f} \nLON: {:10.6f} \nALT: {:8.1f} m\n".format(GPSlat, GPSlong, GPSalt))
        
        #print(GPS_data)
    
def radioLoop():
    print('Radio Thread Begin\n')
    global arduino_data, GPSlat, GPSlong, packet_list, imgCount, imgFlag
    outF=open("HABIVRadioData.txt","w") #Open text file with write permission - REPLACES EXISTING CONTENT
    outF.write('Start of Transmitted Radio Data\n') #Write header text to text file
    while 1:
        LOCtimeSTR = time.strftime("%m/%d/%Y %H:%M:%S", time.localtime()) #Fetches current time and turns into string
        radioModule.write(bytes((arduino_data + GPS_data), 'utf-8'))
        #TRANSMITTING IMAGE: Consider moving to separate thread
        if not packet_list:
            print("No Image Data to send")
        else:
            imgId = 'IMG' + str(imgCount) + ' ' #the value will chane for each new image
            imgIdHdr = bytes(imgId, 'utf-8')
            tarPckt = packet_list[0].content
            #pcktX = b''.join([imgIdHdr, tarPckt])
            pcktX = imgIdHdr + tarPckt
            print("PcktX: " + str(pcktX))
            #pcktToSend = imgIdHdr + tarPckt
            #imgIdHdr += tarPckt
            print("Sending packet Data")
            radioModule.write(pcktX)
            packet_list.pop(0)
            
            #Check to see if list is now empty
            if not packet_list:
                    imgCount+=1
                    #Potentially insert a delay for other thread to recognize that the list is empty
        print (arduino_data)
        print (GPS_data)
#         print (GPSlat)
#         print (GPSlong)
        outF.close() #Close text file
        outF=open("HABIVRadioData.txt",'a')
        outF.write(LOCtimeSTR + ' ' + arduino_data + GPS_data +'\n')
        #sleep(5)
        
def ETSLoop():
    print('ETS Thread Begin\n')
    global GPSlat, GPSlong, latBoundN, latBoundS, longBoundE, longBoundW, xRadEllipse, yRadEllipse
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(3, GPIO.OUT) #Uses physical board numbers. = GPIO2
    GPIO.output(3, GPIO.LOW)
    
    while ((GPSlat == 0.0) & (GPSlong == 0.0)):
         print("Waiting for valid GPS coords to record start point!")
         sleep(1)
    GPSlatStart = GPSlat #Starting point latitude
    GPSlongStart = GPSlong #Starting point longitude
    print("Start point GPS coords recorded!")
    print("Starting LAT: " + str(GPSlatStart))
    print("Starting LON: " + str(GPSlongStart))
    
    while 1:
#         print (GPSlat) #Debug statements
#         print (GPSlong)

        # Inequality (eqLeftSide > eqRightSide) used to determine if the current payload location is outside of the ellipseWPI
        eqLeftSide = (yRadEllipse ** 2) * ((GPSlong - GPSlongStart) ** 2) + (xRadEllipse ** 2) * ((GPSlat - GPSlatStart) ** 2)
        eqRightSide = (xRadEllipse ** 2) * (yRadEllipse ** 2)
        print("Eq Left Side:  " + str(eqLeftSide))
        print("Eq Right Side: " + str(eqRightSide))
        if (eqLeftSide > eqRightSide):
            GPIO.output(3, GPIO.HIGH)
            print("ETS SYSTEM TRIGGERED")
        sleep(1)

def captureImage():
    global packet_list
    while 1:
        if not packet_list: #if list is empty
            camera.capture("test.jpeg")
            print("New image has been captured")
            #Encode newly captured image
            call(["./ssdv", "-e", "-c", "test01", "-i", "1", "test.jpeg", "output.bin"]) #Need to update callsign to be my own
            
            #Variable to identify each packet
            it = 1
            
            #Open and split the output.bin file to be transmitted
            with open("output.bin", "rb") as f:
                #Check to see if EOF was reached
                fContent = f.read(256) #Reads the first 256 bytes(first SSDV packet)
                while fContent!= b"":
                    newPacket = packet(it, 1, 120, fContent) #need to change the 120
                    fContent = f.read(256) #Each SSDV packet is 256 bytes in length
                    packet_list.append(newPacket)
                    it+=1



# Create threads
arduino_thread = Thread(target = arduinoLoop)
pi_thread = Thread(target = piLoop)
transmit_thread = Thread(target = radioLoop)
ets_thread = Thread(target = ETSLoop)
image_capture_thread = Thread(target = captureImage)

#Initializing PiCamera
camera = PiCamera()
camera.resolution = (1280, 720)
camera.vflip = True #Consider changing this. Might not be necessary
time.sleep(2)

# Start threads
arduino_thread.start()
pi_thread.start()
transmit_thread.start()
ets_thread.start()
image_capture_thread.start()
