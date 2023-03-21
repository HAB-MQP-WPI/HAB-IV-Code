#!/usr/bin/env python
from time import sleep
from threading import Thread  # Library used in making threads
import time
import serial  # Used with serial devices (Arduino, radio module, GPS)
import pynmeagps  # Python GPS library
import RPi.GPIO as GPIO  # Library for using Pi GPIO pins
from picamera import PiCamera  # Used for interfacing Pi with camera
from subprocess import call
from csv import writer  # Used in writing to CSV files (column headers)
from csv import DictWriter  # Used in writing dictionaries to CSV files (CSV data entries)

#NOTE: Use ls /dev/tty* in terminal to detect Arduino and radio port!

#Globals
GPSlat = 0.0  # Current latitude (0 until a valid output is made)
GPSlong = 0.0  # Current longitude (0 until a valid output is made)
GPSlatStart = 0.0  # Starting point latitude
GPSlongStart = 0.0  # Starting point longitude
yRadEllipse = .000417  # X axis radius of ellipse travel boundary
xRadEllipse = .000139  # Y axis radius of ellipse travel boundary

#Variables for Image Transmission
packet_list = []
imgCount = 0
imgFlag = 0 #Not sure if being used

etsTrigWait = 1 # How many GPS coordinate transmissions outside ETS boundary needed before activation
etsTrigCount = 0 # Current number of consecutive GPS coordinates outside of ETS boundary

ETS_arm_height = 304.8  # During ascent, height (in meters) at which to arm the ETS system for automatic activation on freefall
# Parachute deploys when the payload descends below 1000ft (304.8m)
# Need to arm the system as low as the parachute can deploy at
ETS_fall_count_threshold = 3  # How many times an appropriate descent speed is recorded in a row
ETS_trigger_velocity = -25  # Velocity in m/s at which to trigger the ETS (a drop is occurring)

# Create data buffer with default values to avoid errors
currDict = {'Date': time.strftime("%m/%d/%Y", time.localtime()),  # Initialize CSV data buffer with some default values
            'Time': time.strftime("%H:%M:%S", time.localtime()),
            'Latitude (DD)': 0,
            'Longitude (DD)': 0,
            'GPS Alt (m)': 0,
            'ETS Triggered': 0,
            'BMP180 Temp (*C)': 0,
            'BMP180 Pressure (Pa)': 0,
            'BMP180 Alt (m)': 0,
            'CO2 (ppm)': 0,
            'OZ (ppb)': 0,
            'CH4 (ppm)': 0,
            'UV Index': 0,
            'One-Wire Temp (*F)': 0,
            'Red Pressure Temp (*C)': 0,
            'Red Pressure Temp (*F)': 0,
            'Red Pressure (mb)': 0,
            'Red Alt (m)': 0,
            'Red Alt Change from Startup (m)': 0
            }

columnTitles = ['Date',  # List of column headers for CSV file
                'Time',
                'Latitude (DD)',
                'Longitude (DD)',
                'GPS Alt (m)',
                'ETS Triggered',
                'BMP180 Temp (*C)',
                'BMP180 Pressure (Pa)',
                'BMP180 Alt (m)',
                'CO2 (ppm)',
                'OZ (ppb)',
                'CH4 (ppm)',
                'UV Index',
                'One-Wire Temp (*F)',
                'Red Pressure Temp (*C)',
                'Red Pressure Temp (*F)',
                'Red Pressure (mb)',
                'Red Alt (m)',
                'Red Alt Change from Startup (m)']

#Packet Class
class packet():
    def __init__(self, pcktNum, imgId, pcktTotal, content):
        self.pcktNum = pcktNum
        self.imgId = imgId
        self.pcktTotal = pcktTotal
        self.content = content

# Serial Devices Setup

# Set up Arduino com port
arduino = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

# Set up radio module com port
radioModule =serial.Serial(
    port='/dev/ttyUSB0',  # If error, try swapping radio and GPS serial port names! Usually works
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

# Set up GPS com port
stream = serial.Serial('/dev/ttyUSB1',9600)
nmr = pynmeagps.NMEAReader(stream)  # Set up NMEA reader - interprets GPS output

def read_GGA():  # Reads a GGA message from GPS module and updates GPS data globals
    for x in range(24):  # Wait for GPS to output a GGA message 
        try:
            (raw_data, parsed_dat) = nmr.read()  # Read GPS message output
            if parsed_dat.msgID =="GGA":  # Once a GGA message is received, parse and use data
                GPSlat1 = 0  # Lat, long, and alt will return 0 if data is not valid
                GPSlong1 = 0
                GPSalt1 = 0
                if parsed_dat.quality > 0:  # Update GPS coords if data is valid
                    GPSlat1 = parsed_dat.lat
                    GPSlong1 = parsed_dat.lon
                    GPSalt1 = parsed_dat.alt
                return parsed_dat.time, parsed_dat.numSV, GPSlat1, GPSlong1, GPSalt1  # Return values upon successful data update
        except Exception as e:
            print(e)
    return 0,0,0,0,0  # Return 0 for data values in error case

def arduinoLoop():  # Handles reception, packaging and sending of sensor data from Arduino
    #Code from Pi_Arduino_Test
    print('Arduino Thread Begin\n')
    arduino_buffer = ''
    csvTitle = "HABIVSensorData_" + str(time.strftime("%m-%d-%Y_%H-%M-%S", time.localtime())) + ".csv"  # Name file with current date and time
    myCSV = open(csvTitle,'a')  # Open text file with append permission - add to existing content
    writer_object = writer(myCSV)
    writer_object.writerow(columnTitles)
    myCSV.close()  # Close file being written to (save the data)
    while 1:
        
        firstLine = arduino.readline().decode('UTF-8')  # Read serial lines until first line of package is found
        if firstLine[0:21] == "ARDUINO SENSOR PACKET":  # If Arduino packet header is read...
            arduino_buffer += str(firstLine)  # Add first line of package to buffer
            
            currDict['Date'] = time.strftime("%m/%d/%Y", time.localtime())  # Fetches current date and turns into string
            currDict['Time'] = time.strftime("%H:%M:%S", time.localtime())  # Fetches current time and turns into string
            
            for line in range(13): #Add rest of package lines to buffer
                currentLine = arduino.readline().decode('UTF-8')  # Read a line of serial text from Arduino
                for i in columnTitles:  # Check that the data description is in the CSV column headers
                    if currentLine.partition(":")[0] == i:
                        currDict[currentLine.partition(":")[0]] = (currentLine.partition(": ")[2]).partition("\r")[0]  # Update dictionary value
                        break
                
            myCSV = open(csvTitle,'a')  # Open text file with append permission - add to existing content

            dictWriter_object = DictWriter(myCSV, fieldnames = columnTitles)  # Write current dictionary data to CSV files, match data to corresponding columns
            dictWriter_object.writerow(currDict)
            myCSV.close()  # Close file being written to (save the data)
            
            packetString = ""  # Initialize packet string to be sent to base station
            index = 0
            for i in columnTitles:  # Make a .csv row to send in text form over radio
                if index > 0:
                    packetString+=','
                packetString+=str(currDict[i])
                index+=1
            packetString+='\n'
            
            radioModule.write(bytes('PAYLOAD DATA PACKET\n' + str(packetString), 'UTF-8'))  # Send the string packet over radio to base station
            print(bytes(str(currDict), 'UTF-8'))
            print(bytes('PAYLOAD DATA PACKET\n' + str(packetString), 'UTF-8'))
        
def piLoop():  # Handles reception of data from GPS module
    print('GPS Thread Begin\n')
    global GPSlat, GPSlong
#     GPStime = ''  # THIS COULD BE USEFUL LATER!!!!!
    GPSlat = GPSlong = GPSalt = GPSsats = 0  # Make GPS values 0 by default
    
    while 1:
        GPStime,GPSsats,GPSlat,GPSlong,GPSalt = read_GGA()  # Fetch GPS data
        currDict['Latitude (DD)'] = str(GPSlat)  # Write GPS data to dictionary
        currDict['Longitude (DD)'] = str(GPSlong)
        currDict['GPS Alt (m)'] = str(GPSalt)
        sleep(1)
    
def radioLoop():
    print('Radio Thread Begin\n')
    global arduino_data, GPSlat, GPSlong, packet_list, imgCount, imgFlag
    
    while 1:
        #TRANSMITTING IMAGE: Consider moving to separate thread
        if not packet_list:
            print("No Image Data to send")
            sleep(1)
        else:
            imgId = 'IMG' + str(imgCount) + ' \n' #the value will chane for each new image
            imgIdHdr = bytes(imgId, 'UTF-8')
            #endChar = bytes("END\n", 'UTF-8')
            tarPckt = packet_list[0].content
            #pcktX = b''.join([imgIdHdr, tarPckt])
            pcktX = imgIdHdr + tarPckt
            #print("PcktX: " + str(pcktX))
            #pcktToSend = imgIdHdr + tarPckt
            #imgIdHdr += tarPckt
            print("Sending packet Data")
            radioModule.write(pcktX)
            packet_list.pop(0)
            
            #Check to see if list is now empty
            if not packet_list:
                imgCount+=1
                #Potentially insert a delay for other thread to recognize that the list is empty
        
def ETSLoop():  # Handles triggering of the ETS based on GPS and altitude data
    print('ETS Thread Begin\n')
    global GPSlat, GPSlong, xRadEllipse, yRadEllipse, ETS_arm_height, ETS_fall_count_threshold, ETS_trigger_velocity
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(3, GPIO.OUT)  # Uses physical board numbers. = GPIO2
    GPIO.output(3, GPIO.LOW)  # Initialize ETS activation pin as low (logical 0)
    
    while ((GPSlat == 0.0) & (GPSlong == 0.0)):  # Wait for valid GPS readings to begin before arming ETS
        print("Waiting for valid GPS coords to record start point!")
        sleep(1)
    GPSlatStart = GPSlat  # Record starting point latitude
    GPSlongStart = GPSlong  #Record starting point longitude
    print("Start point GPS coords recorded!")
    print("Starting LAT: " + str(GPSlatStart))
    print("Starting LON: " + str(GPSlongStart))
    
    lastAltitude = currAltitude = float(currDict['Red Alt (m)'])
    ETS_armed = False  # Start with the ETS unarmed - won't activate from velocity readings until it passes threshold
    ETS_fall_count = 0  # Counts number of threshold breaking velocity readings in a row

    while 1:
        
        if(float(currDict['Red Alt (m)']) > ETS_arm_height):  # Arm ETS if altitude threshold is passed
            ETS_armed = True
        
        lastAltitude = currAltitude  # Pass on the last read altitude to dedicated variable
        currAltitude = float(currDict['Red Alt (m)'])  # Read the current altitude
        currVelocity = currAltitude - lastAltitude  # Calculate vertical velocity of payload in m/s
        
#         print("Last Altitude: " + str(lastAltitude))  # Debug Lines
#         print("Current Altitude: " + str(currAltitude))
#         print("Current Velocity: " + str(currVelocity))
#         print("Is ETS armed?: " + str(ETS_armed))
#         print("ETS Fall Count: " + str(ETS_fall_count))
        
        if(ETS_armed & (currVelocity < ETS_trigger_velocity)):  # If the ETS is armed and a low enough velocity is detected
            ETS_fall_count += 1  # Iterate the fall count 
            if(ETS_fall_count >= ETS_fall_count_threshold):
                GPIO.output(3, GPIO.HIGH)  # Activate ETS NMOS by setting GPIO high
                print("ETS SYSTEM TRIGGERED (HIGH DROP VELOCITY)")
                currDict['ETS Triggered'] = 1  # Change ETS Triggered dictionary value to high
            else:
                ETS_fall_count = 0  # Reset the count if a velocity under the threshold is detected

        # Inequality (eqLeftSide > eqRightSide) used to determine if the current payload location is outside of the ellipseWPI
        eqLeftSide = (yRadEllipse ** 2) * ((GPSlong - GPSlongStart) ** 2) + (xRadEllipse ** 2) * ((GPSlat - GPSlatStart) ** 2)
        eqRightSide = (xRadEllipse ** 2) * (yRadEllipse ** 2)
#         print("Eq Left Side:  " + str(eqLeftSide))  # Debug line
#         print("Eq Right Side: " + str(eqRightSide))  # Debug line
        if (eqLeftSide > eqRightSide):  # If equality is untrue...
            etsTrigCount = etsTrigCount + 1  # Increase count by one if coord is outside ETS bound
            if(etsTrigCount >= etsTrigWait):  # If consecutive hits are greater than the set wait count, trigger ETS
                GPIO.output(3, GPIO.HIGH)  # Activate ETS NMOS by setting GPIO high
                print("ETS SYSTEM TRIGGERED (GPS BOUNDARY CROSSED)")
                currDict['ETS Triggered'] = 1  # Change ETS Triggered dictionary value to high
        else:
            etsTrigCount = 0  # Reset consecutive count # if coord is inside boundary
        sleep(1)  # One Second wait between GPS message receptions
 
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
arduino_thread = Thread(target = arduinoLoop)  # Handles reception, packaging and sending of sensor data from Arduino
pi_thread = Thread(target = piLoop)  # Handles reception of data from GPS module
transmit_thread = Thread(target = radioLoop)
ets_thread = Thread(target = ETSLoop)  # Handles triggering of the ETS based on GPS and altitude data
image_capture_thread = Thread(target = captureImage)

# Initializing PiCamera
camera = PiCamera()
camera.resolution = (1280, 720)
camera.vflip = False  # Consider changing this. Might not be necessary
time.sleep(2)

# Start threads
arduino_thread.start()
pi_thread.start()
transmit_thread.start()
ets_thread.start()
image_capture_thread.start()