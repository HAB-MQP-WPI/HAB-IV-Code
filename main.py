import serial 
from serial.tools import list_ports
import shutil
import os
import time
import csv

list1 = list_ports.comports() 
connected = []
for element in list1:
    connected.append(element.device)
print("Connected COM ports: " + str(connected)) 

#Radio Module
ser = serial.Serial( 
        port='COM4', #was COM3
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1, 
)

#Arduino
ser2 = serial.Serial( 
        port='COM9', #Change to port listed in Arduino IDE
        baudrate=9600
)

#GLOBAL VARIABLES
oldImgId = "IMG0"
latIndex = 0
longIndex = 0
latitude = 0
longitude = 0
coordFlag = 0;
oldLat = 0.0;
oldLong = 0.0;
timeFlag = 1

#Closing and Reopening serial ports
ser.close() 
ser2.close()
ser.open()
ser2.open()

#File Creation
f = open("input.bin", "w+b")

mainData = open("data.csv", "a", newline="")
writer = csv.writer(mainData) #create a wrirter object
#Create headers
writer.writerow(["Date", "Time", "Latitude (DD)", "Longitude (DD)", "GPS Alt (m)", "ETS Triggered", "BMP180 Temp (*C)", "BMP180 Pressure (Pa)", "BMP180 Alt (m)", "CO2 (ppm)", "OZ (ppb)", "CH4 (ppm)", "UV Index", "One-Wire Temp (*F)", "Red Pressure Temp (*C)", "Red Pressure Temp (*F)", "Red Pressure (mb)", "Red Alt (m)", "Red Alt Change from Startup (m)"])
mainData.close()

#Determine if serial port is open
print(ser.isOpen())

#MAIN BLOCK
while 1:

    if timeFlag:
        timeMarker = time.time() + 1
        xTime = timeMarker - 1
        timeFlag = 0

    x = ser.readline()
    x_str = str(x, 'utf-8', errors = 'ignore') #may have to remove ignore errors flaag



    if x_str == "":
        continue
    
    else:
    
        if 'IMG' in x_str:

            #Save image ID & read in data
            currImgId = (x_str.split(" ",1)[0])
            imgData = ser.read(256)

            #Compare current ImgID
            if currImgId == oldImgId:
                print("IMG IDs match!")
                print("currImgId:" + currImgId)
                print("oldImgId:" + oldImgId + "\n")
                f.write(imgData)  
            else:
                print("IMG IDs do not match")
                print("currImgId:" + currImgId)
                print("oldImgId:" + oldImgId + "\n")

                fileID = str(oldImgId) + ".jpeg"
                binID = oldImgId + ".bin"

                oldImgId = currImgId

                f.close()
                f = open("input.bin", "rb")
                
                with open(binID, "wb") as binCpy:
                    shutil.copyfileobj(f,binCpy)
                binCpy.close()
                
                f.close()
                os.remove("input.bin")
                f = open("input.bin", "w+b") 
                f.write(imgData)

        elif "PAYLOAD DATA PACKET" in x_str:

            packetData = ser.readline() #next line will contain data buffer separated by commas
            packetData = packetData.decode()
            packetData = packetData.rstrip("\n")

            print(packetData)
            #Write to CSV file
            data = packetData.split(",")
            print(data)
            mainData = open("data.csv",'a') #Open csv file with append permission - add to existing content
            writer = csv.writer(mainData) #create a wrirter object
            writer.writerow([data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15], data[16], data[17], data[18]])
            mainData.close()
            lat = data[2]
            lon = data[3]
            alt = data[4]

            currTime = time.time()
            if (currTime >= timeMarker):
                print("Time passed: " + str(currTime-xTime) + "\n")
                data = str(lon) + "," + str(lat) + "," + str(alt) + "\n"
                ser2.write(data.encode())
                print("Sending coords to Arduino ...")
                print("SEND LON: " + str(lon) + " SEND LAT: " + str(lat) + " SEND ALT: " + str(alt))
                timeFlag = 1

        else: 
            print("Unrecognized data\n")
                
