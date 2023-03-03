# HAB-IV-Code
This is a repository for the fourth iteration of the High Altitude Balloon project(HAB) Major Qualifying Project(MQP) at WPI

This repository contains all code and resources used during the HABIV WPI MQP testing. This code is seperated into two main sections the code used on the base station and the code used on the payload.

# Payload Code
The payload read data from a series of sensors and recorded and sent that data via the Xbee radio. The payload code is in two seperate sections the code that was run on the Ardunio Uno and the Code that was run on the Raspberry PI 4. 
The function of the Arduino was to read data from the primary sensors and do some basic processing on that data. This was done on the Arduino because it is better able to handle processing the sensor data than the Pi. After reading the sensor values the data is then sent via the serial port to the Rasberry Pi. 
The Pi code will read the data from the Arduino as well as the data from a few secondary or backup sensors. It will then send that data to the radio which will transmit it. One of the main sensors on the Pi is a secondary GPS sensor whose postion data gets send via the radio rather than the through satalites like the spot tracker. To incorportate the GPS sensor and Arduino which both have their own own clock threading was used. The basic components of each thread can be seen in the example code provided. 

# Base Station
The base station code recieves the data sent from the payload vis the radio. This data is then processed and posted online in real time to ThinkSpeak API. This enables us to have real time graphing of data coming in from our payload. The base station code also uses the GPS data to track the postion of the payload and determine the angle for the antenna to get the best signal from it. 
