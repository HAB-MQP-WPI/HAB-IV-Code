from time import sleep
import RPi.GPIO as GPIO  # Library for using Pi GPIO pins
import serial  # Used with serial devices (Arduino, radio module, GPS)

# This test is used to set a countdown until the ETS activates.
# Highly recommended you hang the payload from a tree branch or something, start this
# code, and have someone ready to catch the payload when it activates.

# Set up radio module com port
radioModule =serial.Serial(
    port='/dev/ttyUSB0',  # If error, try swapping radio and GPS serial port names! Usually works
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(3, GPIO.OUT)  # Uses physical board numbers. = GPIO2
GPIO.output(3, GPIO.LOW)

t = 60  # Time in seconds to count down from

while(t):
    timeMessage = str(t) + " seconds to activation!"
    print(timeMessage)
    radioModule.write(bytes(timeMessage, 'UTF-8'))  # Send the string packet over radio to base station
    sleep(1) # One Second wait between GPS message receptions
    t-=1
    
GPIO.output(3, GPIO.HIGH) # Activate ETS NMOS by setting GPIO high
print("ETS SYSTEM TRIGGERED")