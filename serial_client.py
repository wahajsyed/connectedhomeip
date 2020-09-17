import serial
import time

def toggleBulb():
    sendActuation("B")

def reset():
    sendActuation("R")

def sendActuation(command):
    command = command + "\r\n"

    lightbulb.write(command)
    print "<<\n"
    print lightbulb.read(100)
    bytesToRead = lightbulb.inWaiting()
    while bytesToRead > 0 :
        print lightbulb.read(bytesToRead)
        bytesToRead = lightbulb.inWaiting()


lightbulb = serial.Serial("/dev/tty.usbmodem0000000000001", baudrate=115200, timeout=2.0)

count = 0 
while count < 10:
    print "Toggle Light Bulb"
    toggleBulb()
    time.sleep(2)
    count = count + 1

