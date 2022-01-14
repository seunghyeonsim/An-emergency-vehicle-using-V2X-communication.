#!/usr/bin/env python

import RPi.GPIO as GPIO
import time
import signal
import sys
import socket
#import rospy

#def signal_handler(sig, frame):
#    import time
#    time.sleep(2)
#    os.system('killall -9 python rosout')
#    sys.exit(0)
#signal.signal(signal.SIGINT, signal_handler)

#rospy.init_node('human_track')
 
def server2():
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    sock.bind(('192.168.154.113',8080))
    data,addr=sock.recvfrom(20000)
    print("f",data.decode())
    return(data.decode())




# Setup
GPIO.setmode(GPIO.BCM)

GPIO.setup(9, GPIO.OUT)
GPIO.setup(10, GPIO.OUT)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(7, GPIO.OUT)
GPIO.setup(8, GPIO.OUT)

# Turn off all lights when user ends demo
def allLightsOff(signal, frame):
    
    GPIO.output(10, False)
    GPIO.output(11, False)
    GPIO.output(8, False)
    GPIO.output(7, False)
    GPIO.cleanup()
    sys.exit(0)
signal.signal(signal.SIGINT, allLightsOff)
# Loop forever


def Red(sec):
    # Red 
    GPIO.output(11, False)
    GPIO.output(10, True)
    GPIO.output(8, False)
    GPIO.output(7, True)
    
    
    time.sleep(sec)

def Green(sec):
    GPIO.output(11, True)
    GPIO.output(10, False)
    GPIO.output(8, True)
    GPIO.output(7, False)
    time.sleep(sec)

def Yellow(sec):
    GPIO.output(11, False)
    GPIO.output(10, False)
    GPIO.output(8, False)
    GPIO.output(7, False)
    #GPIO.output(?, False)
    #GPIO.output(?, False)
    time.sleep(sec)



while True :
    Green(5)
    Red(5)
    a=server2()
    if a=="tr":
        Green(5)
    elif  a=="Green":
        Green(5)
    else :
        Red(5)
        Green(5)
   
        
