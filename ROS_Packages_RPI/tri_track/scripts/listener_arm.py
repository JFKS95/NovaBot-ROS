#!/usr/bin/env python

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String

import serial
import pigpio 
import time
import threading
import pyudev
context = pyudev.Context()
pi = pigpio.pi()


armData = '080170170090030001' + '\n'
saberData = ['0','0','0','0']


########################### Botboarduino ###########################

def connectArduino():
    while True:
        try:
            for device in context.list_devices(subsystem='tty', ID_SERIAL_SHORT='A49BK0V'):
                port = '/dev/' + device.sys_name
            arduino = serial.Serial(port, baudrate = 115200, timeout = 1)
            print('Connected to Arduino')
            sendData(arduino)
            return

        except:
            print('Searching for Arduino...')
            time.sleep(0.5)
            

def sendData(ser):
    while True:
        ser.write(armData.encode())

connectArduinoThread = threading.Thread(target=connectArduino, args=())
connectArduinoThread.start()




################### Connecting to Tri-Track ###################

pwmPin1 = 19 #Right side/CH1    
pwmPin2 = 12 #Left side/CH2

dutyCycleForward = 18
dutyCycleTurn = [18,14]
dutyCycleStop = 16
dutyCycleBack = 14

frequency = 100

def connectSaber():
    while True:        
        pi.hardware_PWM(pwmPin1, frequency, dutyCycleStop*10000)
        pi.hardware_PWM(pwmPin2, frequency, dutyCycleStop*10000)
        
        while(saberData[0]=='1'): #D-pad up
            pi.hardware_PWM(pwmPin1, frequency, dutyCycleForward*10000)
            pi.hardware_PWM(pwmPin2, frequency, dutyCycleForward*10000)
            
        while(saberData[1]=='1'): #D-pad down
            pi.hardware_PWM(pwmPin1, frequency, dutyCycleBack*10000)
            pi.hardware_PWM(pwmPin2, frequency, dutyCycleBack*10000)
        
        while(saberData[2]=='1'): #D-pad left
            pi.hardware_PWM(pwmPin1, frequency, dutyCycleTurn[0]*10000)
            pi.hardware_PWM(pwmPin2, frequency, dutyCycleTurn[1]*10000)

        while(saberData[3]=='1'): #D-pad right
            pi.hardware_PWM(pwmPin1, frequency, dutyCycleTurn[1]*10000)
            pi.hardware_PWM(pwmPin2, frequency, dutyCycleTurn[0]*10000)


connectSaberThread = threading.Thread(target=connectSaber, args=())
connectSaberThread.start()




########################### ROS Message ###########################

def callback(data):
    global armData
    global saberData

    recvData = data.data
    
    armData = recvData[:-4] + '\n'
    saberData = list(recvData[18:])
    
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', recvData)

def listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('ArmListener', anonymous=True)

    print('1')
    
    rospy.Subscriber('ArmTopic', String, callback)

    print('2')
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    print('3')

if __name__ == '__main__':
    listener()

