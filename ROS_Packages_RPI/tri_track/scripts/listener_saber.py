#!/usr/bin/env python

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String

import time
import pigpio 
import threading
pi = pigpio.pi()

incomingData = '0000'


################### Connecting to Tri-Track ###################

pwmPin1 = 19 #Right side/CH1    
pwmPin2 = 12 #Left side/CH2


dutyCycleForward = 18
dutyCycleTurn = [18,14]
dutyCycleStop = 16
dutyCycleBack = 14

frequency = 100


def runSabertooth():
    while True:
        command = list(incomingData)
                       
        pi.hardware_PWM(pwmPin1, frequency, dutyCycleStop*10000)
        pi.hardware_PWM(pwmPin2, frequency, dutyCycleStop*10000)
        
        while(command[0]=='1'): #D-pad up
            pi.hardware_PWM(pwmPin1, frequency, dutyCycleForward*10000)
            pi.hardware_PWM(pwmPin2, frequency, dutyCycleForward*10000)
            
        while(command[1]=='1'): #D-pad down
            pi.hardware_PWM(pwmPin1, frequency, dutyCycleBack*10000)
            pi.hardware_PWM(pwmPin2, frequency, dutyCycleBack*10000)
        
        while(command[2]=='1'): #D-pad left
            pi.hardware_PWM(pwmPin1, frequency, dutyCycleTurn[0]*10000)
            pi.hardware_PWM(pwmPin2, frequency, dutyCycleTurn[1]*10000)

        while(command[3]=='1'): #D-pad right
            pi.hardware_PWM(pwmPin1, frequency, dutyCycleTurn[1]*10000)
            pi.hardware_PWM(pwmPin2, frequency, dutyCycleTurn[0]*10000)


connectSaberThread = threading.Thread(target=runSabertooth, args=())
connectSaberThread.start()


################### Getting data from ROS ###################

def callback(data):
    global incomingData
    incomingData = data.data
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', incomingData)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('SaberListener', anonymous=True)

    rospy.Subscriber('SaberTopic', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()










