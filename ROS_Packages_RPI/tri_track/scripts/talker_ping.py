#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import Float32

import RPi.GPIO as GPIO
import time
import threading

GPIO.setmode(GPIO.BCM)

startTime = 0
endTime = 0
distance = 0
duration = 0

#GPIO.setmode(GPIO.BCM)

def sendPulse():
    GPIO.setup(14, GPIO.OUT) #pinmode output

    GPIO.output(14, GPIO.LOW) #output low
    time.sleep(0.000005)

    GPIO.output(14, GPIO.HIGH) #output high
    time.sleep(0.000005)
    
    GPIO.output(14, GPIO.LOW) #output low
    

def getPulseTimes():
    global startTime
    global endTime

    GPIO.setup(14, GPIO.IN) #pinmode input

    while(GPIO.input(14) == 0):
        startTime = time.time()

    while(GPIO.input(14) == 1):
        endTime = time.time()
            

def calcDistance():
    global distance
    global duration
    
    duration = endTime - startTime
    duration = duration / 2

    distance = duration / 29
    distance = distance * 1000000 #converts to cm


def getDistance():
    sendPulse()
    getPulseTimes()
    calcDistance()
    time.sleep(0.1)


        
def talker():
    pub = rospy.Publisher('PINGSensorTopic', Float32, queue_size=10)
    rospy.init_node('PINGSensorTalker', anonymous=True)
    rate = rospy.Rate(50) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()

        getDistance()
        
        rospy.loginfo(distance)
        pub.publish(distance)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
