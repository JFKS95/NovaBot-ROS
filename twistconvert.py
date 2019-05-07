#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
#import threading
import time
import pigpio 
pi = pigpio.pi()


# Distance between the robot's wheels in meteres and max speed in m/s
WHEEL_DIST = 0.23
MAX_SPEED = 0.26

rightWheelSpeed = 0.0
leftWheelSpeed = 0.0


################### Connecting to Tri-Track ###################

pwmPin1 = 19 #Right side/CH1    
pwmPin2 = 12 #Left side/CH2

# Duty Cycles
dutyCycleForward = 20
dutyCycleBack = 10
dutyCycleStop = 16 #16.25 forward starts, 15.6 reverse starts

dutyCycleStopF = 16.5
dutyCycleStopR = 15.1

dutyCycleTurn = [dutyCycleForward, dutyCycleBack]

# Default values stop motors
dutyCycleMotor1 = dutyCycleStop 
dutyCycleMotor2 = dutyCycleStop

# PWM frequency
frequency = 100 


def connectTriTrack():
    while not rospy.is_shutdown():
        #try:
            # if Right wheel is negative and left wheel is positive
            if(rightWheelSpeed < 0 and leftWheelSpeed > 0):
                #print("clockwise")
                dutyCycleMotor1 = dutyCycleStopR + ((2/MAX_SPEED ) *  rightWheelSpeed)
                dutyCycleMotor2 = dutyCycleStopF + ((2/MAX_SPEED  ) * leftWheelSpeed)
                
                pi.hardware_PWM(pwmPin1, frequency, int(dutyCycleMotor1*10000))
                pi.hardware_PWM(pwmPin2, frequency, int(dutyCycleMotor2*10000))


            # if Right wheel is positive and left wheel is negative
            elif(rightWheelSpeed > 0 and leftWheelSpeed < 0):
                #print("anticlockwise")
                dutyCycleMotor1 = dutyCycleStopF + ((2/MAX_SPEED ) *  rightWheelSpeed)
                dutyCycleMotor2 = dutyCycleStopR + ((2/MAX_SPEED  ) * leftWheelSpeed)
                
                pi.hardware_PWM(pwmPin1, frequency, int(dutyCycleMotor1*10000))
                pi.hardware_PWM(pwmPin2, frequency, int(dutyCycleMotor2*10000))


            # if Right wheel is positive and left wheel is positive
            elif(rightWheelSpeed > 0 and leftWheelSpeed > 0):
                dutyCycleMotor1 = dutyCycleStopF + ((2/MAX_SPEED ) *  rightWheelSpeed)
                dutyCycleMotor2 = dutyCycleStopF + ((2/MAX_SPEED  ) * leftWheelSpeed)
                
                pi.hardware_PWM(pwmPin1, frequency, int(dutyCycleMotor1*10000))
                pi.hardware_PWM(pwmPin2, frequency, int(dutyCycleMotor2*10000))


            # if Right wheel is negative and left wheel is negative
            elif(rightWheelSpeed < 0 and leftWheelSpeed < 0):
                dutyCycleMotor1 = dutyCycleStopR + ((2/MAX_SPEED ) *  rightWheelSpeed)
                dutyCycleMotor2 = dutyCycleStopR + ((2/MAX_SPEED  ) * leftWheelSpeed)
                
                pi.hardware_PWM(pwmPin1, frequency, int(dutyCycleMotor1*10000))
                pi.hardware_PWM(pwmPin2, frequency, int(dutyCycleMotor2*10000))




            # if Right wheel is stopped and left wheel is positive
            elif(rightWheelSpeed == 0 and leftWheelSpeed > 0):
                dutyCycleMotor1 = dutyCycleStop + ((2/MAX_SPEED ) *  rightWheelSpeed)
                dutyCycleMotor2 = dutyCycleStopF + ((2/MAX_SPEED  ) * leftWheelSpeed)
                
                pi.hardware_PWM(pwmPin1, frequency, int(dutyCycleMotor1*10000))
                pi.hardware_PWM(pwmPin2, frequency, int(dutyCycleMotor2*10000))
                

            # if Right wheel is stopped and left wheel is negative
            elif(rightWheelSpeed == 0 and leftWheelSpeed < 0):
                dutyCycleMotor1 = dutyCycleStop + ((2/MAX_SPEED ) *  rightWheelSpeed)
                dutyCycleMotor2 = dutyCycleStopR + ((2/MAX_SPEED  ) * leftWheelSpeed)
                
                pi.hardware_PWM(pwmPin1, frequency, int(dutyCycleMotor1*10000))
                pi.hardware_PWM(pwmPin2, frequency, int(dutyCycleMotor2*10000))


            # if Right wheel is positive and left wheel is stopped
            elif(rightWheelSpeed > 0 and leftWheelSpeed == 0):
                dutyCycleMotor1 = dutyCycleStopF + ((2/MAX_SPEED ) *  rightWheelSpeed)
                dutyCycleMotor2 = dutyCycleStop + ((2/MAX_SPEED  ) * leftWheelSpeed)
                
                pi.hardware_PWM(pwmPin1, frequency, int(dutyCycleMotor1*10000))
                pi.hardware_PWM(pwmPin2, frequency, int(dutyCycleMotor2*10000))


            # if Right wheel is negative and left wheel is stopped
            elif(rightWheelSpeed < 0 and leftWheelSpeed == 0):
                dutyCycleMotor1 = dutyCycleStopR + ((2/MAX_SPEED ) *  rightWheelSpeed)
                dutyCycleMotor2 = dutyCycleStop + ((2/MAX_SPEED  ) * leftWheelSpeed)
                
                pi.hardware_PWM(pwmPin1, frequency, int(dutyCycleMotor1*10000))
                pi.hardware_PWM(pwmPin2, frequency, int(dutyCycleMotor2*10000))



            # Both stopped
            else:
                dutyCycleMotor1 = dutyCycleStop + ((2/MAX_SPEED ) *  rightWheelSpeed)
                dutyCycleMotor2 = dutyCycleStop + ((2/MAX_SPEED  ) * leftWheelSpeed)
                
                pi.hardware_PWM(pwmPin1, frequency, int(dutyCycleMotor1*10000))
                pi.hardware_PWM(pwmPin2, frequency, int(dutyCycleMotor2*10000))                

            #time.sleep(0.01)

        #except:
        #    print('Retrying PWM...')
        #    time.sleep(0.5)

#connectTriTrackThread = threading.Thread(target=connectTriTrack, args=())
#connectTriTrackThread.start()



################### Connecting to ROS ###################

def callback(data):
    global rightWheelSpeed
    global leftWheelSpeed
    
    vLinearx = data.linear.x
    vAngularz = data.angular.z
    
    #rospy.loginfo(vLinearx)
    #rospy.loginfo(vAngularz)

    # Convert the Twist message into velocities (m/s) for the right and left motors
    rightWheelSpeed = ((vAngularz * WHEEL_DIST) / 2 + vLinearx)# * 1.25
    leftWheelSpeed = ((vLinearx * 2) - rightWheelSpeed)# * 1.25

    rospy.loginfo(rightWheelSpeed)
    rospy.loginfo(leftWheelSpeed)


    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listenernode', anonymous=True)

    rospy.Subscriber("cmd_vel", Twist, callback)

    connectTriTrack()
    
    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

    
    
if __name__ == '__main__':
    listener()    
