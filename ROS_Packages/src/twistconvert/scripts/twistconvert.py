#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Distance between the robot's wheels in meteres
WHEEL_DIST = 0.5 

def callback(data):
    vLinearx = data.linear.x
    vAngularz = data.angular.z
    
    #rospy.loginfo(vLinearx)
    #rospy.loginfo(vAngularz)

    # Convert the Twist message into velocities (m/s) for the right and left motors
    rightWheelSpeed = (vAngularz * WHEEL_DIST) / 2 + vLinearx 
    leftWheelSpeed = (vLinearx * 2) - rightWheelSpeed

    rospy.loginfo("Right Wheel Speed: %s" % rightWheelSpeed)
    rospy.loginfo("Left Wheel Speed:  %s" % leftWheelSpeed)


    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listenersaber', anonymous=True)

    rospy.Subscriber("cmd_vel", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()