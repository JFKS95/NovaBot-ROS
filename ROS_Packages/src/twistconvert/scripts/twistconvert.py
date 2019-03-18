#!/usr/bin/env python
import rospy
#from std_msgs.msg import String
from geometry_msgs.msg import Twist


WHEEL_DIST = 0.5

def callback(data):
    vLinearx = data.linear.x
    vAngularz = data.angular.z
    
    #rospy.loginfo(vLinearx)
    #rospy.loginfo(vAngularz)

    speed_wish_right = (vAngularz * WHEEL_DIST) / 2 + vLinearx
    speed_wish_left = (vLinearx * 2) - speed_wish_right

    rospy.loginfo(speed_wish_right)
    rospy.loginfo(speed_wish_right)


    
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