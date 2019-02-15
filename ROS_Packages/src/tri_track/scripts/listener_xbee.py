#!/usr/bin/env python
import rospy
import subprocess
from std_msgs.msg import String

def callback(data):
    data = data.data
    data = data[0:-13]
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    if data=='s':
        subprocess.call(['python3', '/home/team-g/catkin_ws/src/beginner_tutorials/scripts/Xbee_API_Transmit_Frame.py'])

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('XbeeNode', anonymous=True)

    rospy.Subscriber("XbeeTopic", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
