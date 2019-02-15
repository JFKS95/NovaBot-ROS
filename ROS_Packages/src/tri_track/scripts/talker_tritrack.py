#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import Tri_Track_Manual_Master as TriTrack

def talker():
    pub = rospy.Publisher('ArmTopic', String, queue_size=10)
    rospy.init_node('ArmTalker', anonymous=True)
    rate = rospy.Rate(50) # 50hz
    while not rospy.is_shutdown():
        message = str(TriTrack.getTriTrackMessage())
        rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
