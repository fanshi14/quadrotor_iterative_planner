#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Empty
import sys
import time

def talker():
    pub = rospy.Publisher('/truck_octomap_flag', Empty, queue_size=1)
    rospy.init_node('truck_octomap_flag', anonymous=True)
    while not rospy.is_shutdown():
        flag = Empty()
        pub.publish(flag)
        time.sleep(1)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
