#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Empty
import sys
import time

def talker():
    pub = rospy.Publisher('/lane_marker_flag', Empty, queue_size=1)
    rospy.init_node('lane_marker_flag', anonymous=True)
    cnt = 0
    time.sleep(2.5)
    flag = Empty()
    pub.publish(flag)
    # while not rospy.is_shutdown():
    #     if cnt > 3:
    #         break
    #     cnt += 1
    #     flag = Empty()
    #     pub.publish(flag)
    #     time.sleep(1)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
