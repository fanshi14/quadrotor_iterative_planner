#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Vector3
import sys

def talker():
    pub = rospy.Publisher('/query_point_depth', Vector3, queue_size=10)
    rospy.init_node('query_input', anonymous=True)
    print "Press t to quit."
    while not rospy.is_shutdown():
        input_s = sys.stdin.readline()
        if input_s[0] == 't':
            break
        input_l = [float(i) for i in input_s.split()]
        vec = Vector3()
        vec.x = input_l[0]
        vec.y = input_l[1]
        vec.z = input_l[2]
        pub.publish(vec)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
