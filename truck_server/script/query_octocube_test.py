#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Vector3
import sys
import time

def talker():
    pub = rospy.Publisher('/query_point_depth', Vector3, queue_size=10)
    rospy.init_node('query_input', anonymous=True)
    print "Press t to quit."
    while not rospy.is_shutdown():
        for z in range (0, 50):
            for x in range(-44, 60):
                for y in range (-53, 53):
                    input_l = [x*0.1, y*0.1, z*0.1]
                    print input_l
                    vec = Vector3()
                    vec.x = input_l[0]
                    vec.y = input_l[1]
                    vec.z = input_l[2]
                    pub.publish(vec)
                    time.sleep(0.2)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
