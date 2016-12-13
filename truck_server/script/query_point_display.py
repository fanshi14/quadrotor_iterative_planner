#!/usr/bin/env python
# license removed for brevity
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
import sys

class markerDisplay:
    def __callback(self, data):
        print "Display new point"
        self.__point_marker.pose.position.x = data.x
        self.__point_marker.pose.position.y = data.y
        self.__point_marker.pose.position.z = data.z
        self.__marker_pub.publish(self.__point_marker)

    def main(self):
        self.__marker_pub = rospy.Publisher('/query_point_marker', Marker, queue_size=10)
        self.__point_marker = Marker()
        self.__point_marker.header.frame_id = "world"
        self.__point_marker.header.stamp = rospy.Time.now()
        self.__point_marker.ns = "basic_shapes"
        self.__point_marker.id = 0
        self.__point_marker.type = Marker.SPHERE
        self.__point_marker.action = Marker.ADD
        self.__point_marker.pose.orientation.x = 0.0
        self.__point_marker.pose.orientation.y = 0.0
        self.__point_marker.pose.orientation.z = 0.0
        self.__point_marker.pose.orientation.w = 1.0
        self.__point_marker.scale.x = 0.3
        self.__point_marker.scale.y = 0.3
        self.__point_marker.scale.z = 0.3
        self.__point_marker.color.a = 1.0
        self.__point_marker.color.r = 1
        self.__point_marker.color.g = 0.0
        self.__point_marker.color.b = 0

        rospy.Subscriber("/query_point", Vector3, self.__callback)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('query_point_display', anonymous=True)
    display = markerDisplay()
    display.main()
