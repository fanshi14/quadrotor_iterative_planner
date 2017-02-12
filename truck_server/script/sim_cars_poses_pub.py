#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import sys
import math

def talker():
    rospy.init_node('sim_cars_poses_pub', anonymous=True)
    has_car_small = rospy.get_param("~has_car_small", False)
    has_car_big = rospy.get_param("~has_car_big", False)
    route_name = rospy.get_param("~route_name", "circle")
    route_radius = rospy.get_param("~route_radius", 20.0)
    truck_vel = rospy.get_param("~truck_vel", 4.17)
    car_small_vel = rospy.get_param("~car_small_vel", 4.17)
    car_big_vel = rospy.get_param("~car_big_vel", 4.17)

    cars_poses_pub = rospy.Publisher('/cars_poses', PoseArray, queue_size=1)
    truck_odom_pub = rospy.Publisher('/truck_odom', Odometry, queue_size=1)
    car_small_odom_pub = rospy.Publisher('/car_small_odom', Odometry, queue_size=1)
    car_big_odom_pub = rospy.Publisher('/car_big_odom', Odometry, queue_size=1)

    car_small_radius = route_radius - 3.5
    car_big_radius = route_radius + 3.5
    truck_radius = route_radius

    print "Route name: ", route_name

    car_small_ang_vel = car_small_vel / car_small_radius
    car_big_ang_vel = car_big_vel / car_big_radius
    truck_ang_vel = truck_vel / truck_radius
    cnt = 0
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        poses = PoseArray()
        truck_odom = car_small_odom = car_big_odom = Odometry()
        poses.header.frame_id = truck_odom.header.frame_id = car_small_odom.header.frame_id = car_big_odom.header.frame_id = "world"
        poses.header.stamp = truck_odom.header.stamp = car_small_odom.header.stamp = car_big_odom.header.stamp = rospy.Time.now()

        if route_name == "circle":
            ## truck
            cur_pose = Pose()
            truck_ang = truck_ang_vel * cnt * 0.02
            cur_pose.position.x = truck_radius * math.sin(truck_ang)
            cur_pose.position.y = -truck_radius * math.cos(truck_ang)
            cur_pose.position.z = 0.8
            cur_pose.orientation.w = truck_ang
            poses.poses.append(cur_pose)
            truck_odom.pose.pose = cur_pose
            truck_odom_pub.publish(truck_odom)
            ## car small
            if has_car_small:
                temp_pose = Pose()
                car_small_ang = car_small_ang_vel * cnt * 0.02
                temp_pose.position.x = car_small_radius * math.sin(car_small_ang)
                temp_pose.position.y = -car_small_radius * math.cos(car_small_ang)
                temp_pose.orientation.w = car_small_ang
                poses.poses.append(temp_pose)
                car_small_odom.pose.pose = temp_pose
                car_small_odom_pub.publish(car_small_odom)
            ## car big
            if has_car_big:
                temp_pose = Pose()
                car_big_ang = car_big_ang_vel * cnt * 0.02
                temp_pose.position.x = car_big_radius * math.sin(car_big_ang)
                temp_pose.position.y = -car_big_radius * math.cos(car_big_ang)
                temp_pose.orientation.w = car_big_ang
                poses.poses.append(temp_pose)
                car_big_odom.pose.pose = temp_pose
                car_big_odom_pub.publish(car_small_odom)

            cars_poses_pub.publish(poses)
            cnt += 1
            rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
