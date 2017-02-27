#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys
import math

def talker():
    rospy.init_node('sim_cars_poses_pub', anonymous=True)
    has_car_inner = rospy.get_param("~has_car_inner", False)
    has_car_outter = rospy.get_param("~has_car_outter", False)
    route_id = rospy.get_param("~route_id", "circle")
    route_radius = rospy.get_param("~route_radius", 20.0)
    truck_vel = rospy.get_param("~truck_vel", 4.17)
    car_inner_vel = rospy.get_param("~car_inner_vel", 4.17)
    car_outter_vel = rospy.get_param("~car_outter_vel", 4.17)
    lane_width = rospy.get_param("~lane_width", 5)

    cars_poses_pub = rospy.Publisher('/cars_poses', PoseArray, queue_size=1)
    truck_odom_pub = rospy.Publisher('/truck_odom', Odometry, queue_size=1)
    car_inner_odom_pub = rospy.Publisher('/car_inner_odom', Odometry, queue_size=1)
    car_outter_odom_pub = rospy.Publisher('/car_outter_odom', Odometry, queue_size=1)

    car_inner_radius = route_radius - lane_width
    car_outter_radius = route_radius + lane_width
    truck_radius = route_radius

    print "Route id: ", route_id

    car_inner_ang_vel = car_inner_vel / car_inner_radius
    car_outter_ang_vel = car_outter_vel / car_outter_radius
    truck_ang_vel = truck_vel / truck_radius
    cnt = 0
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        poses = PoseArray()
        truck_odom = car_inner_odom = car_outter_odom = Odometry()
        poses.header.frame_id = truck_odom.header.frame_id = car_inner_odom.header.frame_id = car_outter_odom.header.frame_id = "world"
        poses.header.stamp = truck_odom.header.stamp = car_inner_odom.header.stamp = car_outter_odom.header.stamp = rospy.Time.now()

        ## truck
        cur_pose = Pose()
        truck_ang = truck_ang_vel * cnt * 0.02
        cur_pose.position.x = truck_radius * math.sin(truck_ang)
        cur_pose.position.y = -truck_radius * math.cos(truck_ang)
        cur_pose.position.z = 0.8
        cur_pose.orientation.w = truck_ang
        poses.poses.append(cur_pose)
        truck_odom.pose.pose = cur_pose
        cur_twist = Twist()
        cur_twist.linear.x = truck_vel * math.cos(truck_ang)
        cur_twist.linear.y = truck_vel * math.sin(truck_ang)
        truck_odom.twist.twist = cur_twist
        truck_odom_pub.publish(truck_odom)
        ## car small
        if has_car_inner:
            temp_pose = Pose()
            car_inner_ang = car_inner_ang_vel * cnt * 0.02
            temp_pose.position.x = car_inner_radius * math.sin(car_inner_ang)
            temp_pose.position.y = -car_inner_radius * math.cos(car_inner_ang)
            temp_pose.orientation.w = car_inner_ang
            poses.poses.append(temp_pose)
            car_inner_odom.pose.pose = temp_pose
            temp_twist = Twist()
            temp_twist.linear.x = car_inner_vel * math.cos(car_inner_ang)
            temp_twist.linear.y = car_inner_vel * math.sin(car_inner_ang)
            truck_odom.twist.twist = temp_twist
            car_inner_odom_pub.publish(car_inner_odom)
        ## car big
        if has_car_outter:
            temp_pose = Pose()
            car_outter_ang = car_outter_ang_vel * cnt * 0.02
            temp_pose.position.x = car_outter_radius * math.sin(car_outter_ang)
            temp_pose.position.y = -car_outter_radius * math.cos(car_outter_ang)
            temp_pose.orientation.w = car_outter_ang
            poses.poses.append(temp_pose)
            car_outter_odom.pose.pose = temp_pose
            temp_twist = Twist()
            temp_twist.linear.x = car_outter_vel * math.cos(car_outter_ang)
            temp_twist.linear.y = car_outter_vel * math.sin(car_outter_ang)
            truck_odom.twist.twist = temp_twist
            car_outter_odom_pub.publish(car_outter_odom)

        cars_poses_pub.publish(poses)
        cnt += 1
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
