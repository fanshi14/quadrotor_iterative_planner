#!/usr/bin/env python

# Copyright (c) 2016, JSK(University of Tokyo)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Open Source Robotics Foundation, Inc.
#       nor the names of its contributors may be used to endorse or promote
#       products derived from this software without specific prior
#       written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Authors: Fan Shi, Moju Zhao
# Maintainer: Fan Shi <shifan@jsk.imi.i.u-tokyo.ac.jp> 

import time
import sys
import math
import rospy
import tf
import numpy as np
from geometry_msgs.msg import TwistStamped, Twist
from std_msgs.msg import Float64, Empty, Bool
from dji_sdk.dji_drone import DJIDrone
import dynamic_reconfigure.server


class uav_states:

    def init(self):
        ### Flag for DJI M100
        if len(sys.argv) != 4:
            self.__use_dji = False
        else:
            if sys.argv[1] == 'True':
                self.__use_dji = True
            else:
                self.__use_dji = False

        if self.__use_dji == False:
            rospy.init_node('uav_states', anonymous=True)

        ## Initial Process
        ### DJI bringup
        if self.__use_dji == True:
            self.__drone = DJIDrone()
            self.__drone.request_sdk_permission_control()
            rospy.loginfo("Use DJI")

        ## ROS Param
        self.__uav_global_coordinate_control_flag = rospy.get_param("~global_coordinate_control", "True")
        ### Topic Name
        self.__control_velocity_sub_topic_name = rospy.get_param("~control_velocity_sub_topic_name", "/cmd_vel")

        if self.__uav_global_coordinate_control_flag:
            rospy.loginfo("uav global coordinate control mode")
        else:
            rospy.loginfo("uav local coordinate control mode")

        ## Subscriber
        self.__subscriber_uav_command = rospy.Subscriber(self.__control_velocity_sub_topic_name, Twist, self.__control_velocity_callback)

        time.sleep(0.5)

    # Callback Func
    def __control_velocity_callback(self, msg):
        if self.__uav_global_coordinate_control_flag:
            self.__drone.velocity_control(1, msg.linear.x, msg.linear.y, msg.linear.z, self.__control_yaw_ang)
        else:
            self.__drone.velocity_control(0, msg.linear.x, msg.linear.y, msg.linear.z, self.__control_yaw_ang)

    # Main Process
    def __control_callback(self, event):
        if not self.__object_pos_update:
            return
        
        control_veloicty_msg = TwistStamped()
        raw_control_vel = np.array([0, 0, 0])
        yaw_rotation_angle = 0
        
        ## UAV state transformation depends on current position
        ## when state is TARGET_LOST
        if self.__uav_state_machine == self.__TARGET_LOST:
            if self.__object_pos[2] >= 0.0:
                if self.__object_pos[2] < self.__force_landing_height_upperbound:
                    self.__uav_state_machine = self.__WAITING_LANDING
                    rospy.loginfo("UAV is waiting for landing.")
                else:
                    self.__uav_state_machine = self.__TRACKING_TARGET
        ## when state is TRACKING_TARGET
        elif self.__uav_state_machine == self.__TRACKING_TARGET:
            if self.__object_pos[2] >= 0:
                if self.__object_pos[2] < self.__force_landing_height_upperbound:
                    self.__uav_state_machine = self.__WAITING_LANDING
                    rospy.loginfo("UAV is waiting for landing.")
            else:
                rospy.logwarn("tracking_landing_control: tracking target phase lost target, z: %f", self.__object_pos[2])
                self.__uav_state_machine = self.__TARGET_LOST
                self.__tracking_goingdown_cnt = 0
        ## when state is TRACKING_GOING_DOWN
        elif self.__uav_state_machine == self.__TRACKING_GOING_DOWN:
            if self.__object_pos[2] >= 0:
                if self.__object_pos[2] < self.__force_landing_height_upperbound:
                    self.__uav_state_machine = self.__WAITING_LANDING
                    rospy.loginfo("UAV is waiting for landing.")
            else:
                rospy.logwarn("tracking_landing_control: tracking target phase lost target, z: %f", self.__object_pos[2])
                self.__uav_state_machine = self.__TARGET_LOST
                self.__tracking_goingdown_cnt = 0

                
        ## when state is WAITING_LANDING
        ## here is "if" instead of "elif", because the previous state could be transformed to be WAITING_LANDING state
        if self.__uav_state_machine == self.__WAITING_LANDING:
            if self.__object_pos[2] >= 0:
                if self.__object_pos[2] > self.__odometry_reliable_height_lowerbound:
                    self.__uav_state_machine = self.__TRACKING_TARGET
                    self.__tracking_goingdown_cnt = 0
                    self.__force_landing_cnt = 0
                else:
                    ## when uav is in the square neighbor region of target, just force land
                    if abs(self.__object_pos[0]) < 0.13 and abs(self.__object_pos[1] < 0.1):
                        self.__force_landing_cnt += 1
                        if self.__force_landing_cnt > self.__control_frequency * 0.2:
                            self.__uav_state_machine = self.__FORCE_LANDING
            else:
                self.__uav_state_machine = self.__TARGET_LOST
                rospy.logwarn("tracking_landing_control: waiting landing phase lost target, z: %f", self.__object_pos[2])
                self.__tracking_goingdown_cnt = 0
                self.__force_landing_cnt = 0

        ## State transfer between TRACKING_TARGET and TRACKING_GOING_DOWN
        if self.__uav_state_machine == self.__TRACKING_TARGET:
            threshold_x = np.clip(self.__object_pos[2]*0.07*1.5, 0.2, 0.4)
            threshold_y = np.clip(self.__object_pos[2]*0.07, 0.15, 0.35)
            
            if abs(self.__object_pos[0]) < threshold_x and abs(self.__object_pos[1]) < threshold_y:
                self.__tracking_goingdown_cnt += 1
                if self.__tracking_goingdown_cnt > self.__control_frequency * 0.4:
                    rospy.logwarn("TRACKING_GOING_DOWN: x: %f, y: %f, z: %f", self.__object_pos[0], self.__object_pos[1], self.__object_pos[2])
                    self.__uav_state_machine = self.__TRACKING_GOING_DOWN
                
        ## UAV control strategy for each state
        ## when state is FINISH_LANDING
        if self.__uav_state_machine == self.__FINISH_LANDING:
            self.__control_vel[0] = 0
            self.__control_vel[1] = 0
            self.__control_vel[2] = 0
            self.__control_yaw_ang = 0
        ## when state is TARGET_LOST
        elif self.__uav_state_machine == self.__TARGET_LOST:
            self.__control_vel[0] = 0
            self.__control_vel[1] = 0
            self.__control_vel[2] = 0
            self.__control_yaw_ang = 0
        else:
            ## States needing complex control
            ## P control : vel = p_gain * pos_err
            ## CAUTION: BE CAREFULL THAT THE TARGET_POS DENOTES TO THE POSITION OF OBJECT IN TERMS OF LEVEL CAMERA FRAME, NOT THE UAV POSITION IN TERMS OF WORLD FRAME WITH OBJECT ORIGIN
            ## when pitch-yaw control
            if self.__uav_pitch_yaw_control_flag:
                diff_pos = self.__object_pos - self.__target_pos
                raw_control_vel = np.array([self.__p_gain * math.sqrt(diff_pos[0] ** 2 + diff_pos[1] ** 2), 0, 0])
                raw_control_yaw_ang = math.atan2(diff_pos[1], diff_pos[0])/math.pi * 180.0
                self.__control_yaw_ang = self.__p_yaw_gain * raw_control_yaw_ang
            ## when roll-pitch control
            else:
                ## feedforward
                feed_forward_vel = np.array([0, 0, 0])
                if self.__feedforward:
                    abs_vel = np.linalg.norm(self.__object_vel)
                    if abs_vel > self.__feedforward_vel_lowerbound:
                        if self.__feed_forward_gain_changing:
                            if self.__feed_forward_gain > self.__feedforward_lowest_gain:
                                self.__feed_forward_gain -= ((1 - self.__feedforward_lowest_gain) / self.__feedforward_chagin_du / self.__control_frequency)
                                #rospy.loginfo("feedforward gain: %f", self.__feed_forward_gain)
                            if self.__uav_state_machine == self.__FORCE_LANDING:
                                self.__feed_forward_gain = 0.6
                        feed_forward_vel = self.__feed_forward_gain * self.__object_vel
                        if abs_vel > self.__feedforward_vel_upperbound:
                            feed_forward_vel = self.__feedforward_vel_upperbound * self.__object_vel / abs_vel
                    #print np.linalg.norm(feed_forward_vel)
                
                ## P term
                p_term = np.clip(self.__p_gain * (self.__object_pos - self.__target_pos), -self.__max_p_term, self.__max_p_term)

                ## I term
                self.__i_control_accumulation += (self.__object_pos - self.__target_pos) * 1.0 / self.__control_frequency
                if self.__i_control_accumulation[0] * self.__i_gain > self.__max_i_term:
                   self.__i_control_accumulation[0] = self.__max_i_term / self.__i_gain
                elif self.__i_control_accumulation[0] * self.__i_gain < -self.__max_i_term:
                   self.__i_control_accumulation[0] = -self.__max_i_term / self.__i_gain
                if self.__i_control_accumulation[1] * self.__i_gain > self.__max_i_term:
                   self.__i_control_accumulation[1] = self.__max_i_term / self.__i_gain
                elif self.__i_control_accumulation[1] * self.__i_gain < -self.__max_i_term:
                   self.__i_control_accumulation[1] = -self.__max_i_term / self.__i_gain
                i_term = self.__i_gain * self.__i_control_accumulation

                ## Summerize
                raw_control_vel = p_term + i_term + feed_forward_vel

                self.__control_yaw_ang = 0.0

            self.__control_vel = np.clip(raw_control_vel, -self.__max_control_vel, self.__max_control_vel)

            ## when state is TRACKING_TARGET
            if self.__uav_state_machine == self.__TRACKING_TARGET:
                self.__control_vel[2] = 0

            ## when state is TRACKING_GOING_DOWN
            elif self.__uav_state_machine == self.__TRACKING_GOING_DOWN:
                self.__control_vel[2] = self.__landing_speed_in_tracking

            ## when state is WAITING_LANDING
            elif self.__uav_state_machine == self.__WAITING_LANDING:
                if self.__object_pos[2] < self.__force_landing_height_upperbound - 0.2:
                    self.__control_vel[2] = 0.2
                elif self.__object_pos[2] > self.__force_landing_height_upperbound + 0.2:
                    self.__control_vel[2] = -0.2
                else:
                    self.__control_vel[2] = 0

            ## when state is FORCE_LANDING
            elif self.__uav_state_machine == self.__FORCE_LANDING:
                self.__control_vel[2] = self.__landing_speed_in_force_landing
                if self.__use_dji == True:
                    for i in range(0, 100):
                        self.__drone.velocity_control(0, self.__control_vel[0], self.__control_vel[1], self.__control_vel[2], self.__control_yaw_ang)
                        time.sleep(0.02)
                    self.__drone.disarm_drone()
                    time.sleep(0.02)
                    for i in range(0, 100):
                        self.__drone.velocity_control(0, 0, 0, self.__control_vel[2], self.__control_yaw_ang)
                        time.sleep(0.02)
                    self.__drone.disarm_drone()
                    time.sleep(0.02)
                    self.__drone.landing()
                    self.__uav_state_machine = self.__FINISH_LANDING
                rospy.loginfo("UAV is landing.")
                return

        ## DJI
        if(self.__use_dji):
            self.__drone.velocity_control(0, self.__control_vel[0], self.__control_vel[1] , self.__control_vel[2], self.__control_yaw_ang)

        control_veloicty_msg.header.stamp = rospy.Time.now()
        control_veloicty_msg.twist.linear.x = self.__control_vel[0]
        control_veloicty_msg.twist.linear.y = self.__control_vel[1]
        control_veloicty_msg.twist.linear.z = self.__control_vel[2]
        control_veloicty_msg.twist.angular.z = self.__control_yaw_ang

        ## publish part
        self.__publisher_control_velocity.publish(control_veloicty_msg)
        uav_state_machine_msg = PointStamped()
        uav_state_machine_msg.header.stamp = rospy.Time.now()
        uav_state_machine_msg.point.x = self.__uav_state_machine 
        self.__publisher_uav_state_machine.publish(uav_state_machine_msg)

if __name__ == '__main__':
    try:
        myUav = uav_states()
        myUav.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
