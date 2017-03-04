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

        ## Initial Process
        ### DJI bringup
        self.__drone = DJIDrone()
        self.__drone.request_sdk_permission_control()
        rospy.loginfo("Use DJI")

        ## ROS Param
        self.__uav_global_coordinate_control_flag = rospy.get_param("~global_coordinate_control", "True")
        ### Topic Name
        self.__control_velocity_sub_topic_name = rospy.get_param("~control_velocity_sub_topic_name", "/cmd_vel")

        self.__start_control = rospy.get_param("~start_control", True)
        print self.__start_control

        if self.__uav_global_coordinate_control_flag:
            rospy.loginfo("uav global coordinate control mode")
        else:
            rospy.loginfo("uav local coordinate control mode")

        ## Subscriber
        self.__subscriber_uav_command = rospy.Subscriber(self.__control_velocity_sub_topic_name, Twist, self.__control_velocity_callback)

        ## Publisher
        self.__publisher_uav_control = rospy.Publisher("/control_ready", Empty, queue_size = 1)
        time.sleep(0.5)
        msg = Empty()
        self.__publisher_uav_control.publish(msg)

    def __activate_on_signal_callback(self, msg):
        if msg.data:
            print "planning tracking mode: on"
            self.__start_control = True
        else:
            print "planning tracking mode: off"
            self.__start_control = False

    # Callback Func
    def __control_velocity_callback(self, msg):
        if not self.__start_control:
            return

        if self.__uav_global_coordinate_control_flag:
            self.__drone.velocity_control(1, msg.linear.x, msg.linear.y, msg.linear.z, 0.0)
        else:
            self.__drone.velocity_control(0, msg.linear.x, msg.linear.y, msg.linear.z, 0.0)

if __name__ == '__main__':
    try:
        myUav = uav_states()
        myUav.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
