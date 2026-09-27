#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math

import rclpy
from rclpy.node import Node

import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

PUBLISH_TF = False


class BaseOdometry(Node):

    def __init__(self):
        super().__init__('robot_base_odometry')
        self.initialized = False

        # get joints name and characteristics
        self.l_joint = "left_tread"
        self.r_joint = "right_tread"
        self.tread_radius = 0.025
        self.tread_basis = 0.055

        # joint interaction
        self.create_subscription(JointState, 'joint_states', self.jnt_state_cb, 10)

        # tf broadcaster
        if PUBLISH_TF:
            self.br = tf2_ros.TransformBroadcaster(self)

        # publish results on topic
        self.pub = self.create_publisher(Odometry, 'odom', 10)
        self.initialized = False

    def jnt_state_cb(self, msg):
        # creates map
        position = {}
        for name, pos in zip(msg.name, msg.position):
            position[name] = pos

        # initialize
        if not self.initialized:
            self.r_pos = position[self.r_joint]
            self.l_pos = position[self.l_joint]
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.initialized = True
        else:
            delta_r_pos = position[self.r_joint] - self.r_pos
            delta_l_pos = position[self.l_joint] - self.l_pos
            delta_trans = (delta_r_pos + delta_l_pos) * self.tread_radius / 2.0
            delta_rot = (delta_r_pos - delta_l_pos) * self.tread_radius / (2.0 * self.tread_basis)
            self.r_pos = position[self.r_joint]
            self.l_pos = position[self.l_joint]

            # addDelta(self.pose, self.pose.M * twist) equivalent, applied on the robot frame
            self.x += delta_trans * math.cos(self.theta)
            self.y += delta_trans * math.sin(self.theta)
            self.theta += delta_rot

            if PUBLISH_TF:
                # Create transform message
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = "odom"
                t.child_frame_id = "base_link"
                t.transform.translation.x = self.x
                t.transform.translation.y = self.y
                t.transform.translation.z = 0.0
                t.transform.rotation.z = math.sin(self.theta / 2.0)
                t.transform.rotation.w = math.cos(self.theta / 2.0)
                self.br.sendTransform(t)

            self.rot_covar = 1.0
            if delta_rot == 0:
                self.rot_covar = 0.00000000001

            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
            odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
            odom.pose.covariance = [0.00001, 0, 0, 0, 0, 0,
                                    0, 0.00001, 0, 0, 0, 0,
                                    0, 0, 10.0000, 0, 0, 0,
                                    0, 0, 0, 1.00000, 0, 0,
                                    0, 0, 0, 0, 1.00000, 0,
                                    0, 0, 0, 0, 0, self.rot_covar]
            self.pub.publish(odom)


def main():
    rclpy.init()
    base_odometry = BaseOdometry()
    rclpy.spin(base_odometry)
    base_odometry.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
