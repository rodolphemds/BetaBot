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

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nxt_msgs.msg import JointCommand


class BaseController(Node):

    def __init__(self):
        super().__init__('robot_base_controller')
        self.initialized = False
        self.vel_rot_desi = 0
        self.vel_trans_desi = 0
        self.vel_trans = 0
        self.vel_rot = 0
        self.brake = False

        # get joints name and characteristics
        self.l_joint = "left_tread"
        self.r_joint = "right_tread"
        self.tread_radius = 0.025
        self.tread_basis = 0.055
        self.vel_to_eff = 0.5
        self.k_rot = 0.075 / self.vel_to_eff
        self.k_trans = 0.055 / self.vel_to_eff

        # joint interaction
        self.pub = self.create_publisher(JointCommand, 'joint_command', 10)
        self.create_subscription(JointState, 'joint_states', self.jnt_state_cb, 10)

        # base commands
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Bool, 'cmd_base_brake', self.brake_cb, 10)

    def brake_cb(self, msg):
        self.brake = msg.data

    def cmd_vel_cb(self, msg):
        self.vel_rot_desi = msg.angular.z
        self.vel_trans_desi = msg.linear.x

    def jnt_state_cb(self, msg):
        velocity = {}
        for name, vel in zip(msg.name, msg.velocity):
            velocity[name] = vel

        # lowpass for measured velocity
        self.vel_trans = 0.5 * self.vel_trans + 0.5 * (velocity[self.r_joint] + velocity[self.l_joint]) * self.tread_radius / 2.0
        self.vel_rot = 0.5 * self.vel_rot + 0.5 * (velocity[self.r_joint] - velocity[self.l_joint]) * self.tread_radius / (2.0 * self.tread_basis)

        # velocity commands
        vel_trans = self.vel_trans_desi + self.k_trans * (self.vel_trans_desi - self.vel_trans)
        vel_rot = self.vel_rot_desi + self.k_rot * (self.vel_rot_desi - self.vel_rot)

        # tread commands
        l_cmd = JointCommand()
        l_cmd.name = self.l_joint
        l_cmd.header.stamp = self.get_clock().now().to_msg()
        l_cmd.header.frame_id = "left_tread_command"
        l_cmd.brake = self.brake
        l_cmd.effort = self.vel_to_eff * (vel_trans / self.tread_radius - vel_rot * self.tread_basis / self.tread_radius)
        self.pub.publish(l_cmd)
        if l_cmd.effort != 0:
            self.get_logger().info('Moving left_tread (commanding effort : %f, brake : %f).' % (l_cmd.effort, float(l_cmd.brake)))

        r_cmd = JointCommand()
        r_cmd.name = self.r_joint
        r_cmd.header.stamp = self.get_clock().now().to_msg()
        r_cmd.header.frame_id = "right_tread_command"
        r_cmd.brake = self.brake
        r_cmd.effort = self.vel_to_eff * (vel_trans / self.tread_radius + vel_rot * self.tread_basis / self.tread_radius)
        self.pub.publish(r_cmd)
        if r_cmd.effort != 0:
            self.get_logger().info('Moving right_tread (commanding effort : %f, brake : %f).' % (r_cmd.effort, float(r_cmd.brake)))


def main():
    rclpy.init()
    base_controller = BaseController()
    rclpy.spin(base_controller)
    base_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
