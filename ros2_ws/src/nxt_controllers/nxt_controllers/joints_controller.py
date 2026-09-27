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

from sensor_msgs.msg import JointState
from nxt_msgs.msg import JointCommand

head_joint_high = 7  # rad
head_joint_low = -7
torso_joint_low = 0
torso_joint_high = 17
laser_joint_high = 2
laser_joint_low = -0.3
arms_joint_high = 250 * math.pi / 180
arms_joint_low = -140 * math.pi / 180

# accepted rad difference between joint position commanded and actual joint position,
# minimum nxt motor rotation is 50 deg (0.9 rad)
position_accuracy = 0.1

JOINT_LIMITS = {
    "head_joint": (head_joint_low, head_joint_high),
    "torso_joint": (torso_joint_low, torso_joint_high),
    "laser_joint": (laser_joint_low, laser_joint_high),
    "arms_joint": (arms_joint_low, arms_joint_high),
}


def constrain(value, low, high):
    if value < low:
        value = low
    elif value > high:
        value = high
    return value


class JointsController(Node):

    def __init__(self):
        super().__init__('joints_controller')
        self.initialized = False
        self.effort = 0
        self.pos_desi = 0
        # for precise joint control, brake must be "True".
        # If set to "False", the joint controler will be too imprecise and behave strangely.
        self.brake = True
        self.rotations = 0
        self.cmd_sent = False

        # get joint name
        self.declare_parameter('name', 'unknown_joint')
        self.name = self.get_parameter('name').get_parameter_value().string_value

        # joint interaction
        self.pub = self.create_publisher(JointCommand, 'joint_command', 1)
        self.create_subscription(JointState, 'joint_states', self.jnt_state_cb, 10)

        # desired joint position
        self.create_subscription(JointState, "cmd_" + self.name, self.jnt_cmd_cb, 10)

        # define constrains
        if self.name in JOINT_LIMITS:
            self.low, self.high = JOINT_LIMITS[self.name]
        else:
            self.low, self.high = -float('inf'), float('inf')

    def jnt_cmd_cb(self, msg):
        if msg.name[0] == self.name:
            self.pos_desi = constrain(msg.position[0], self.low, self.high)
            self.effort = msg.effort[0]
            self.cmd_sent = False

    def jnt_state_cb(self, msg):
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            if name == self.name and not self.cmd_sent:
                if not self.initialized:
                    self.pos_desi = pos
                    self.initialized = True
                cmd = JointCommand()
                cmd.header.frame_id = name + "_command"
                cmd.header.stamp = self.get_clock().now().to_msg()
                cmd.name = self.name
                cmd.brake = self.brake
                cmd.rotations = abs(self.pos_desi - pos)
                if self.pos_desi >= (pos - position_accuracy) and self.pos_desi <= (pos + position_accuracy):
                    cmd.effort = 0.0
                    cmd.rotations = 0.0
                    self.pub.publish(cmd)
                    self.cmd_sent = True
                elif self.pos_desi < (pos - position_accuracy):
                    cmd.effort = -self.effort
                    self.get_logger().info('Joint %s at %f rad, going to %f rad '
                                            '(commanding effort : %f N, rotations : %f rad).'
                                            % (self.name, pos, self.pos_desi, cmd.effort, cmd.rotations))
                    self.pub.publish(cmd)
                    self.cmd_sent = True
                elif self.pos_desi > (pos + position_accuracy):
                    cmd.effort = self.effort
                    self.get_logger().info('Joint %s at %f rad, going to %f rad '
                                            '(commanding effort : %f N, rotations : %f rad).'
                                            % (self.name, pos, self.pos_desi, cmd.effort, cmd.rotations))
                    self.pub.publish(cmd)
                    self.cmd_sent = True


def main():
    rclpy.init()
    jnt_ctrl = JointsController()
    rclpy.spin(jnt_ctrl)
    jnt_ctrl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
