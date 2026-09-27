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
import sys
import select
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

if os.name == 'nt':
    import msvcrt
else:
    import tty
    import termios

msg = """
------------------------------- Joints controller ---- ---------------------------
Head  >>  a/e : move left/right | z : center head
Torso >>  p/m : move up/down    | o : reset position
Laser >>  w/c : deploy/retract
Arms  >>  q/d : run motor up/down | f : rotate forearms | g/h : open/close hands
        | j : rotate wrists | s : reset position
CTRL-C to quit
"""

e = """
Communications Failed
"""

HEAD_CENTER = 0 * math.pi / 180
HEAD_EFFORT = 0.75
HEAD_STEP = 1  # min accuracy = 0.9 rad

TORSO_RESET = 250 * math.pi / 180
TORSO_EFFORT = 1
TORSO_STEP = 1

LASER_EFFORT = 0.5

ARMS_FOREARMS_UP = 250 * math.pi / 180
ARMS_HANDS_CLOSED = 180 * math.pi / 180
ARMS_HANDS_OPEN = 0
ARMS_WRIST_ROTATED = -140 * math.pi / 180
ARMS_RESET = ARMS_HANDS_OPEN
ARMS_EFFORT = 0.5
ARMS_STEP = 1

arms_pos = 0.0
head_pos = 0.0
laser_pos = 0.0
torso_pos = 0.0

settings = None


def getKey():
    if os.name == 'nt':
        return msvcrt.getch().decode()
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class JointsTeleop(Node):

    def __init__(self):
        super().__init__('joints_teleop')
        self.arms_pos = 0.0
        self.head_pos = 0.0
        self.laser_pos = 0.0
        self.torso_pos = 0.0

        # setting up publishers
        self.cmd_arms_joint_pub = self.create_publisher(JointState, "cmd_arms_joint", 10)
        self.cmd_arms_joint_msg = JointState()
        self.cmd_arms_joint_msg.header.frame_id = "arms_teleop"
        self.cmd_arms_joint_msg.header.stamp = self.get_clock().now().to_msg()
        self.cmd_arms_joint_msg.name = ["arms_joint"]
        self.cmd_arms_joint_msg.position = [0.0]
        self.cmd_arms_joint_msg.effort = [0.0]

        self.cmd_head_joint_pub = self.create_publisher(JointState, "cmd_head_joint", 10)
        self.cmd_head_joint_msg = JointState()
        self.cmd_head_joint_msg.header.frame_id = "head_teleop"
        self.cmd_head_joint_msg.header.stamp = self.get_clock().now().to_msg()
        self.cmd_head_joint_msg.name = ["head_joint"]
        self.cmd_head_joint_msg.position = [0.0]
        self.cmd_head_joint_msg.effort = [0.0]

        self.cmd_torso_joint_pub = self.create_publisher(JointState, "cmd_torso_joint", 10)
        self.cmd_torso_joint_msg = JointState()
        self.cmd_torso_joint_msg.header.frame_id = "torso_teleop"
        self.cmd_torso_joint_msg.header.stamp = self.get_clock().now().to_msg()
        self.cmd_torso_joint_msg.name = ["torso_joint"]
        self.cmd_torso_joint_msg.position = [0.0]
        self.cmd_torso_joint_msg.effort = [0.0]

        self.cmd_laser_joint_pub = self.create_publisher(JointState, "cmd_laser_joint", 10)
        self.cmd_laser_joint_msg = JointState()
        self.cmd_laser_joint_msg.header.frame_id = "laser_teleop"
        self.cmd_laser_joint_msg.header.stamp = self.get_clock().now().to_msg()
        self.cmd_laser_joint_msg.name = ["laser_joint"]
        self.cmd_laser_joint_msg.position = [0.0]
        self.cmd_laser_joint_msg.effort = [0.0]

        # getting joint position
        self.create_subscription(JointState, 'joint_states', self.get_pos_cb, 10)

    def get_pos_cb(self, jnt_state):
        for name, pos, vel, eff in zip(jnt_state.name, jnt_state.position,
                                       jnt_state.velocity, jnt_state.effort):
            if name == "arms_joint":
                self.arms_pos = pos
            elif name == "head_joint":
                self.head_pos = pos
            elif name == "torso_joint":
                self.torso_pos = pos
            elif name == "laser_joint":
                self.laser_pos = pos

    def run(self):
        status = 0
        try:
            print(msg)
            while rclpy.ok():
                key = getKey()
                # head
                if key == 'a':  # left
                    status = status + 1
                    self.cmd_head_joint_msg.position = [self.head_pos - HEAD_STEP]
                    self.cmd_head_joint_msg.effort = [HEAD_EFFORT]
                    self.cmd_head_joint_pub.publish(self.cmd_head_joint_msg)
                    print("Head position : " + str(self.head_pos))
                elif key == 'e':  # right
                    status = status + 1
                    self.cmd_head_joint_msg.position = [self.head_pos + HEAD_STEP]
                    self.cmd_head_joint_msg.effort = [HEAD_EFFORT]
                    self.cmd_head_joint_pub.publish(self.cmd_head_joint_msg)
                    print("Head position : " + str(self.head_pos))
                elif key == 'z':  # center
                    status = status + 1
                    self.cmd_head_joint_msg.position = [HEAD_CENTER]
                    self.cmd_head_joint_msg.effort = [HEAD_EFFORT]
                    self.cmd_head_joint_pub.publish(self.cmd_head_joint_msg)
                    print("Head position : " + str(self.head_pos))
                # torso
                elif key == 'p':  # up
                    status = status + 1
                    self.cmd_torso_joint_msg.position = [self.torso_pos - TORSO_STEP]
                    self.cmd_torso_joint_msg.effort = [TORSO_EFFORT]
                    self.cmd_torso_joint_pub.publish(self.cmd_torso_joint_msg)
                    print("Torso position : " + str(self.torso_pos))
                elif key == 'm':  # down
                    status = status + 1
                    self.cmd_torso_joint_msg.position = [self.torso_pos + TORSO_STEP]
                    self.cmd_torso_joint_msg.effort = [TORSO_EFFORT]
                    self.cmd_torso_joint_pub.publish(self.cmd_torso_joint_msg)
                    print("Torso position : " + str(self.torso_pos))
                elif key == 'o':  # reset
                    status = status + 1
                    self.cmd_torso_joint_msg.position = [TORSO_RESET]
                    self.cmd_torso_joint_msg.effort = [TORSO_EFFORT]
                    self.cmd_torso_joint_pub.publish(self.cmd_torso_joint_msg)
                    print("Torso position : " + str(self.torso_pos))
                # laser
                elif key == 'w':  # up
                    status = status + 1
                    self.cmd_laser_joint_msg.position = [2]
                    self.cmd_laser_joint_msg.effort = [LASER_EFFORT]
                    self.cmd_laser_joint_pub.publish(self.cmd_laser_joint_msg)
                    print("Laser position : " + str(self.laser_pos))
                elif key == 'c':  # down
                    status = status + 1
                    self.cmd_laser_joint_msg.position = [-0.3]
                    self.cmd_laser_joint_msg.effort = [LASER_EFFORT]
                    self.cmd_laser_joint_pub.publish(self.cmd_laser_joint_msg)
                    print("Laser position : " + str(self.laser_pos))
                # arms
                elif key == 'q':  # up
                    status = status + 1
                    self.cmd_arms_joint_msg.position = [self.arms_pos + ARMS_STEP]
                    self.cmd_arms_joint_msg.effort = [ARMS_EFFORT]
                    self.cmd_arms_joint_pub.publish(self.cmd_arms_joint_msg)
                    print("Arms position : " + str(self.arms_pos))
                elif key == 'd':  # down
                    status = status + 1
                    self.cmd_arms_joint_msg.position = [self.arms_pos - ARMS_STEP]
                    self.cmd_arms_joint_msg.effort = [ARMS_EFFORT]
                    self.cmd_arms_joint_pub.publish(self.cmd_arms_joint_msg)
                    print("Arms position : " + str(self.arms_pos))
                elif key == 'f':  # rotate forearms
                    status = status + 1
                    self.cmd_arms_joint_msg.position = [ARMS_FOREARMS_UP]
                    self.cmd_arms_joint_msg.effort = [ARMS_EFFORT]
                    self.cmd_arms_joint_pub.publish(self.cmd_arms_joint_msg)
                    print("Arms position : " + str(self.arms_pos))
                elif key == 'g':  # open hands
                    status = status + 1
                    self.cmd_arms_joint_msg.position = [ARMS_HANDS_OPEN]
                    self.cmd_arms_joint_msg.effort = [ARMS_EFFORT]
                    self.cmd_arms_joint_pub.publish(self.cmd_arms_joint_msg)
                    print("Arms position : " + str(self.arms_pos))
                elif key == 'h':  # close hands
                    status = status + 1
                    self.cmd_arms_joint_msg.position = [ARMS_HANDS_CLOSED]
                    self.cmd_arms_joint_msg.effort = [ARMS_EFFORT]
                    self.cmd_arms_joint_pub.publish(self.cmd_arms_joint_msg)
                    print("Arms position : " + str(self.arms_pos))
                elif key == 'j':  # rotate wrists
                    status = status + 1
                    self.cmd_arms_joint_msg.position = [ARMS_WRIST_ROTATED]
                    self.cmd_arms_joint_msg.effort = [ARMS_EFFORT]
                    self.cmd_arms_joint_pub.publish(self.cmd_arms_joint_msg)
                    print("Arms position : " + str(self.arms_pos))
                elif key == 's':  # reset
                    status = status + 1
                    self.cmd_arms_joint_msg.position = [ARMS_RESET]
                    self.cmd_arms_joint_msg.effort = [ARMS_EFFORT]
                    self.cmd_arms_joint_pub.publish(self.cmd_arms_joint_msg)
                    print("Arms position : " + str(self.arms_pos))
                else:
                    if key == '\x03':
                        break
                if status == 15:
                    print(msg)
                    status = 0
                rclpy.spin_once(self, timeout_sec=0)
        except Exception:
            print(e)


def main():
    global settings
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = JointsTeleop()
    node.run()
    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
