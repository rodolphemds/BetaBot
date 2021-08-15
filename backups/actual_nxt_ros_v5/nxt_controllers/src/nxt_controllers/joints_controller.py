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

import roslib; roslib.load_manifest('nxt_controllers')
import rospy
import math
import thread
from sensor_msgs.msg import JointState
from nxt_msgs.msg import JointCommand
from std_msgs.msg import Bool, String
from time import sleep

head_joint_high = 7 # rad
head_joint_low = -7
torso_joint_low = 0
torso_joint_high = 17
laser_joint_high = 2
laser_joint_low = -0.3
arms_joint_high = 250 * math.pi/180
arms_joint_low = -140 * math.pi/180

position_accuracy = 0.1 # accepted rad difference between joint position commanded and actual joint position, minimum nxt motor rotation is 50 deg (0.9 rad)

def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input
    return input



class JointsController:
    def __init__(self):
        self.initialized = False
        self.effort = 0
        self.pos_desi = 0
        self.brake = True # for precise joint control, brake must be "True". If set to "False", the joint controler will be too imprecise and behave strangely. 
        self.rotations = 0
        self.cmd_sent = False

        # get joint name
        self.name = rospy.get_param('~name', 'unknown_joint')

        # joint interaction
        self.pub = rospy.Publisher('joint_command', JointCommand, queue_size = 1)
        rospy.Subscriber('joint_states', JointState, self.jnt_state_cb)

        # desired joint position
        rospy.Subscriber("cmd_"+self.name, JointState, self.jnt_cmd_cb)

        # define constrains
        exec("self.low ="+self.name+"_low")
        exec("self.high ="+self.name+"_high")

    def jnt_cmd_cb(self, msg):
        if msg.name[0] == self.name:
            self.pos_desi = constrain(msg.position[0], self.low, self.high)
            self.effort = msg.effort[0]
            self.cmd_sent = False

    def jnt_state_cb(self, msg):
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            if name == self.name and self.cmd_sent == False :
                if not self.initialized:
                    self.pos_desi = pos
                    self.initialized = True
                cmd = JointCommand()
                cmd.header.frame_id=name+"_command"
                cmd.header.stamp=rospy.Time.now()
                cmd.name = self.name
                cmd.brake = self.brake
                cmd.rotations = abs(self.pos_desi-pos)
                if self.pos_desi >= (pos-position_accuracy) and self.pos_desi <= (pos+position_accuracy) :
                    cmd.effort = 0
                    cmd.rotations = 0
                    self.pub.publish(cmd)
                    self.cmd_sent = True
                elif self.pos_desi < (pos-position_accuracy):
                    cmd.effort = - self.effort
                    rospy.loginfo('Joint %s at %f rad, going to %f rad (commanding effort : %f N, rotations : %f rad).'%(self.name, pos, self.pos_desi, cmd.effort, cmd.rotations))
                    self.pub.publish(cmd)
                    self.cmd_sent = True
                elif self.pos_desi > (pos+position_accuracy):
                    cmd.effort = self.effort
                    rospy.loginfo('Joint %s at %f rad, going to %f rad (commanding effort : %f N, rotations : %f rad).'%(self.name, pos, self.pos_desi, cmd.effort, cmd.rotations))
                    self.pub.publish(cmd)
                    self.cmd_sent = True


def main():
    rospy.init_node('joints_controller')
    jnt_ctrl = JointsController()
    rospy.spin()



if __name__ == '__main__':
    main()