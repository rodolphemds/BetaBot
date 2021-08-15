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

import roslib; roslib.load_manifest('joints_teleop')
import rospy
import math
import thread
import subprocess
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from PyKDL import Rotation
from time import sleep
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

msg = """
---------------------------
---- Joints controller ---- 
---------------------------
Head  
>>  a/e : move left/right | z : center head
Torso 
>>  p/m : move up/down | o : reset position
Laser 
>>  w/c : deploy/retract
Arms  
>>  q/d : run motor up/down | f : rotate forearms | g/h : open/close hands | j :rotate wrists | s : reset position

CTRL-C to quit
"""

e = """
Communications Failed
"""

HEAD_CENTER = 0 * math.pi/180
HEAD_EFFORT = 0.75
HEAD_STEP = 1 # min accuracy = 0.9 rad

TORSO_RESET = 250 * math.pi/180
TORSO_EFFORT = 1
TORSO_STEP = 1

LASER_EFFORT = 0.5

ARMS_FOREARMS_UP = 250 * math.pi/180
ARMS_HANDS_CLOSED = 180 * math.pi/180
ARMS_HANDS_OPEN = 0
ARMS_WRIST_ROTATED = -140 * math.pi/180
ARMS_RESET = ARMS_HANDS_OPEN
ARMS_EFFORT = 0.5
ARMS_STEP = 1

armsPos = 0
headPos = 0
laserPos = 0
torsoPos = 0

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def getPos_cb(jnt_state):
    for name, pos, vel, eff in zip(jnt_state.name, jnt_state.position, jnt_state.velocity, jnt_state.effort):
        if name == "arms_joint":
            global armsPos
            armsPos = pos
        elif name == "head_joint":
            global headPos
            headPos = pos
        elif name == "torso_joint":
            global torsoPos
            torsoPos = pos
        elif name == "laser_joint":
            global laserPos
            laserPos = pos


if __name__=="__main__":

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('joints_teleop')

    # setting up publishers
    cmd_arms_joint_pub = rospy.Publisher("cmd_arms_joint", JointState, queue_size=10)
    cmd_arms_joint_msg = JointState()
    cmd_arms_joint_msg.header.frame_id = "arms_teleop"
    cmd_arms_joint_msg.header.stamp = rospy.Time.now()
    cmd_arms_joint_msg.name = ("arms_joint",) # must be a tuple not a string
    cmd_arms_joint_msg.position = (0,)
    cmd_arms_joint_msg.effort = (0,)

    cmd_head_joint_pub = rospy.Publisher("cmd_head_joint", JointState, queue_size=10)
    cmd_head_joint_msg = JointState()
    cmd_head_joint_msg.header.frame_id = "head_teleop"
    cmd_head_joint_msg.header.stamp = rospy.Time.now()
    cmd_head_joint_msg.name = ("head_joint",)
    cmd_head_joint_msg.position = (0,)
    cmd_head_joint_msg.effort = (0,)

    cmd_torso_joint_pub = rospy.Publisher("cmd_torso_joint", JointState, queue_size=10)
    cmd_torso_joint_msg = JointState()
    cmd_torso_joint_msg.header.frame_id = "torso_teleop"
    cmd_torso_joint_msg.header.stamp = rospy.Time.now()
    cmd_torso_joint_msg.name = ("torso_joint",)
    cmd_torso_joint_msg.position = (0,)
    cmd_torso_joint_msg.effort = (0,)

    cmd_laser_joint_pub = rospy.Publisher("cmd_laser_joint", JointState, queue_size=10)
    cmd_laser_joint_msg = JointState()
    cmd_laser_joint_msg.header.frame_id = "laser_teleop"
    cmd_laser_joint_msg.header.stamp = rospy.Time.now()
    cmd_laser_joint_msg.name = ("laser_joint",)
    cmd_laser_joint_msg.position = (0,)
    cmd_laser_joint_msg.effort = (0,)

    # getting joint position
    rospy.Subscriber('joint_states', JointState, getPos_cb)

    status = 0

    try :
        print(msg)
        while(1):
            key = getKey()
            # head
            if key == 'a' : # left
                status = status + 1
                cmd_head_joint_msg.position = (headPos - HEAD_STEP,)
                cmd_head_joint_msg.effort = (HEAD_EFFORT,)
                cmd_head_joint_pub.publish(cmd_head_joint_msg)
                print("Head position : " + str(headPos))
            elif key == 'e' : # right
                status = status + 1
                cmd_head_joint_msg.position = (headPos + HEAD_STEP,)
                cmd_head_joint_msg.effort = (HEAD_EFFORT,)
                cmd_head_joint_pub.publish(cmd_head_joint_msg)
                print("Head position : " + str(headPos))
            elif key == 'z' : # center
                status = status + 1
                cmd_head_joint_msg.position = (HEAD_CENTER,)
                cmd_head_joint_msg.effort = (HEAD_EFFORT,)
                cmd_head_joint_pub.publish(cmd_head_joint_msg)
                print("Head position : " + str(headPos))
            # torso
            elif key == 'p' : # up
                status = status + 1
                cmd_torso_joint_msg.position = (torsoPos - TORSO_STEP,)
                cmd_torso_joint_msg.effort = (TORSO_EFFORT,)
                cmd_torso_joint_pub.publish(cmd_torso_joint_msg)
                print("Torso position : " + str(torsoPos))
            elif key == 'm' : # down
                status = status + 1
                cmd_torso_joint_msg.position = (torsoPos + TORSO_STEP,)
                cmd_torso_joint_msg.effort = (TORSO_EFFORT,)
                cmd_torso_joint_pub.publish(cmd_torso_joint_msg)
                print("Torso position : " + str(torsoPos))
            elif key == 'o' : # reset
                status = status + 1
                cmd_torso_joint_msg.position = (TORSO_RESET,)
                cmd_torso_joint_msg.effort = (TORSO_EFFORT,)
                cmd_torso_joint_pub.publish(cmd_torso_joint_msg)
                print("Torso position : " + str(torsoPos))
            # laser
            elif key == 'w' : # up
                status = status + 1
                cmd_laser_joint_msg.position = (2,)
                cmd_laser_joint_msg.effort = (LASER_EFFORT,)
                cmd_laser_joint_pub.publish(cmd_laser_joint_msg)
                print("Laser position : " + str(laserPos))
            elif key == 'c' : # down
                status = status + 1
                cmd_laser_joint_msg.position = (-0.3,)
                cmd_laser_joint_msg.effort = (LASER_EFFORT,)
                cmd_laser_joint_pub.publish(cmd_laser_joint_msg)
                print("Laser position : " + str(laserPos))
            # arms 
            elif key == 'q' : # up
                status = status + 1
                cmd_arms_joint_msg.position = (armsPos + ARMS_STEP,)
                cmd_arms_joint_msg.effort = (ARMS_EFFORT,)
                cmd_arms_joint_pub.publish(cmd_arms_joint_msg)
                print("Arms position : " + str(armsPos))
            elif key == 'd' : # down
                status = status + 1
                cmd_arms_joint_msg.position = (armsPos - ARMS_STEP,)
                cmd_arms_joint_msg.effort = (ARMS_EFFORT,)
                cmd_arms_joint_pub.publish(cmd_arms_joint_msg)
                print("Arms position : " + str(armsPos))
            elif key == 'f' : # rotate forearms
                status = status + 1
                cmd_arms_joint_msg.position = (ARMS_FOREARMS_UP,)
                cmd_arms_joint_msg.effort = (ARMS_EFFORT,)
                cmd_arms_joint_pub.publish(cmd_arms_joint_msg)
                print("Arms position : " + str(armsPos))
            elif key == 'g' : # open hands
                status = status + 1
                cmd_arms_joint_msg.position = (ARMS_HANDS_OPEN,)
                cmd_arms_joint_msg.effort = (ARMS_EFFORT,)
                cmd_arms_joint_pub.publish(cmd_arms_joint_msg)
                print("Arms position : " + str(armsPos))
            elif key == 'h' : # close hands
                status = status + 1
                cmd_arms_joint_msg.position = (ARMS_HANDS_CLOSED,)
                cmd_arms_joint_msg.effort = (ARMS_EFFORT,)
                cmd_arms_joint_pub.publish(cmd_arms_joint_msg)
                print("Arms position : " + str(armsPos))
            elif key == 'j' : # rotate wrists
                status = status + 1
                cmd_arms_joint_msg.position = (ARMS_WRIST_ROTATED,)
                cmd_arms_joint_msg.effort = (ARMS_EFFORT,)
                cmd_arms_joint_pub.publish(cmd_arms_joint_msg)
                print("Arms position : " + str(armsPos))
            elif key == 's' : # reset
                status = status + 1
                cmd_arms_joint_msg.position = (ARMS_RESET,)
                cmd_arms_joint_msg.effort = (ARMS_EFFORT,)
                cmd_arms_joint_pub.publish(cmd_arms_joint_msg)
                print("Arms position : " + str(armsPos))
            
            else:
                if (key == '\x03'):
                    break

            if status == 15 :
                print(msg)
                status = 0
            
    except:
        print(e)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)