#!/usr/bin/env python
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
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

import rclpy
from geometry_msgs.msg import Twist

import sys, select, os
import curses

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

MAX_LIN_VEL = 0.22
MAX_ANG_VEL = 2.84
LIN_VEL_STEP_SIZE = MAX_LIN_VEL / 100
ANG_VEL_STEP_SIZE = MAX_ANG_VEL / 100

LEFT_KEY = curses.KEY_LEFT
RIGHT_KEY = curses.KEY_RIGHT
UP_KEY = curses.KEY_UP
DOWN_KEY = curses.KEY_DOWN
STOP_KEY = " "

# get the curses screen window
screen = curses.initscr()
# turn off input echoing
curses.noecho()
# respond to keys immediately (don't wait for enter)
curses.cbreak()
# map arrow keys to special values
screen.keypad(True)

msg = """
-- control the robot base ------------------------------

Moving around:
        \u2191   
   \u2190         \u2192
        \u2193   

up/down arrow keys : increase/decrease linear velocity
left/right arrow keys : increase/decrease angular velocity
space key : force stop
CTRL-C to quit
"""

e = """
Communication error.
"""


def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel, target_angular_vel)


def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input
    return output


def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input
    return input


def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)
    return vel


def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -MAX_ANG_VEL, MAX_ANG_VEL)
    return vel


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node('base_enhanced_teleop')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)
    status = 0
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    control_linear_vel = 0.0
    control_angular_vel = 0.0

    try:
        screen.addstr(msg)  # print doesn't work with curses, use addstr instead
        while rclpy.ok():
            key = screen.getch()
            if key == ord(STOP_KEY):
                target_linear_vel = 0.0
                control_linear_vel = 0.0
                target_angular_vel = 0.0
                control_angular_vel = 0.0
                screen.addstr(vels(target_linear_vel, target_angular_vel))
            else:
                if key == UP_KEY:
                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                    status = status + 1
                    screen.addstr(vels(target_linear_vel, target_angular_vel))
                elif key == DOWN_KEY:
                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                    status = status + 1
                    screen.addstr(vels(target_linear_vel, target_angular_vel))
                elif key == LEFT_KEY:
                    target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                    status = status + 1
                    screen.addstr(vels(target_linear_vel, target_angular_vel))
                elif key == RIGHT_KEY:
                    target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                    status = status + 1
                    screen.addstr(vels(target_linear_vel, target_angular_vel))

            twist = Twist()
            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE / 2.0))
            twist.linear.x = control_linear_vel
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE / 2.0))
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_vel
            pub.publish(twist)
    except:
        screen.addstr(e)
    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        curses.nocbreak()
        screen.keypad(0)
        curses.echo()
        curses.endwin()
        node.destroy_node()
        rclpy.shutdown()
