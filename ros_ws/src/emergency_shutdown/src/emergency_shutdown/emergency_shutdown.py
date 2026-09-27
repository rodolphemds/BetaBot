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

import roslib; roslib.load_manifest('emergency_shutdown')
import rospy
import math
import threading
import subprocess
from time import sleep 
from std_msgs.msg import Bool, String
import os

def main():
    # Initializing ROS node
    rospy.loginfo("Initializing emergency_shutdown node...")
    rospy.init_node('emergency_shutdown')
    callback_handle_frequency = 10.0
    last_callback_handle = rospy.Time.now()

    last_request = [None]
    def shutdown_cmd(msg):
        last_request[0] = msg.data
        if msg.data == "LOW_BATTERY":
            rospy.logwarn("An emergency shutdown is requested to protect battery life.")
            rospy.logwarn("Please find an other power source within 10 sec to abort robot shutdown.")
            sleep(10)
            if last_request[0] == "LOW_BATTERY":
                rospy.logwarn("Do not forget to manually kill the power switch on the robot base.")
                os.system("shutdown now")
        if msg.data == "ROOT_ORDER":
            rospy.logwarn("An emergency shutdown is commanded by root order.")
            rospy.logwarn('You have 5 sec to stop publishing "ROOT_ORDER" on the emergency_shutdown_request publisher if you want to abort.')
            sleep(5)
            if last_request[0] == "ROOT_ORDER":
                os.system("shutdown now")

    sub = rospy.Subscriber('emergency_shutdown_request', String, shutdown_cmd, None, 2)
    while not rospy.is_shutdown():
        rospy.sleep(0.01)

if __name__ == '__main__':
    main()