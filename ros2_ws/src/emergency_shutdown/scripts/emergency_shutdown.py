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

import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from time import sleep


class EmergencyShutdown(Node):

    def shutdown_cmd(self, msg):
        if msg.data == "LOW_BATTERY":
            self.get_logger().warn("An emergency shutdown is requested to protect battery life.")
            self.get_logger().warn("Please find an other power source within 10 sec to abort robot shutdown.")
            sleep(10)
            if msg.data == "LOW_BATTERY":
                self.get_logger().warn("Do not forget to manually kill the power switch on the robot base.")
                os.system("shutdown now")
        if msg.data == "ROOT_ORDER":
            self.get_logger().warn("An emergency shutdown is commanded by root order.")
            self.get_logger().warn('You have 5 sec to stop publishing "ROOT_ORDER" on the '
                                    'emergency_shutdown_request publisher if you want to abort.')
            sleep(5)
            if msg.data == "ROOT_ORDER":
                os.system("shutdown now")


def main():
    rclpy.init()
    node = EmergencyShutdown()
    node.get_logger().info("Initializing emergency_shutdown node...")
    node.create_subscription(String, 'emergency_shutdown_request', node.shutdown_cmd, 2)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
