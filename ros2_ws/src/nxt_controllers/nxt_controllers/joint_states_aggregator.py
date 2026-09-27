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

from sensor_msgs.msg import JointState


class JS:
    def __init__(self, name, header, position, velocity, effort):
        self.name = name
        self.header = header
        self.position = position
        self.velocity = velocity
        self.effort = effort


class JSAggregator(Node):

    def __init__(self):
        super().__init__("joint_states_aggregator")
        # create motor
        self.create_subscription(JointState, 'joint_state', self.callback, 10)
        # create publisher
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.observed_states = {}
        self.updates_since_publish = 0

    def callback(self, data):
        num_joints = len(data.name)
        if len(data.position) < num_joints:
            self.get_logger().error("Position array shorter than names %s < %d"
                                    % (len(data.position), num_joints))
            return
        elif len(data.velocity) < num_joints:
            self.get_logger().error("Velocity array shorter than names %s < %d"
                                    % (len(data.velocity), num_joints))
            return
        elif len(data.effort) < num_joints:
            self.get_logger().error("Effort array shorter than names %s < %d"
                                    % (len(data.effort), num_joints))
            return

        for i in range(0, num_joints):
            self.observed_states[data.name[i]] = JS(data.name[i],
                                                    data.header,
                                                    data.position[i],
                                                    data.velocity[i],
                                                    data.effort[i])

        stamp_nanoseconds = self.get_clock().now().nanoseconds
        todelete = [k for k, v in self.observed_states.items()
                    if stamp_nanoseconds - self._stamp_ns(v.header) > 10e9]
        for td in todelete:
            del self.observed_states[td]

        # Only publish if there has been as many updates as there are joints,
        # otherwise odom gets zero deltas and the robot jerks around.
        if self.updates_since_publish < len(self.observed_states.keys()):
            self.updates_since_publish += 1
            return

        self.updates_since_publish = 0
        msg_out = JointState()
        msg_out.header = data.header
        for k, v in self.observed_states.items():
            msg_out.name.append(v.name)
            msg_out.position.append(v.position)
            msg_out.velocity.append(v.velocity)
            msg_out.effort.append(v.effort)
        self.pub.publish(msg_out)

    @staticmethod
    def _stamp_ns(header):
        return header.stamp.sec * 1e9 + header.stamp.nanosec


def main():
    rclpy.init()
    agg = JSAggregator()
    rclpy.spin(agg)
    agg.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
