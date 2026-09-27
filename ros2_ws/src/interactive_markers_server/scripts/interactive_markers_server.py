#!/usr/bin/env python
################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################
# Authors: Gilbert #
# Authors: Rodolphe Matias de Sousa #

import copy
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl


class InteractiveMarkersServerNode(Node):

    def __init__(self):
        super().__init__("interactive_markers_server")
        self.server = InteractiveMarkerServer(self, "interactive_markers_server")
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 5)

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = "robot_marker"

        control = InteractiveMarkerControl()
        control.orientation_mode = InteractiveMarkerControl.FIXED
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.always_visible = True
        int_marker.controls.append(copy.deepcopy(control))

        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
        int_marker.controls.append(copy.deepcopy(control))

        self.server.insert(int_marker, self.process_feedback)
        self.server.applyChanges()

    def process_feedback(self, feedback):
        yaw = self.euler_from_quaternion((feedback.pose.orientation.x,
                                           feedback.pose.orientation.y,
                                           feedback.pose.orientation.z,
                                           feedback.pose.orientation.w))[2]
        twist = Twist()
        twist.angular.z = 2.2 * yaw
        twist.linear.x = 1.0 * feedback.pose.position.x
        self.vel_pub.publish(twist)
        self.server.setPose("robot_marker", Pose())
        self.server.applyChanges()

    @staticmethod
    def euler_from_quaternion(quaternion):
        x, y, z, w = quaternion
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return (0.0, 0.0, yaw)


def main():
    rclpy.init()
    node = InteractiveMarkersServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
