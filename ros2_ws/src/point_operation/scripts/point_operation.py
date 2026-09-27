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
# Authors: Gilbert, modified by Rodolphe Matias de Sousa #

import math
import time
from math import copysign, sqrt, pow, pi, atan2

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point, Quaternion

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

msg = """
control your robot
-----------------------
Insert xyz - coordinate.
x : position x (m)
y : position y (m)
z : orientation z (degree: -180 ~ 180)

If you want to close, insert 's'
-----------------------
"""


class GotoPoint(Node):

    def __init__(self):
        super().__init__('point_operation', automatically_declare_parameters_from_overrides=False)
        self.cmd_vel = self.create_publisher(Twist, 'cmd_vel', 5)
        position = Point()
        move_cmd = Twist()
        r = self.create_rate(10)
        self.tf_listener = tf2_ros.Buffer()
        self.tf = tf2_ros.TransformListener(self.tf_listener, self)
        self.odom_frame = 'odom'

        self.base_frame = 'base_footprint'
        try:
            now = rclpy.time.Time()
            self.tf_listener.can_transform(self.odom_frame, 'base_footprint', now)
        except Exception:
            try:
                self.tf_listener.can_transform(self.odom_frame, 'base_link', now)
                self.base_frame = 'base_link'
            except Exception:
                self.get_logger().info("Cannot find transform between odom and base_link or base_footprint")
                rclpy.shutdown()

        (position, rotation) = self.get_odom()
        last_rotation = 0
        linear_speed = 1
        angular_speed = 1
        (goal_x, goal_y, goal_z) = self.getkey()
        if goal_z > 180 or goal_z < -180:
            print("you input wrong z range.")
            self.shutdown()
        goal_z = np.deg2rad(goal_z)
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        distance = goal_distance
        while distance > 0.05:
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y
            path_angle = atan2(goal_y - y_start, goal_x - x_start)
            if path_angle < -pi / 4 or path_angle > pi / 4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2 * pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2 * pi + path_angle
            if last_rotation > pi - 0.1 and rotation <= 0:
                rotation = 2 * pi + rotation
            elif last_rotation < -pi + 0.1 and rotation > 0:
                rotation = -2 * pi + rotation
            move_cmd.angular.z = angular_speed * path_angle - rotation
            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
            move_cmd.linear.x = min(linear_speed * distance, 0.1)
            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)
            last_rotation = rotation
            self.cmd_vel.publish(move_cmd)
            r.sleep()
        (position, rotation) = self.get_odom()
        while abs(rotation - goal_z) > 0.05:
            (position, rotation) = self.get_odom()
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
            self.cmd_vel.publish(move_cmd)
            r.sleep()
        self.get_logger().info("Stopping the robot...")
        self.cmd_vel.publish(Twist())

    def getkey(self):
        x, y, z = input("| x | y | z |\n").split()
        if x == 's':
            self.shutdown()
        x, y, z = [float(x), float(y), float(z)]
        return x, y, z

    def get_odom(self):
        try:
            trans = self.tf_listener.lookup_transform(self.odom_frame, self.base_frame,
                                                      rclpy.time.Time()).transform
            position = Point()
            position.x = trans.translation.x
            position.y = trans.translation.y
            position.z = trans.translation.z
            q = [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w]
            rotation = self.euler_from_quaternion(q)
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info("TF Exception")
            return (Point(), 0.0)
        return (position, rotation[2])

    @staticmethod
    def euler_from_quaternion(quaternion):
        x, y, z, w = quaternion
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = copysign(pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return [roll, pitch, yaw]

    def shutdown(self):
        self.cmd_vel.publish(Twist())
        time.sleep(1)
        raise SystemExit


def main():
    rclpy.init()
    while rclpy.ok():
        print(msg)
        try:
            GotoPoint()
        except SystemExit:
            break


if __name__ == '__main__':
    try:
        main()
    except Exception:
        print("shutdown point_operation.")
