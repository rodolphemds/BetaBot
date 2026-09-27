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
import threading

import usb.core
import nxt.locator
from nxt.sensor import PORT_1, PORT_2, PORT_3, PORT_4
from nxt.sensor import Type
from nxt.sensor import Touch, Ultrasonic
from nxt.motor import Motor, PORT_A, PORT_B, PORT_C
from nxt.locator import find_one_brick
import nxt.sensor
import nxt.motor
import nxt.error

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Range
from nxt_msgs.msg import JointCommand, Contact

POWER_TO_NM = 0.01


class NXT1ROS(Node):

    def __init__(self):
        super().__init__('nxt1_ros')
        self.callback_handle_frequency = 10.0
        self.last_callback_handle = self.get_clock().now()
        self.lock = threading.Lock()

        # Connecting to NXT1 brick
        n1_connected = False
        while not n1_connected:
            try:
                self.get_logger().info("Connecting to NXT1...")
                self.b1 = find_one_brick(name="NXT1")
                n1_connected = True
                self.get_logger().info("NXT1 connected")
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError

        # Publishers definition
        self.right_bumper_publisher = self.create_publisher(Contact, "right_bumper", 1)
        self.get_logger().info("Setting up publisher on right_bumper "
                                "[nxt_msgs/Contact] and header frame identity right_bumper_link.")
        self.left_bumper_publisher = self.create_publisher(Contact, "left_bumper", 1)
        self.get_logger().info("Setting up publisher on left_bumper "
                                "[nxt_msgs/Contact] and header frame identity left_bumper_link.")
        self.back_bumper_publisher = self.create_publisher(Contact, "back_bumper", 1)
        self.get_logger().info("Setting up publisher on back_bumper "
                                "[nxt_msgs/Contact] and header frame identity back_bumper_link.")
        self.ultrasonic_sensor_publisher = self.create_publisher(Range, "ultrasonic_sensor", 1)
        self.get_logger().info("Setting up publisher on ultrasonic_sensor "
                                "[sensor_msgs/Range] and header frame identity ultrasonic_sensor_link.")

        self.joint_state_publisher = self.create_publisher(JointState, "joint_state", 10)
        self.joint_command_publisher = self.create_publisher(JointCommand, "joint_command", 1)

        # Declaring sensors and motors
        self.components = []
        self.right_bumper = Touch(self.b1, PORT_1)
        self.get_logger().info("Connecting to right_bumper on NXT1_PORT_1")
        params_right_bumper = {'type': 'touch', 'name': 'right_bumper', 'port': 'PORT_1',
                               'brick': 'NXT1', 'desired_frequency': 1}
        self.components.append(RightTouchSensor(self, params_right_bumper, self.b1))
        self.left_bumper = Touch(self.b1, PORT_2)
        self.get_logger().info("Connecting to left_bumper on NXT1_PORT_2")
        params_left_bumper = {'type': 'touch', 'name': 'left_bumper', 'port': 'PORT_2',
                              'brick': 'NXT1', 'desired_frequency': 1}
        self.components.append(LeftTouchSensor(self, params_left_bumper, self.b1))
        self.back_bumper = Touch(self.b1, PORT_3)
        self.get_logger().info("Connecting to back_bumper on NXT1_PORT_3")
        params_back_bumper = {'type': 'touch', 'name': 'back_bumper', 'port': 'PORT_3',
                              'brick': 'NXT1', 'desired_frequency': 1}
        self.components.append(BackTouchSensor(self, params_back_bumper, self.b1))
        self.ultrasonic_sensor = Ultrasonic(self.b1, PORT_4)
        self.get_logger().info("Connecting to ultrasonic_sensor on NXT1_PORT_4")
        params_ultrasonic_sensor = {'type': 'ultrasonic', 'name': 'ultrasonic_sensor', 'port': 'PORT_4',
                                     'brick': 'NXT1', 'desired_frequency': 1}
        self.components.append(UltrasonicSensor(self, params_ultrasonic_sensor, self.b1))
        self.right_tread = Motor(self.b1, PORT_A)
        self.get_logger().info("Connecting to right_tread on NXT1_PORT_A")
        params_right_tread = {'type': 'motor', 'name': 'right_tread', 'port': 'PORT_A',
                              'brick': 'NXT1', 'desired_frequency': 2}
        self.components.append(RightTreadMotor(self, params_right_tread, self.b1))
        self.torso_joint = Motor(self.b1, PORT_B)
        self.get_logger().info("Connecting to torso_joint on NXT1_PORT_B")
        params_torso_joint = {'type': 'motor', 'name': 'torso_joint', 'port': 'PORT_B',
                              'brick': 'NXT1', 'desired_frequency': 1}
        self.components.append(TorsoJoint(self, params_torso_joint, self.b1))
        self.left_tread = Motor(self.b1, PORT_C)
        self.get_logger().info("Connecting to left_tread on NXT1_PORT_C")
        params_left_tread = {'type': 'motor', 'name': 'left_tread', 'port': 'PORT_C',
                             'brick': 'NXT1', 'desired_frequency': 2}
        self.components.append(LeftTreadMotor(self, params_left_tread, self.b1))

        self.destroy_joint_command_subscription = self.create_subscription(
            JointCommand, "joint_command", self.joint_command_cb, 2)

    def joint_command_cb(self, msg):
        with self.lock:
            for component in self.components:
                if hasattr(component, 'cmd_cb'):
                    component.cmd_cb(msg)

    def spin_once(self):
        with self.lock:
            triggered = False
            for c in self.components:
                if c.needs_trigger() and not triggered:
                    c.do_trigger()
                    triggered = True
        now = self.get_clock().now()
        if (now - self.last_callback_handle).nanoseconds > 1e9 / self.callback_handle_frequency:
            self.last_callback_handle = now
            rclpy.spin_once(self, timeout_sec=0.01)

    def cleanup_node(self):
        self.get_logger().info("Shutting down NXT1 sensors and motors...")
        self.right_tread.run(0, 0)
        self.left_tread.run(0, 0)
        self.torso_joint.run(0, 0)
        self.ultrasonic_sensor.command(nxt.sensor.Ultrasonic.Commands.OFF)


# Base class
class Device:
    def __init__(self, node, params):
        self.node = node
        self.desired_period = 1.0 / params['desired_frequency']
        self.period = self.desired_period
        self.initialized = False
        self.name = params['name']

    def now(self):
        return self.node.get_clock().now()

    def logdebug(self, text):
        self.node.get_logger().debug(text)

    def logwarn(self, text):
        self.node.get_logger().warning(text)

    def needs_trigger(self):
        # initialize
        if not self.initialized:
            self.initialized = True
            self.last_run = self.now()
            self.logdebug('Initializing %s' % self.name)
            return False
        # compute frequency
        now = self.now()
        period = 0.9 * self.period + 0.1 * (now - self.last_run).nanoseconds / 1e9

        # check period
        if period > self.desired_period * 1.2:
            self.logwarn("%s not reaching desired frequency: actual %f, desired %f."
                         % (self.name, 1.0 / period, 1.0 / self.desired_period))
        elif period > self.desired_period * 1.5:
            self.logwarn("%s not reaching desired frequency: actual %f, desired %f."
                         % (self.name, 1.0 / period, 1.0 / self.desired_period))

        return period > self.desired_period

    def do_trigger(self):
        try:
            self.logdebug('Trigger %s with current frequency %f.' % (self.name, 1.0 / self.period))
            now = self.now()
            self.period = 0.9 * self.period + 0.1 * (now - self.last_run).nanoseconds / 1e9
            self.last_run = now
            self.trigger()
            self.logdebug('Trigger %s took %f mili-seconds.'
                          % (self.name, (self.now() - now).nanoseconds / 1e6))
        except nxt.error.DirProtError:
            self.logwarn("caught an exception nxt.error.DirProtError.")
            pass
        except nxt.error.I2CError:
            self.logwarn("caught an exception nxt.error.I2CError.")
            pass


class RightTouchSensor(Device):
    def __init__(self, node, params_right_bumper, comm):
        Device.__init__(self, node, params_right_bumper)

    def trigger(self):
        right_bumper_report = Contact()
        right_bumper_report.header.frame_id = "right_bumper_link"
        right_bumper_report.header.stamp = self.now().to_msg()
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                right_bumper_report.contact = self.node.right_bumper.get_sample()
                USBTransmissionSuccess = True
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
        self.node.right_bumper_publisher.publish(right_bumper_report)


class LeftTouchSensor(Device):
    def __init__(self, node, params_left_bumper, comm):
        Device.__init__(self, node, params_left_bumper)

    def trigger(self):
        left_bumper_report = Contact()
        left_bumper_report.header.frame_id = "left_bumper_link"
        left_bumper_report.header.stamp = self.now().to_msg()
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                left_bumper_report.contact = self.node.left_bumper.get_sample()
                USBTransmissionSuccess = True
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
        self.node.left_bumper_publisher.publish(left_bumper_report)


class BackTouchSensor(Device):
    def __init__(self, node, params_back_bumper, comm):
        Device.__init__(self, node, params_back_bumper)

    def trigger(self):
        back_bumper_report = Contact()
        back_bumper_report.header.frame_id = "back_bumper_link"
        back_bumper_report.header.stamp = self.now().to_msg()
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                back_bumper_report.contact = self.node.back_bumper.get_sample()
                USBTransmissionSuccess = True
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
        self.node.back_bumper_publisher.publish(back_bumper_report)


class UltrasonicSensor(Device):
    def __init__(self, node, params_ultrasonic_sensor, comm):
        Device.__init__(self, node, params_ultrasonic_sensor)

    def trigger(self):
        ultrasonic_sensor_report = Range()
        ultrasonic_sensor_report.header.frame_id = "ultrasonic_sensor_link"
        ultrasonic_sensor_report.header.stamp = self.now().to_msg()
        ultrasonic_sensor_report.radiation_type = 0
        ultrasonic_sensor_report.field_of_view = 0.5235987756  # rad
        ultrasonic_sensor_report.min_range = 0.07
        ultrasonic_sensor_report.max_range = 2.54
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                ultrasonic_sensor_report.range = self.node.ultrasonic_sensor.get_sample() / 100.0
                USBTransmissionSuccess = True
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
        self.node.ultrasonic_sensor_publisher.publish(ultrasonic_sensor_report)


class LeftTreadMotor(Device):
    def __init__(self, node, params_left_tread, comm):
        Device.__init__(self, node, params_left_tread)
        self.name = "left_tread"
        self.power_max = 125
        self.brake = False
        self.cmd = 0  # the commanded power value

        # Use absolute position
        # True  = Return the position as an absolute value between
        #         0 to 2*pi radians (one full rotation).
        # False = The position ranges from -32,768 to +32,767
        #         degrees, relative to the starting position.
        #         After this the 16-bit signed integer will overflow.
        self.use_absolute_position = False
        # Number of degrees per revolution (integer),
        # this can be tuned for better acuracy
        self.counts_per_rev = 358

        if self.use_absolute_position:
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    self.node.left_tread.reset_position(False)
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError

        self.last_js = None
        self.logdebug("Connecting to joint_state motor publisher.")

    def cmd_cb(self, msg):
        if msg.name == self.name:
            # Store the commanded power value,
            # limited to the range +/-125
            cmd = msg.effort / POWER_TO_NM
            if cmd > self.power_max:
                cmd = self.power_max
            elif cmd < -self.power_max:
                cmd = -self.power_max
            self.cmd = cmd
            self.brake = msg.brake

    def trigger(self):
        js = JointState()
        js.header.stamp = self.now().to_msg()
        js.name.append(self.name)

        # Get the rotational position of the motor
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                rotation_count = self.node.left_tread.get_tacho().rotation_count
                USBTransmissionSuccess = True
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
        position_in_radians = rotation_count * math.pi / 180.0

        if self.use_absolute_position:
            # Check if we have gone backwards
            # past the starting position
            if position_in_radians < 0.0:
                position_in_radians = 2.0 * math.pi + position_in_radians

        js.position.append(position_in_radians)
        js.effort.append(self.cmd * POWER_TO_NM)  # this is just the commanded effort
        vel = 0
        if self.last_js:
            dt = (js.header.stamp.sec - self.last_js.header.stamp.sec) + \
                 (js.header.stamp.nanosec - self.last_js.header.stamp.nanosec) / 1e9
            vel = (js.position[0] - self.last_js.position[0]) / dt
            js.velocity.append(vel)
        else:
            vel = 0
            js.velocity.append(vel)
        self.node.joint_state_publisher.publish(js)
        self.last_js = js

        if self.use_absolute_position:
            # If motor has done a full rotation, reset the encoder to zero
            if (rotation_count >= self.counts_per_rev) \
                    or (rotation_count <= -1.0 * self.counts_per_rev):
                USBTransmissionSuccess = False
                while not USBTransmissionSuccess:
                    try:
                        self.node.left_tread.reset_position(False)
                        USBTransmissionSuccess = True
                    except usb.core.USBError as e:
                        if e.errno == 110:
                            continue
                        else:
                            raise usb.core.USBError

        # send command
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                # backward and forward are inversed for the treads motor
                self.node.left_tread.run(-int(self.cmd), 0)
                USBTransmissionSuccess = True
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
        if self.brake:
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    self.node.left_tread.brake()
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError


class RightTreadMotor(Device):
    def __init__(self, node, params_right_tread, comm):
        Device.__init__(self, node, params_right_tread)
        self.name = "right_tread"
        self.power_max = 125
        self.brake = False
        self.cmd = 0  # the commanded power value

        # Use absolute position
        # True  = Return the position as an absolute value between
        #         0 to 2*pi radians (one full rotation).
        # False = The position ranges from -32,768 to +32,767
        #         degrees, relative to the starting position.
        #         After this the 16-bit signed integer will overflow.
        self.use_absolute_position = False
        # Number of degrees per revolution (integer),
        # this can be tuned for better acuracy
        self.counts_per_rev = 358

        if self.use_absolute_position:
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    self.node.right_tread.reset_position(False)
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError

        self.last_js = None
        self.logdebug("Connecting to joint_state motor publisher.")

    def cmd_cb(self, msg):
        if msg.name == self.name:
            # Store the commanded power value,
            # limited to the range +/-125
            cmd = msg.effort / POWER_TO_NM
            if cmd > self.power_max:
                cmd = self.power_max
            elif cmd < -self.power_max:
                cmd = -self.power_max
            self.cmd = cmd
            self.brake = msg.brake

    def trigger(self):
        js = JointState()
        js.header.stamp = self.now().to_msg()
        js.name.append(self.name)

        # Get the rotational position of the motor
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                rotation_count = self.node.right_tread.get_tacho().rotation_count
                USBTransmissionSuccess = True
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
        position_in_radians = rotation_count * math.pi / 180.0

        if self.use_absolute_position:
            # Check if we have gone backwards
            # past the starting position
            if position_in_radians < 0.0:
                position_in_radians = 2.0 * math.pi + position_in_radians

        js.position.append(position_in_radians)
        js.effort.append(self.cmd * POWER_TO_NM)  # this is just the commanded effort
        vel = 0
        if self.last_js:
            dt = (js.header.stamp.sec - self.last_js.header.stamp.sec) + \
                 (js.header.stamp.nanosec - self.last_js.header.stamp.nanosec) / 1e9
            vel = (js.position[0] - self.last_js.position[0]) / dt
            js.velocity.append(vel)
        else:
            vel = 0
            js.velocity.append(vel)
        self.node.joint_state_publisher.publish(js)
        self.last_js = js

        if self.use_absolute_position:
            # If motor has done a full rotation, reset the encoder to zero
            if (rotation_count >= self.counts_per_rev) \
                    or (rotation_count <= -1.0 * self.counts_per_rev):
                USBTransmissionSuccess = False
                while not USBTransmissionSuccess:
                    try:
                        self.node.right_tread.reset_position(False)
                        USBTransmissionSuccess = True
                    except usb.core.USBError as e:
                        if e.errno == 110:
                            continue
                        else:
                            raise usb.core.USBError

        # send command
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                # backward and forward are inversed for the treads motor
                self.node.right_tread.run(-int(self.cmd), 0)
                USBTransmissionSuccess = True
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
        if self.brake:
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    self.node.right_tread.brake()
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError


class TorsoJoint(Device):
    def __init__(self, node, params_torso_joint, comm):
        Device.__init__(self, node, params_torso_joint)
        self.name = "torso_joint"
        self.power_max = 100  # power is a value between -127 and 128
        self.brake = False  # brake after rotations
        self.power = 0  # the commanded power value
        self.rotations = 0  # the commanded rotations to execute
        self.jnt_cmd = JointCommand()

        # Use absolute position
        # True  = Return the position as an absolute value between
        #         0 to 2*pi radians (one full rotation).
        # False = The position ranges from -32,768 to +32,767
        #         degrees, relative to the starting position.
        #         After this the 16-bit signed integer will overflow.
        self.use_absolute_position = False
        # Number of degrees per revolution (integer),
        # this can be tuned for better acuracy
        self.counts_per_rev = 360

        if self.use_absolute_position:
            # Upon initialization, reset the motor encoder to zero.
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    self.node.torso_joint.reset_position(False)
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError

        self.last_js = None

    def cmd_cb(self, msg):
        if msg.name == self.name:
            # Store the commanded power and rotations value,
            # limiting power to the range +/- power_max
            power = msg.effort / POWER_TO_NM
            if power > self.power_max:
                power = self.power_max
            elif power < -self.power_max:
                power = -self.power_max
            self.power = int(power)
            self.rotations = int(msg.rotations) * 180 / math.pi
            self.brake = msg.brake

    def trigger(self):
        js = JointState()
        js.header.stamp = self.now().to_msg()
        js.name.append(self.name)

        # Get the rotational position of the motor
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                rotation_count = self.node.torso_joint.get_tacho().rotation_count
                USBTransmissionSuccess = True
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
        position_in_radians = rotation_count * math.pi / 180.0

        if self.use_absolute_position:
            # Check if we have gone backwards
            # past the starting position
            if position_in_radians < 0.0:
                position_in_radians = 2.0 * math.pi + position_in_radians

        js.position.append(position_in_radians)
        js.effort.append(self.power * POWER_TO_NM)  # this is just the commanded effort
        vel = 0
        if self.last_js:
            dt = (js.header.stamp.sec - self.last_js.header.stamp.sec) + \
                 (js.header.stamp.nanosec - self.last_js.header.stamp.nanosec) / 1e9
            vel = (js.position[0] - self.last_js.position[0]) / dt
            js.velocity.append(vel)
        else:
            vel = 0
            js.velocity.append(vel)
        self.node.joint_state_publisher.publish(js)
        self.last_js = js

        if self.use_absolute_position:
            # If motor has done a full rotation, reset the encoder to zero
            if (rotation_count >= self.counts_per_rev) \
                    or (rotation_count <= -1.0 * self.counts_per_rev):
                USBTransmissionSuccess = False
                while not USBTransmissionSuccess:
                    try:
                        self.node.torso_joint.reset_position(False)
                        USBTransmissionSuccess = True
                    except usb.core.USBError as e:
                        if e.errno == 110:
                            continue
                        else:
                            raise usb.core.USBError

        # send command
        if self.power != 0:
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    try:
                        self.node.torso_joint.turn(power=self.power, tacho_units=self.rotations,
                                                   brake=self.brake)
                    except nxt.motor.BlockedException:
                        self.logwarn("Torso joint is blocked.")
                        pass
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError
            self.jnt_cmd.header.frame_id = self.name + "_command"
            self.jnt_cmd.header.stamp = self.now().to_msg()
            self.jnt_cmd.name = self.name
            self.jnt_cmd.brake = self.brake
            self.jnt_cmd.rotations = 0.0
            self.jnt_cmd.effort = 0.0
            self.node.joint_command_publisher.publish(self.jnt_cmd)


def main():
    rclpy.init()
    node = NXT1ROS()
    try:
        while rclpy.ok():
            node.spin_once()
    finally:
        node.cleanup_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
