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
from nxt.motor import Motor, PORT_A, PORT_B, PORT_C
from nxt.locator import find_one_brick
import nxt.brick
import nxt.error
import nxt.sensor
import nxt.motor

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
from nxt_msgs.msg import JointCommand, Contact, Color, Light, RFID, Sound

POWER_TO_NM = 0.01

ALLOWED_COLOR_COMMANDS = ("RED", "GREEN", "BLUE", "NONE", "FULL")


class NXT2ROS(Node):

    def __init__(self):
        super().__init__('nxt2_ros')
        self.callback_handle_frequency = 10.0
        self.last_callback_handle = self.get_clock().now()
        self.lock = threading.Lock()

        # Connecting to NXT2 brick
        n2_connected = False
        while not n2_connected:
            try:
                self.get_logger().info("Connecting to NXT2...")
                self.b2 = find_one_brick(name="NXT2")
                n2_connected = True
                self.get_logger().info("NXT2 connected")
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError

        # Publishers definition
        self.color_sensor_publisher = self.create_publisher(Color, "color_sensor", 1)
        self.get_logger().info("Setting up publisher on color_sensor "
                                "[nxt_msgs/Color] and header frame identity color_sensor_link.")
        self.line_following_sensor_publisher = self.create_publisher(Light, "line_following_sensor", 1)
        self.get_logger().info("Setting up publisher on line_following_sensor "
                                "[nxt_msgs/Light] and header frame identity line_following_sensor_link.")
        self.rfid_sensor_publisher = self.create_publisher(RFID, "rfid_sensor", 1)
        self.get_logger().info("Setting up publisher on rfid_sensor "
                                "[nxt_msgs/RFID] and header frame identity rfid_sensor_link.")
        self.sound_sensor_publisher = self.create_publisher(Sound, "sound_sensor", 1)
        self.get_logger().info("Setting up publisher on sound_sensor "
                                "[nxt_msgs/Sound] and header frame identity sound_sensor_link.")

        self.joint_state_publisher = self.create_publisher(JointState, "joint_state", 10)
        self.joint_command_publisher = self.create_publisher(JointCommand, "joint_command", 1)

        # Declaring sensors and motors
        self.components = []
        self.sound_sensor = nxt.sensor.Sound(self.b2, PORT_1)
        self.get_logger().info("Connecting to sound_sensor on NXT2_PORT_1")
        params_sound_sensor = {'type': 'sound', 'name': 'sound_sensor', 'port': 'PORT_1',
                               'brick': 'NXT2', 'desired_frequency': 1}
        self.components.append(SoundSensor(self, params_sound_sensor, self.b2))
        self.color_sensor = nxt.sensor.Color20(self.b2, PORT_2)
        self.color_sensor.set_light_color(Type.COLORNONE)
        self.get_logger().info("Connecting to color_sensor on NXT2_PORT_2")
        self.get_logger().info("Subscribing to color_sensor_command publisher "
                                "to turn on a specified color sensor light.")
        params_color_sensor = {'type': 'color', 'name': 'color_sensor', 'port': 'PORT_2',
                               'brick': 'NXT2', 'desired_frequency': 1}
        self.components.append(ColorSensor(self, params_color_sensor, self.b2))
        self.line_following_sensor = nxt.sensor.Light(self.b2, PORT_3)
        self.line_following_sensor.set_illuminated(active=True)
        self.get_logger().info("Connecting to line_following_sensor on NXT2_PORT_3")
        self.get_logger().info("Subscribing to line_following_sensor_set_illuminated publisher "
                                "to control sensor light emission.")
        params_line_following_sensor = {'type': 'light', 'name': 'line_following_sensor', 'port': 'PORT_3',
                                         'brick': 'NXT2', 'desired_frequency': 2}
        self.components.append(LineFollowingSensor(self, params_line_following_sensor, self.b2))
        self.get_logger().info("Connecting to RFID_sensor on NXT2_PORT_4")
        params_rfid_sensor = {'type': 'RFID', 'name': 'rfid_sensor', 'port': 'PORT_4',
                               'brick': 'NXT2', 'desired_frequency': 1}
        self.components.append(RFIDSensor(self, params_rfid_sensor, self.b2))
        self.head_joint = Motor(self.b2, PORT_A)
        self.get_logger().info("Connecting to head_joint on NXT2_PORT_A")
        params_head_joint = {'type': 'motor', 'name': 'head_joint', 'port': 'PORT_A',
                             'brick': 'NXT2', 'desired_frequency': 1}
        self.components.append(HeadJoint(self, params_head_joint, self.b2))
        self.laser_joint = Motor(self.b2, PORT_B)
        self.get_logger().info("Connecting to laser_joint on NXT2_PORT_B")
        params_laser_joint = {'type': 'motor', 'name': 'laser_joint', 'port': 'PORT_B',
                               'brick': 'NXT2', 'desired_frequency': 1}
        self.components.append(LaserJoint(self, params_laser_joint, self.b2))
        self.arms_joint = Motor(self.b2, PORT_C)
        self.get_logger().info("Connecting to arms_joint on NXT2_PORT_C")
        params_arms_joint = {'type': 'motor', 'name': 'arms_joint', 'port': 'PORT_C',
                              'brick': 'NXT2', 'desired_frequency': 1}
        self.components.append(ArmsJoint(self, params_arms_joint, self.b2))

        self.create_subscription(String, "color_sensor_command", self.color_sensor_command_cb, 2)
        self.create_subscription(Bool, "line_following_sensor_set_illuminated",
                                 self.line_following_sensor_command_cb, 2)
        self.create_subscription(JointCommand, "joint_command", self.joint_command_cb, 2)

    def color_sensor_command_cb(self, msg):
        for component in self.components:
            if isinstance(component, ColorSensor):
                component.color_sensor_command_callback(msg)

    def line_following_sensor_command_cb(self, msg):
        for component in self.components:
            if isinstance(component, LineFollowingSensor):
                component.line_following_sensor_command_callback(msg)

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
        self.get_logger().info("Shutting down NXT2 sensors and motors...")
        self.head_joint.run(0, 0)
        self.arms_joint.run(0, 0)
        self.laser_joint.run(0, 0)
        self.line_following_sensor.set_illuminated(active=False)
        self.color_sensor.set_light_color(Type.COLORNONE)
        self.b2.stop_program()


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


class ColorSensor(Device):
    """
        This uses the NXT 2.0 RGB color sensor
        to detect six preset color values,
        and measure the intensity of reflected light
        with the option to enable the red, green,
        or blue LEDs individually.
    """

    def __init__(self, node, params_color_sensor, comm):
        Device.__init__(self, node, params_color_sensor)
        self.color_sensor_command_color = "FULL"
        self.color_code = 0
        self.color_type = Type.COLORFULL

    def color_sensor_command_callback(self, color_sensor_command):
        if color_sensor_command.data in ALLOWED_COLOR_COMMANDS:
            self.color_sensor_command_color = color_sensor_command.data
        else:
            self.color_sensor_command_color = "FULL"

    def trigger(self):
        color_sensor_report = Color()
        color_sensor_report.header.frame_id = "color_sensor_link"
        color_sensor_report.header.stamp = self.now().to_msg()
        if self.color_sensor_command_color == "RED":
            self.color_type = Type.COLORRED
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    self.node.color_sensor.set_light_color(self.color_type)
                    color_sensor_report.reflected_light_intensity = \
                        self.node.color_sensor.get_reflected_light(self.color_type)
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError
        elif self.color_sensor_command_color == "GREEN":
            self.color_type = Type.COLORGREEN
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    self.node.color_sensor.set_light_color(self.color_type)
                    color_sensor_report.reflected_light_intensity = \
                        self.node.color_sensor.get_reflected_light(self.color_type)
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError
        elif self.color_sensor_command_color == "BLUE":
            self.color_type = Type.COLORBLUE
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    self.node.color_sensor.set_light_color(self.color_type)
                    color_sensor_report.reflected_light_intensity = \
                        self.node.color_sensor.get_reflected_light(self.color_type)
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError
        elif self.color_sensor_command_color == "NONE":
            self.color_type = Type.COLORNONE
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    self.node.color_sensor.set_light_color(self.color_type)
                    color_sensor_report.reflected_light_intensity = \
                        self.node.color_sensor.get_reflected_light(self.color_type)
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError
        elif self.color_sensor_command_color == "FULL":
            self.color_type = Type.COLORFULL
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    self.node.color_sensor.set_light_color(self.color_type)
                    color_sensor_report.reflected_light_intensity = \
                        self.node.color_sensor.get_reflected_light(self.color_type)
                    # Color detection can only be performed when led is set to white (Type.COLORFULL).
                    self.color_code = self.node.color_sensor.get_color()
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError
        if self.color_code == 1:  # black
            color_sensor_report.r = 0.0
            color_sensor_report.g = 0.0
            color_sensor_report.b = 0.0
            color_sensor_report.color = "black"
        elif self.color_code == 2:  # blue
            color_sensor_report.r = 0.0
            color_sensor_report.g = 0.0
            color_sensor_report.b = 1.0
            color_sensor_report.color = "blue"
        elif self.color_code == 3:  # green
            color_sensor_report.r = 0.0
            color_sensor_report.g = 1.0
            color_sensor_report.b = 0.0
            color_sensor_report.color = "green"
        elif self.color_code == 4:  # yellow
            color_sensor_report.r = 1.0
            color_sensor_report.g = 1.0
            color_sensor_report.b = 0.0
            color_sensor_report.color = "yellow"
        elif self.color_code == 5:  # red
            color_sensor_report.r = 1.0
            color_sensor_report.g = 0.0
            color_sensor_report.b = 1.0
            color_sensor_report.color = "red"
        elif self.color_code == 6:  # white
            color_sensor_report.r = 1.0
            color_sensor_report.g = 1.0
            color_sensor_report.b = 1.0
            color_sensor_report.color = "white"
        else:
            color_sensor_report.r = 9
            color_sensor_report.g = 9
            color_sensor_report.b = 9
            color_sensor_report.color = "undefined"

        self.node.color_sensor_publisher.publish(color_sensor_report)


class LineFollowingSensor(Device):
    """
        This uses the NXT1 light sensor to
        measure the intensity of ambient light,
        with the option to enable the LED to
        measure reflected light.
    """
    def __init__(self, node, params_line_following_sensor, comm):
        Device.__init__(self, node, params_line_following_sensor)
        self.line_following_sensor_illuminated = True

    def line_following_sensor_command_callback(self, line_following_sensor_set_illuminated):
        self.line_following_sensor_illuminated = line_following_sensor_set_illuminated.data

    def trigger(self):
        if self.line_following_sensor_illuminated:
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    self.node.line_following_sensor.set_illuminated(active=True)
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError
        elif self.line_following_sensor_illuminated is False:
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    self.node.line_following_sensor.set_illuminated(active=False)
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError
        line_following_sensor_report = Light()
        line_following_sensor_report.header.frame_id = "line_following_sensor_link"
        line_following_sensor_report.header.stamp = self.now().to_msg()
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                line_following_sensor_report.intensity = \
                    self.node.line_following_sensor.get_lightness()
                USBTransmissionSuccess = True
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
        self.node.line_following_sensor_publisher.publish(line_following_sensor_report)


class SoundSensor(Device):
    def __init__(self, node, params_sound_sensor, comm):
        Device.__init__(self, node, params_sound_sensor)

    def trigger(self):
        sound_sensor_report = Sound()
        sound_sensor_report.header.frame_id = "sound_sensor_link"
        sound_sensor_report.header.stamp = self.now().to_msg()
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                self.node.sound_sensor.set_adjusted(active=False)
                sound_sensor_report.data_db = self.node.sound_sensor.get_loudness()
                self.node.sound_sensor.set_adjusted(active=True)
                sound_sensor_report.data_dba = self.node.sound_sensor.get_loudness()
                USBTransmissionSuccess = True
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
        self.node.sound_sensor_publisher.publish(sound_sensor_report)


class RFIDSensor(Device):
    def __init__(self, node, params_rfid_sensor, comm):
        Device.__init__(self, node, params_rfid_sensor)
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                node.b2.start_program('NXT2_rfid_read.rxe')
                USBTransmissionSuccess = True
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError

    def trigger(self):
        rfid_sensor_report = RFID()
        rfid_sensor_report.header.frame_id = "rfid_sensor_link"
        rfid_sensor_report.header.stamp = self.now().to_msg()
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                FileReadSuccess = False
                while not FileReadSuccess:
                    try:
                        read_data = nxt.brick.FileReader(self.node.b2, "rfid_read.txt")
                        FileReadSuccess = True
                    except AttributeError:
                        continue
                    except nxt.error.FileNotFound:
                        continue
                USBTransmissionSuccess = True
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
        data = read_data.read(bytes=None)
        rfid_read = data[0:12]
        rfid_sensor_report.data = rfid_read
        rfid_sensor_report.sensor_type = "RFID"
        rfid_sensor_report.sensor_manufacturer = "Codatex"
        rfid_sensor_report.firmware_version = "V1.0"
        rfid_sensor_report.sensor_serial = "08010204"
        self.node.rfid_sensor_publisher.publish(rfid_sensor_report)


class NXT2Joint(Device):
    def __init__(self, node, params, joint_name, power_max, brake):
        Device.__init__(self, node, params)
        self.name = joint_name
        self.power_max = power_max  # power is a value between -127 and 128
        self.brake = brake  # brake after rotations
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
                    self.motor.reset_position(False)
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError

        self.last_js = None

    @property
    def motor(self):
        return getattr(self.node, self.name)

    @property
    def blocked_message(self):
        return "%s joint is blocked." % self.name.capitalize()

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
                rotation_count = self.motor.get_tacho().rotation_count
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
                        self.motor.reset_position(False)
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
                        self.motor.turn(power=self.power, tacho_units=self.rotations, brake=self.brake)
                    except nxt.motor.BlockedException:
                        self.logwarn(self.blocked_message)
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


class HeadJoint(NXT2Joint):
    def __init__(self, node, params_head_joint, comm):
        NXT2Joint.__init__(self, node, params_head_joint, "head_joint", 75, True)


class LaserJoint(NXT2Joint):
    def __init__(self, node, params_laser_joint, comm):
        NXT2Joint.__init__(self, node, params_laser_joint, "laser_joint", 50, True)


class ArmsJoint(NXT2Joint):
    def __init__(self, node, params_arms_joint, comm):
        NXT2Joint.__init__(self, node, params_arms_joint, "arms_joint", 50, True)


def main():
    rclpy.init()
    node = NXT2ROS()
    try:
        while rclpy.ok():
            node.spin_once()
    finally:
        node.cleanup_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
