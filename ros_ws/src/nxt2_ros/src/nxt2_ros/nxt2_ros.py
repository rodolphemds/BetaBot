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

import roslib; roslib.load_manifest('nxt2_ros')
import usb.core
import nxt.locator
import rospy
import math
from nxt.sensor import PORT_1, PORT_2, PORT_3, PORT_4
from nxt.sensor import Type
from nxt.sensor import *
from nxt.motor import Motor, SynchronizedMotors, PORT_A, PORT_B, PORT_C
from nxt.brick import Brick
from nxt.locator import find_one_brick
import nxt.sensor
import nxt.motor
import thread
import subprocess
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Bool, String
from nxt_msgs.msg import JointCommand, Contact, Color, Light, RFID, Sound
from PyKDL import Rotation
from time import sleep

line_following_sensor_illuminated = False
color_sensor_command_color = "NONE"

global my_lock
my_lock = thread.allocate_lock()

global power_to_nm
power_to_nm = 0.01

# Publishers definition
color_sensor_publisher = rospy.Publisher("color_sensor", Color, queue_size=1)
rospy.loginfo("Setting up publisher on color_sensor [nxt_msgs/Color] and header frame identity color_sensor_link.")
line_following_sensor_publisher = rospy.Publisher("line_following_sensor", Light, queue_size=1)
rospy.loginfo("Setting up publisher on line_following_sensor [nxt_msgs/Light] and header frame identity line_following_sensor_link.")
rfid_sensor_publisher = rospy.Publisher("rfid_sensor", RFID, queue_size=1)
rospy.loginfo("Setting up publisher on rfid_sensor [nxt_msgs/RFID] and header frame identity rfid_sensor_link.")
sound_sensor_publisher = rospy.Publisher("sound_sensor", Sound, queue_size=1)
rospy.loginfo("Setting up publisher on sound_sensor [nxt_msgs/Sound] and header frame identity sound_sensor_link.")


# Base class
class Device:
    def __init__(self, params):
        self.desired_period = 1.0 / params['desired_frequency']
        self.period = self.desired_period
        self.initialized = False
        self.name = params['name']

    def needs_trigger(self):
        # initialize
        if not self.initialized:
            self.initialized = True
            self.last_run = rospy.Time.now()
            rospy.logdebug('Initializing %s'%self.name)
            return False
        # compute frequency
        now = rospy.Time.now()
        period = 0.9 * self.period + 0.1 * (now - self.last_run).to_sec()

        # check period
        if period > self.desired_period * 1.2:
            rospy.logwarn("%s not reaching desired frequency: actual %f, desired %f."%(self.name, 1.0/period, 1.0/self.desired_period))
        elif period > self.desired_period * 1.5:
            rospy.logerr("%s not reaching desired frequency: actual %f, desired %f."%(self.name, 1.0/period, 1.0/self.desired_period))

        return period > self.desired_period


    def do_trigger(self):
        try:
          rospy.logdebug('Trigger %s with current frequency %f.'%(self.name, 1.0/self.period))
          now = rospy.Time.now()
          self.period = 0.9 * self.period + 0.1 * (now - self.last_run).to_sec()
          self.last_run = now
          self.trigger()
          rospy.logdebug('Trigger %s took %f mili-seconds.'%(self.name, (rospy.Time.now() - now).to_sec()*1000))
        except nxt.error.DirProtError:
          rospy.logwarn("caught an exception nxt.error.DirProtError.")
          pass
        except nxt.error.I2CError:
          rospy.logwarn("caught an exception nxt.error.I2CError.")
          pass


class ColorSensor(Device):
    """
        This uses the NXT 2.0 RGB color sensor
        to detect six preset color values,
        and measure the intensity of reflected light
        with the option to enable the red, green,
        or blue LEDs individually.
    """

    def __init__(self, params_color_sensor, comm):
        Device.__init__(self, params_color_sensor)
        self.color_sensor_command_color = "FULL"
        self.color_code = 0
        self.color_type = Type.COLORFULL
    def color_sensor_command_callback(self, color_sensor_command):
        if self.color_sensor_command_color == "RED" or self.color_sensor_command_color == "GREEN" or self.color_sensor_command_color == "BLUE" or self.color_sensor_command_color == "NONE" or self.color_sensor_command_color == "FULL":
            self.color_sensor_command_color = color_sensor_command.data
        else :
            self.color_sensor_command_color = "FULL"
    def trigger(self):
        color_sensor_command_subscriber = rospy.Subscriber("color_sensor_command", String,
                                                           self.color_sensor_command_callback)
        color_sensor_report = Color()
        color_sensor_report.header.frame_id = "color_sensor_link"
        color_sensor_report.header.stamp = rospy.Time.now()
        if self.color_sensor_command_color == "RED":
            self.color_type = Type.COLORRED
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    color_sensor.set_light_color(self.color_type)
                    color_sensor_report.reflected_light_intensity = color_sensor.get_reflected_light(self.color_type)
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError
        elif self.color_sensor_command_color == "GREEN":
            self.color_type = Type.COLORGREEN
            rospy.loginfo(self.color_type)
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    color_sensor.set_light_color(self.color_type)
                    color_sensor_report.reflected_light_intensity = color_sensor.get_reflected_light(self.color_type)
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
                    color_sensor.set_light_color(self.color_type)
                    color_sensor_report.reflected_light_intensity = color_sensor.get_reflected_light(self.color_type)
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError
        elif self.color_sensor_command_color == "NONE" :
            self.color_type = Type.COLORNONE
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    color_sensor.set_light_color(self.color_type)
                    color_sensor_report.reflected_light_intensity = color_sensor.get_reflected_light(self.color_type)
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
                    color_sensor.set_light_color(self.color_type)
                    color_sensor_report.reflected_light_intensity = color_sensor.get_reflected_light(self.color_type)
                    self.color_code = color_sensor.get_color() # Color detection can only be performed when led is set to white (Type.COLORFULL).
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

        color_sensor_publisher.publish(color_sensor_report)


class LineFollowingSensor(Device):
    """
        This uses the NXT1 light sensor to
        measure the intensity of ambient light,
        with the option to enable the LED to
        measure reflected light.
    """
    def __init__(self, params_line_following_sensor, comm):
        Device.__init__(self, params_line_following_sensor)
        self.line_following_sensor_illuminated = True
    def line_following_sensor_command_callback(self, line_following_sensor_set_illuminated):
        self.line_following_sensor_illuminated = line_following_sensor_set_illuminated.data
    def trigger(self):
        line_following_sensor_command_subscriber = rospy.Subscriber("line_following_sensor_set_illuminated", Bool,
                                                                    self.line_following_sensor_command_callback)
        if self.line_following_sensor_illuminated:
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    line_following_sensor.set_illuminated(active=True)
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError
        elif self.line_following_sensor_illuminated == False :
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    line_following_sensor.set_illuminated(active=False)
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError
        line_following_sensor_report = Light()
        line_following_sensor_report.header.frame_id = "line_following_sensor_link"
        line_following_sensor_report.header.stamp = rospy.Time.now()
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                line_following_sensor_report.intensity = line_following_sensor.get_lightness()
                USBTransmissionSuccess = True
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
        line_following_sensor_publisher.publish(line_following_sensor_report)


class SoundSensor(Device):
    def __init__(self, params_sound_sensor, comm):
        Device.__init__(self, params_sound_sensor)
    def trigger(self):
        sound_sensor_report = Sound()
        sound_sensor_report.header.frame_id="sound_sensor_link"
        sound_sensor_report.header.stamp = rospy.Time.now()
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                sound_sensor.set_adjusted(active=False)
                sound_sensor_report.data_db = sound_sensor.get_loudness()
                sound_sensor.set_adjusted(active=True)
                sound_sensor_report.data_dbA = sound_sensor.get_loudness()
                USBTransmissionSuccess = True
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
        sound_sensor_publisher.publish(sound_sensor_report)


class RFIDSensor(Device):
    def __init__(self, params_rfid_sensor, comm):
        Device.__init__(self, params_rfid_sensor)
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                b2.start_program('NXT2_rfid_read.rxe')
                USBTransmissionSuccess = True
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
    def trigger(self):
        rfid_sensor_report = RFID()
        rfid_sensor_report.header.frame_id="rfid_sensor_link"
        rfid_sensor_report.header.stamp = rospy.Time.now()
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                FileReadSuccess = False
                while not FileReadSuccess:
                    try :
                        read_data=nxt.brick.FileReader(b2, "rfid_read.txt")
                        FileReadSuccess = True
                    except AttributeError as e:
                        continue
                    except nxt.error.FileNotFound as e:
                        continue
                USBTransmissionSuccess = True
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
        data=read_data.read(bytes=None)
        rfid_read=data[0:12]
        rfid_sensor_report.data=rfid_read
        rfid_sensor_report.sensor_type = "RFID"
        rfid_sensor_report.sensor_manufacturer ="Codatex"
        rfid_sensor_report.firmware_version = "V1.0"
        rfid_sensor_report.sensor_serial = "08010204"
        rfid_sensor_publisher.publish(rfid_sensor_report)


class HeadJoint(Device):
    def __init__(self, params_head_joint, comm):
        Device.__init__(self, params_head_joint)
        self.name = "head_joint"
        self.power_max = 75 # power is a value between -127 and 128 (an absolute value greater than 64 is recommended)
        self.brake = True # brake after rotations
        self.power = 0  # the commanded power value
        self.rotations = 0 # the commanded rotations to execute
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
            # Note: Setting this to True seems to do nothing.
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    head_joint.reset_position(False)
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError

        # create general publisher
        self.pub = rospy.Publisher('joint_state', JointState, queue_size=10)
        self.last_js = None

        # create subscriber and publisher for joint commands
        self.jnt_cmd_pub = rospy.Publisher('joint_command', JointCommand, queue_size = 1)
        self.sub = rospy.Subscriber('joint_command', JointCommand, self.cmd_cb, None, 2)

    def cmd_cb(self, msg):
        if msg.name == self.name:
            # Store the commanded power and rotations value,
            # limiting power to the range +/- power_max
            power = msg.effort / power_to_nm
            if power > self.power_max:
                power = self.power_max
            elif power < -self.power_max:
                power = -self.power_max
            self.power = int(power) # if power < 0 the motor will rotate self.rotations deg in reverse
            self.rotations = int(msg.rotations * 180 / math.pi) # convert msg.rotations from rad into deg 
            self.brake = msg.brake

    def trigger(self):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name.append(self.name)

        # Get the rotational position of the motor
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                rotation_count = head_joint.get_tacho().rotation_count
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
        js.effort.append(self.power * power_to_nm)  # this is just the commanded effort
        vel = 0
        if self.last_js:
            vel = (js.position[0] - self.last_js.position[0]) / (js.header.stamp - self.last_js.header.stamp).to_sec()
            js.velocity.append(vel)
        else:
            vel = 0
            js.velocity.append(vel)
        self.pub.publish(js)
        self.last_js = js

        if self.use_absolute_position:
            # If motor has done a full rotation, reset the encoder to zero
            if (rotation_count >= self.counts_per_rev) \
                    or (rotation_count <= -1.0 * self.counts_per_rev):
                USBTransmissionSuccess = False
                while not USBTransmissionSuccess:
                    try:
                        head_joint.reset_position(False)
                        USBTransmissionSuccess = True
                    except usb.core.USBError as e:
                        if e.errno == 110:
                            continue
                        else:
                            raise usb.core.USBError

        # send command
        if self.power != 0 : 
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    try : 
                        head_joint.turn(power = self.power, tacho_units = self.rotations, brake = self.brake)
                    except nxt.motor.BlockedException:
                        rospy.logwarn("Head joint is blocked.")
                        pass
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError
            self.jnt_cmd.header.frame_id=self.name+"_command"
            self.jnt_cmd.header.stamp=rospy.Time.now()
            self.jnt_cmd.name = self.name
            self.jnt_cmd.brake = self.brake
            self.jnt_cmd.rotations = 0
            self.jnt_cmd.effort = 0
            self.jnt_cmd_pub.publish(self.jnt_cmd)


class LaserJoint(Device):
    def __init__(self, params_laser_joint, comm):
        Device.__init__(self, params_laser_joint)
        self.name = "laser_joint"
        self.power_max = 50 # power is a value between -127 and 128 (an absolute value greater than 64 is recommended)
        self.brake = True # brake after rotations
        self.power = 0  # the commanded power value
        self.rotations = 0 # the commanded rotations to execute
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
            # Note: Setting this to True seems to do nothing.
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    laser_joint.reset_position(False)
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError

        # create general publisher
        self.pub = rospy.Publisher('joint_state', JointState, queue_size=10)
        self.last_js = None

        # create subscriber and publisher for joint commands
        self.jnt_cmd_pub = rospy.Publisher('joint_command', JointCommand, queue_size = 1)
        self.sub = rospy.Subscriber('joint_command', JointCommand, self.cmd_cb, None, 2)

    def cmd_cb(self, msg):
        if msg.name == self.name:
            # Store the commanded power value,
            # limited to the range +/-power_max
            power = msg.effort / power_to_nm
            if power > self.power_max:
                power = self.power_max
            elif power < -self.power_max:
                power = -self.power_max
            self.power = int(power) # if power < 0 the motor will rotate self.rotations deg in reverse
            self.rotations = int(msg.rotations * 180 / math.pi) # convert msg.rotations from rad into deg 
            self.brake = msg.brake

    def trigger(self):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name.append(self.name)

        # Get the rotational position of the motor
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                rotation_count = laser_joint.get_tacho().rotation_count
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
        js.effort.append(self.power * power_to_nm)  # this is just the commanded effort
        vel = 0
        if self.last_js:
            vel = (js.position[0] - self.last_js.position[0]) / (js.header.stamp - self.last_js.header.stamp).to_sec()
            js.velocity.append(vel)
        else:
            vel = 0
            js.velocity.append(vel)
        self.pub.publish(js)
        self.last_js = js

        if self.use_absolute_position:
            # If motor has done a full rotation, reset the encoder to zero
            if (rotation_count >= self.counts_per_rev) \
                    or (rotation_count <= -1.0 * self.counts_per_rev):
                USBTransmissionSuccess = False
                while not USBTransmissionSuccess:
                    try:
                        laser_joint.reset_position(False)
                        USBTransmissionSuccess = True
                    except usb.core.USBError as e:
                        if e.errno == 110:
                            continue
                        else:
                            raise usb.core.USBError

        # send command
        if self.power != 0 : 
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    try : 
                        laser_joint.turn(power = self.power, tacho_units = self.rotations, brake = self.brake)
                    except nxt.motor.BlockedException:
                        rospy.logwarn("Laser joint is blocked.")
                        pass
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError
            self.jnt_cmd.header.frame_id=self.name+"_command"
            self.jnt_cmd.header.stamp=rospy.Time.now()
            self.jnt_cmd.name = self.name
            self.jnt_cmd.brake = self.brake
            self.jnt_cmd.rotations = 0
            self.jnt_cmd.effort = 0
            self.jnt_cmd_pub.publish(self.jnt_cmd)
            

class ArmsJoint(Device):
    def __init__(self, params_arms_joint, comm):
        Device.__init__(self, params_arms_joint)
        self.name = "arms_joint"
        self.power_max = 50 # power is a value between -127 and 128 (an absolute value greater than 64 is recommended)
        self.brake = True # brake after rotations
        self.power = 0  # the commanded power value
        self.rotations = 0 # the commanded rotations to execute
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
            # Note: Setting this to True seems to do nothing.
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    arms_joint.reset_position(False)
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError

        # create general publisher
        self.pub = rospy.Publisher('joint_state', JointState, queue_size=10)
        self.last_js = None

        # create subscriber and publisher for joint commands
        self.jnt_cmd_pub = rospy.Publisher('joint_command', JointCommand, queue_size = 1)
        self.sub = rospy.Subscriber('joint_command', JointCommand, self.cmd_cb, None, 2)

    def cmd_cb(self, msg):
        if msg.name == self.name:
            # Store the commanded power and rotations value,
            # limiting power to the range +/- power_max
            power = msg.effort / power_to_nm
            if power > self.power_max:
                power = self.power_max
            elif power < -self.power_max:
                power = -self.power_max
            self.power = int(power) # if power < 0 the motor will rotate self.rotations deg in reverse
            self.rotations = int(msg.rotations * 180 / math.pi) # convert msg.rotations from rad into deg 
            self.brake = msg.brake

    def trigger(self):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name.append(self.name)

        # Get the rotational position of the motor
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                rotation_count = arms_joint.get_tacho().rotation_count
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
        js.effort.append(self.power * power_to_nm)  # this is just the commanded effort
        vel = 0
        if self.last_js:
            vel = (js.position[0] - self.last_js.position[0]) / (js.header.stamp - self.last_js.header.stamp).to_sec()
            js.velocity.append(vel)
        else:
            vel = 0
            js.velocity.append(vel)
        self.pub.publish(js)
        self.last_js = js

        if self.use_absolute_position:
            # If motor has done a full rotation, reset the encoder to zero
            if (rotation_count >= self.counts_per_rev) \
                    or (rotation_count <= -1.0 * self.counts_per_rev):
                USBTransmissionSuccess = False
                while not USBTransmissionSuccess:
                    try:
                        arms_joint.reset_position(False)
                        USBTransmissionSuccess = True
                    except usb.core.USBError as e:
                        if e.errno == 110:
                            continue
                        else:
                            raise usb.core.USBError

        # send command
        if self.power != 0 : 
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    try : 
                        arms_joint.turn(power = self.power, tacho_units = self.rotations, brake = self.brake)
                    except nxt.motor.BlockedException:
                        rospy.logwarn("Arms joint is blocked.")
                        pass
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError
            self.jnt_cmd.header.frame_id=self.name+"_command"
            self.jnt_cmd.header.stamp=rospy.Time.now()
            self.jnt_cmd.name = self.name
            self.jnt_cmd.brake = self.brake
            self.jnt_cmd.rotations = 0
            self.jnt_cmd.effort = 0
            self.jnt_cmd_pub.publish(self.jnt_cmd)


def main():
    # Initializing ROS node
    rospy.loginfo("Initializing nxt_ros node...")
    rospy.init_node('nxt2_ros')
    callback_handle_frequency = 10.0
    last_callback_handle = rospy.Time.now()

    # Connecting to NXT2 brick
    NXT2IsConnected = False
    global b2
    while not NXT2IsConnected:
        try:
            rospy.loginfo("Connecting to NXT2...")
            b2 = nxt.locator.find_one_brick(name="NXT2")
            NXT2IsConnected = True
            rospy.loginfo("NXT2 connected")
        except usb.core.USBError as e:
            if e.errno == 110:
                continue
            else:
                raise usb.core.USBError

    # Declaring sensors and motors
    components = []
    global sound_sensor
    sound_sensor = nxt.sensor.Sound(b2, PORT_1)
    rospy.loginfo("Connecting to sound_sensor on NXT2_PORT_1")
    params_sound_sensor = {'type': 'sound', 'name': 'sound_sensor', 'port': 'PORT_1', 'brick': 'NXT2', 'desired_frequency': 1}
    components.append(SoundSensor(params_sound_sensor, b2))
    global color_sensor
    color_sensor = nxt.sensor.Color20(b2, PORT_2)
    color_sensor.set_light_color(Type.COLORNONE)
    rospy.loginfo("Connecting to color_sensor on NXT2_PORT_2")
    rospy.loginfo("Subscribing to color_sensor_command publisher to turn on a specified color sensor light.")
    params_color_sensor = {'type': 'color', 'name': 'color_sensor', 'port': 'PORT_2', 'brick': 'NXT2', 'desired_frequency': 1}
    components.append(ColorSensor(params_color_sensor, b2))
    global line_following_sensor
    line_following_sensor = nxt.sensor.Light(b2, PORT_3)
    line_following_sensor.set_illuminated(active=True)
    rospy.loginfo("Connecting to line_following_sensor on NXT2_PORT_3")
    rospy.loginfo("Subscribing to line_following_sensor_set_illuminated publisher to control sensor light emission.")
    params_line_following_sensor = {'type': 'light', 'name': 'line_following_sensor', 'port': 'PORT_3', 'brick': 'NXT2', 'desired_frequency': 2}
    components.append(LineFollowingSensor(params_line_following_sensor, b2))
    global rfid_sensor  # the rfid_read program needs to be stored in the NXT2 brick
    rospy.loginfo("Connecting to RFID_sensor on NXT2_PORT_4")
    params_rfid_sensor = {'type': 'RFID', 'name': 'rfid_sensor', 'port': 'PORT_4', 'brick': 'NXT2','desired_frequency': 1}
    components.append(RFIDSensor(params_rfid_sensor, b2))
    global head_joint
    head_joint = Motor(b2, PORT_A)
    rospy.loginfo("Connecting to head_joint on NX2_PORT_A")
    params_head_joint = {'type': 'motor', 'name': 'head_joint', 'port': 'PORT_A', 'brick': 'NXT2', 'desired_frequency': 1}
    components.append(HeadJoint(params_head_joint, b2))
    global laser_joint
    laser_joint = Motor(b2, PORT_B)
    rospy.loginfo("Connecting to laser_joint on NXT2_PORT_B")
    params_laser_joint = {'type': 'motor', 'name': 'laser_joint', 'port': 'PORT_B', 'brick': 'NXT2', 'desired_frequency': 1}
    components.append(LaserJoint(params_laser_joint, b2))
    global arms_joint
    arms_joint = Motor(b2, PORT_C)
    rospy.loginfo("Connecting to arms_joint on NXT2_PORT_C")
    params_arms_joint = {'type': 'motor', 'name': 'arms_joint', 'port': 'PORT_C', 'brick': 'NXT2', 'desired_frequency': 1}
    components.append(ArmsJoint(params_arms_joint, b2))

    while not rospy.is_shutdown():
        my_lock.acquire()
        triggered = False
        for c in components:
            if c.needs_trigger() and not triggered:
                c.do_trigger()
                triggered = True
        my_lock.release()
        now = rospy.Time.now()
        if (now - last_callback_handle).to_sec() > 1.0/callback_handle_frequency:
            last_callback_handle = now
            rospy.sleep(0.01)

    def cleanup_node():
        rospy.loginfo("Shutting down NXT2 sensors and motors...")
        head_joint.run(0, 0)
        arms_joint.run(0, 0)
        laser_joint.run(0, 0)
        line_following_sensor.set_illuminated(active=False)
        color_sensor.set_light_color(Type.COLORNONE)
        b2.stop_program()
    rospy.on_shutdown(cleanup_node)


if __name__ == '__main__':
    main()