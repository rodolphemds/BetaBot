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

# Works with Python 3 and requires nxt-python

import roslib; roslib.load_manifest('nxt1_ros')
import usb.core
import nxt.locator
import rospy
import math
from nxt.sensor import S_1, S_2, S_3, S_4
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

global my_lock
my_lock = thread.allocate_lock()

global power_to_nm
power_to_nm = 0.01

# Publishers definition
right_bumper_publisher = rospy.Publisher("right_bumper", Contact, queue_size=1)
rospy.loginfo("Setting up publisher on right_bumper [nxt_msgs/Contact] and header frame identity right_bumper_link.")
left_bumper_publisher = rospy.Publisher("left_bumper", Contact, queue_size=1)
rospy.loginfo("Setting up publisher on left_bumper [nxt_msgs/Contact] and header frame identity left_bumper_link.")
back_bumper_publisher = rospy.Publisher("back_bumper", Contact, queue_size=1)
rospy.loginfo("Setting up publisher on back_bumper [nxt_msgs/Contact] and header frame identity back_bumper_link.")
ultrasonic_sensor_publisher = rospy.Publisher("ultrasonic_sensor", Range, queue_size=1)
rospy.loginfo("Setting up publisher on ultrasonic_sensor [nxt_msgs/Range] and header frame identity ultrasonic_sensor_link. ")

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


class RightTouchSensor(Device):
    def __init__(self, params_right_bumper, comm):
        Device.__init__(self, params_right_bumper)
    def trigger(self):
        right_bumper_report = Contact()
        right_bumper_report.header.frame_id="right_bumper_link"
        right_bumper_report.header.stamp = rospy.Time.now()
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                right_bumper_report.contact = right_bumper.get_sample()
                USBTransmissionSuccess = True
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
        right_bumper_publisher.publish(right_bumper_report)


class LeftTouchSensor(Device):
    def __init__(self, params_left_bumper, comm):
        Device.__init__(self, params_left_bumper)
    def trigger(self):
        left_bumper_report = Contact()
        left_bumper_report.header.frame_id="left_bumper_link"
        left_bumper_report.header.stamp = rospy.Time.now()
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                left_bumper_report.contact = left_bumper.get_sample()
                USBTransmissionSuccess = True
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
        left_bumper_publisher.publish(left_bumper_report)


class BackTouchSensor(Device):
    def __init__(self, params_back_bumper, comm):
        Device.__init__(self, params_back_bumper)
    def trigger(self):
        back_bumper_report = Contact()
        back_bumper_report.header.frame_id="back_bumper_link"
        back_bumper_report.header.stamp = rospy.Time.now()
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                back_bumper_report.contact = back_bumper.get_sample()
                USBTransmissionSuccess = True
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
        back_bumper_publisher.publish(back_bumper_report)


class UltrasonicSensor(Device):
    def __init__(self, params_ultrasonic_sensor, comm):
        Device.__init__(self, params_ultrasonic_sensor)
    def trigger(self):
        ultrasonic_sensor_report = Range()
        ultrasonic_sensor_report.header.frame_id = "ultrasonic_sensor_link"
        ultrasonic_sensor_report.header.stamp = rospy.Time.now()
        ultrasonic_sensor_report.radiation_type = 0
        ultrasonic_sensor_report.field_of_view = 0.5235987756 # rad
        ultrasonic_sensor_report.min_range = 0.07
        ultrasonic_sensor_report.max_range = 2.54
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                ultrasonic_sensor_report.range = ultrasonic_sensor.get_sample() / 100.0
                USBTransmissionSuccess = True
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
        ultrasonic_sensor_publisher.publish(ultrasonic_sensor_report)


class LeftTreadMotor(Device):
    def __init__(self, params_left_tread, comm):
        Device.__init__(self, params_left_tread)
        self.name = "left_tread"
        USBTransmissionSuccess = False
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
            # Upon initialization, reset the motor encoder to zero.
            # Note: Setting this to True seems to do nothing.
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    left_tread.reset_position(False)
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError

        # create general publisher
        self.pub = rospy.Publisher('joint_state', JointState, queue_size=10)
        self.last_js = None
        rospy.loginfo("Connecting to joint_state motor publisher.")

        # create subscriber
        self.sub = rospy.Subscriber('joint_command', JointCommand, self.cmd_cb, None, 2)

    def cmd_cb(self, msg):
        if msg.name == self.name:
            # Store the commanded power value,
            # limited to the range +/-125
            cmd = msg.effort / power_to_nm
            if cmd > self.power_max:
                cmd = self.power_max
            elif cmd < -self.power_max:
                cmd = -self.power_max
            self.cmd = cmd
            self.brake = msg.brake

    def trigger(self):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name.append(self.name)

        # Get the rotational position of the motor
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                rotation_count = left_tread.get_tacho().rotation_count
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
        js.effort.append(self.cmd * power_to_nm)  # this is just the commanded effort
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
                        left_tread.reset_position(False)
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
                left_tread.run(-int(self.cmd), 0) # backward and forward are inversed for the treads motor
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
                    left_tread.brake()
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError


class RightTreadMotor(Device):
    def __init__(self, params_right_tread, comm):
        Device.__init__(self, params_right_tread)
        self.name = "right_tread"
        USBTransmissionSuccess = False
        self.power_max = 125
        self.brake = False
        self.cmd = 0 # the commanded power value

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
            # Upon initialization, reset the motor encoder to zero.
            # Note: Setting this to True seems to do nothing.
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    right_tread.reset_position(False)
                    USBTransmissionSuccess = True
                except usb.core.USBError as e :
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError

        # create general publisher
        self.pub = rospy.Publisher('joint_state', JointState, queue_size=10)
        self.last_js = None
        rospy.loginfo("Connecting to joint_state motor publisher.")

        # create subscriber
        self.sub = rospy.Subscriber('joint_command', JointCommand, self.cmd_cb, None, 2)

    def cmd_cb(self, msg):
        if msg.name == self.name:
            # Store the commanded power value,
            # limited to the range +/-125
            cmd = msg.effort / power_to_nm
            if cmd > self.power_max:
                cmd = self.power_max
            elif cmd < -self.power_max:
                cmd = -self.power_max
            self.cmd = cmd
            self.brake = msg.brake

    def trigger(self):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name.append(self.name)

        # Get the rotational position of the motor
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                rotation_count = right_tread.get_tacho().rotation_count
                USBTransmissionSuccess = True
            except usb.core.USBError as e :
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
        position_in_radians = rotation_count * math.pi / 180.0

        if self.use_absolute_position:
            # Check if we have gone backwards
            # past the starting position
            if position_in_radians < 0.0:
                position_in_radians = 2.0*math.pi + position_in_radians

        js.position.append(position_in_radians)
        js.effort.append(self.cmd * power_to_nm) # this is just the commanded effort
        vel = 0
        if self.last_js:
            vel = (js.position[0]-self.last_js.position[0])/(js.header.stamp-self.last_js.header.stamp).to_sec()
            js.velocity.append(vel)
        else:
            vel = 0
            js.velocity.append(vel)
        self.pub.publish(js)
        self.last_js = js

        if self.use_absolute_position:
            # If motor has done a full rotation, reset the encoder to zero
            if (rotation_count >= self.counts_per_rev) \
            or (rotation_count <= -1.0*self.counts_per_rev):
                USBTransmissionSuccess = False
                while not USBTransmissionSuccess:
                    try:
                        right_tread.reset_position(False)
                        USBTransmissionSuccess = True
                    except usb.core.USBError as e :
                        if e.errno == 110:
                            continue
                        else:
                            raise usb.core.USBError

        # send command
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                right_tread.run(- int(self.cmd), 0) # backward and forward are inversed for the treads motor
                USBTransmissionSuccess = True
            except usb.core.USBError as e :
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
        if self.brake:
            USBTransmissionSuccess = False
            while not USBTransmissionSuccess:
                try:
                    right_tread.brake()
                    USBTransmissionSuccess = True
                except usb.core.USBError as e:
                    if e.errno == 110:
                        continue
                    else:
                        raise usb.core.USBError


class TorsoJoint(Device):
    def __init__(self, params_torso_joint, comm):
        Device.__init__(self, params_torso_joint)
        self.name = "torso_joint"
        self.power_max = 100 # power is a value between -127 and 128 (an absolute value greater than 64 is recommended)
        self.brake = False # brake after rotations
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
                    torso_joint.reset_position(False)
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
            self.rotations = int(msg.rotations) * 180 / math.pi # convert msg.rotations from rad into deg 
            self.brake = msg.brake

    def trigger(self):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name.append(self.name)

        # Get the rotational position of the motor
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                rotation_count = torso_joint.get_tacho().rotation_count
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
                        torso_joint.reset_position(False)
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
                        torso_joint.turn(power = self.power, tacho_units = self.rotations, brake = self.brake)
                    except nxt.motor.BlockedException:
                        rospy.logwarn("Torso joint is blocked.")
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
    rospy.init_node('nxt1_ros')
    callback_handle_frequency = 10.0
    last_callback_handle = rospy.Time.now()

    # Connecting to NXT1 brick
    NXT1IsConnected = False
    global b1
    while not NXT1IsConnected:
        try:
            rospy.loginfo("Connecting to NXT1...")
            b1 = nxt.locator.find_one_brick(name="NXT1")
            NXT1IsConnected = True
            rospy.loginfo("NXT1 connected")
        except usb.core.USBError as e:
            if e.errno == 110:
                continue
            else:
                raise usb.core.USBError

    # Declaring sensors and motors
    components = []
    global right_bumper
    right_bumper = Touch(b1, PORT_1)
    rospy.loginfo("Connecting to right_bumper on NXT1_PORT_1")
    params_right_bumper = {'type': 'touch', 'name': 'right_bumper', 'port': 'PORT_1', 'brick': 'NXT1', 'desired_frequency': 2}
    components.append(RightTouchSensor(params_right_bumper, b1))
    global left_bumper
    left_bumper = Touch(b1, PORT_2)
    rospy.loginfo("Connecting to left_bumper on NXT1_PORT_2")
    params_left_bumper = {'type': 'touch', 'name': 'left_bumper', 'port': 'PORT_2', 'brick': 'NXT1', 'desired_frequency': 2}
    components.append(LeftTouchSensor(params_left_bumper, b1))
    global back_bumper
    back_bumper = Touch(b1, PORT_3)
    rospy.loginfo("Connecting to back_bumper on NXT1_PORT_3")
    params_back_bumper = {'type': 'touch', 'name': 'back_bumper', 'port': 'PORT_3', 'brick': 'NXT1', 'desired_frequency': 2}
    components.append(BackTouchSensor(params_back_bumper, b1))
    global ultrasonic_sensor
    ultrasonic_sensor = Ultrasonic(b1, PORT_4)
    rospy.loginfo("Connecting to ultrasonic_sensor on NXT1_PORT_4")
    params_ultrasonic_sensor = {'type': 'ultrasonic', 'name': 'ultrasonic_sensor', 'port': 'PORT_4', 'brick': 'NXT1', 'desired_frequency': 3}
    components.append(UltrasonicSensor(params_ultrasonic_sensor, b1))
    global right_tread
    right_tread = Motor(b1, PORT_A)
    rospy.loginfo("Connecting to right_tread on NXT1_PORT_A")
    params_right_tread = {'type': 'motor', 'name': 'right_tread', 'port': 'PORT_A', 'brick': 'NXT1', 'desired_frequency': 20}
    components.append(RightTreadMotor(params_right_tread, b1))
    global torso_joint
    torso_joint = Motor(b1, PORT_B)
    rospy.loginfo("Connecting to torso_joint on NXT1_PORT_B")
    params_torso_joint = {'type': 'motor', 'name': 'torso_joint', 'port': 'PORT_B', 'brick': 'NXT1', 'desired_frequency': 10}
    components.append(TorsoJoint(params_torso_joint, b1))
    global left_tread
    left_tread = Motor(b1, PORT_C)
    rospy.loginfo("Connecting to left_tread on NXT1_PORT_C")
    params_left_tread = {'type': 'motor', 'name': 'left_tread', 'port': 'PORT_C', 'brick': 'NXT1', 'desired_frequency': 20}
    components.append(LeftTreadMotor(params_left_tread, b1))

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
        rospy.loginfo("Shutting down NXT1 sensors and motors...")
        right_tread.run(0, 0)
        left_tread.run(0, 0)
        torso_joint.run(0, 0)
        ultrasonic_sensor.command(nxt.sensor.Ultrasonic.Commands.OFF)
    rospy.on_shutdown(cleanup_node)


if __name__ == '__main__':
    main()