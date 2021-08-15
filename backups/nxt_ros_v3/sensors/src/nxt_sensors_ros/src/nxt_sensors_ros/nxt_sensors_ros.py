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

import roslib; roslib.load_manifest('nxt_sensors_ros')
import nxt.locator
import rospy
import math
from nxt.sensor import PORT_1, PORT_2, PORT_3, PORT_4
from nxt.sensor import Type
from nxt.sensor import *
from nxt.brick import Brick
from nxt.locator import find_one_brick
import nxt.sensor
import thread
import subprocess
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String
from nxt_msgs.msg import Contact, Color, Light, RFID, Sound
from PyKDL import Rotation
from time import sleep

global my_lock
my_lock = thread.allocate_lock()

# Publishers definition
color_sensor_publisher = rospy.Publisher("color_sensor", Color, queue_size=1)
rospy.loginfo("Setting up publisher on color_sensor [nxt_msgs/Color] and header frame identity color_sensor_link.")
line_following_sensor_publisher = rospy.Publisher("line_following_sensor", Light, queue_size=1)
rospy.loginfo("Setting up publisher on line_following_sensor [nxt_msgs/Light] and header frame identity line_following_sensor_link.")
rfid_sensor_publisher = rospy.Publisher("rfid_sensor", RFID, queue_size=1)
rospy.loginfo("Setting up publisher on rfid_sensor [nxt_msgs/RFID] and header frame identity rfid_sensor_link.")
sound_sensor_publisher = rospy.Publisher("sound_sensor", Sound, queue_size=1)
rospy.loginfo("Setting up publisher on sound_sensor [nxt_msgs/Sound] and header frame identity sound_sensor_link.")
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

    def color_sensor_command_callback(self, color_sensor_command):
        color_sensor_command_color = color_sensor_command.data
        rospy.loginfo("Subscribing to color_sensor_command publisher to turn on a specified color sensor light.")

    def trigger(self):
        color_sensor_command_color = "NONE"
        color_sensor_command_subscriber = rospy.Subscriber("color_sensor_command", String,
                                                           self.color_sensor_command_callback, None, 2)
        if color_sensor_command_color != "":
            if color_sensor_command_color == "RED":
                color_type = Type.COLORRED
                USBTransmissionSuccess = False
                while not USBTransmissionSuccess:
                    try:
                        color_sensor.set_light_color(color_type)
                        USBTransmissionSuccess = True
                    except usb.core.USBError as e:
                        if e.errno == 110:
                            continue
                        else:
                            raise usb.core.USBError
            if color_sensor_command_color == "GREEN":
                color_type = Type.COLORGREEN
                USBTransmissionSuccess = False
                while not USBTransmissionSuccess:
                    try:
                        color_sensor.set_light_color(color_type)
                        USBTransmissionSuccess = True
                    except usb.core.USBError as e:
                        if e.errno == 110:
                            continue
                        else:
                            raise usb.core.USBError
            if color_sensor_command_color == "BLUE":
                color_type = Type.COLORBLUE
                USBTransmissionSuccess = False
                while not USBTransmissionSuccess:
                    try:
                        color_sensor.set_light_color(color_type)
                        USBTransmissionSuccess = True
                    except usb.core.USBError as e:
                        if e.errno == 110:
                            continue
                        else:
                            raise usb.core.USBError
            if color_sensor_command_color == "NONE":
                color_type = Type.COLORNONE
                USBTransmissionSuccess = False
                while not USBTransmissionSuccess:
                    try:
                        color_sensor.set_light_color(color_type)
                        USBTransmissionSuccess = True
                    except usb.core.USBError as e:
                        if e.errno == 110:
                            continue
                        else:
                            raise usb.core.USBError
            if color_sensor_command_color == "FULL":
                color_type = Type.COLORFULL
                USBTransmissionSuccess = False
                while not USBTransmissionSuccess:
                    try:
                        color_sensor.set_light_color(color_type)
                        USBTransmissionSuccess = True
                    except usb.core.USBError as e:
                        if e.errno == 110:
                            continue
                        else:
                            raise usb.core.USBError
        color_sensor_report = Color()
        color_sensor_report.header.frame_id = "color_sensor_link"
        color_sensor_report.header.stamp = rospy.Time.now()
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                color_sensor_report.intensity = color_sensor.get_reflected_light(color_on)
                USBTransmissionSuccess = True
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                color_code = color_sensor.get_color()
                USBTransmissionSuccess = True
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
        if color_code == 1:  # black
            color_sensor_report.r = 0.0
            color_sensor_report.g = 0.0
            color_sensor_report.b = 0.0
            color_sensor_report.color = "black"
        elif color_code == 2:  # blue
            color_sensor_report.r = 0.0
            color_sensor_report.g = 0.0
            color_sensor_report.b = 1.0
            color_sensor_report.color = "blue"
        elif color_code == 3:  # green
            color_sensor_report.r = 0.0
            color_sensor_report.g = 1.0
            color_sensor_report.b = 0.0
            color_sensor_report.color = "green"
        elif color_code == 4:  # yellow
            color_sensor_report.r = 1.0
            color_sensor_report.g = 1.0
            color_sensor_report.b = 0.0
            color_sensor_report.color = "yellow"
        elif color_code == 5:  # red
            color_sensor_report.r = 1.0
            color_sensor_report.g = 0.0
            color_sensor_report.b = 1.0
            color_sensor_report.color = "red"
        elif color_code == 6:  # white
            color_sensor_report.r = 1.0
            color_sensor_report.g = 1.0
            color_sensor_report.b = 1.0
            color_sensor_report.color = "white"
        else:
            color_sensor_report.r = 9
            color_sensor_report.g = 9
            color_sensor_report.b = 9
            color_sensor_report.color = "undefined"
            rospy.logerr('Undefined color of color sensor')

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
    def line_following_sensor_command_callback(self, line_following_sensor_light_on):
        rospy.loginfo("Subscribing to line_following_sensor_set_illuminated publisher to control sensor light emission.")
        line_following_sensor_illuminated = False
        line_following_sensor_illuminated = line_following_sensor_light_on.data
    def trigger(self):
        line_following_sensor_command_subscriber = rospy.Subscriber("line_following_sensor_set_illuminated", Bool,
                                                                    self.line_following_sensor_command_callback, None, 2)
        if line_following_sensor_illuminated:
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
        line_following_sensor_report = Light()
        line_following_sensor_report.header.frame_id = "line_following_sensor_link"
        line_following_sensor_report.header.stamp = rospy.Time.now()
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                line_following_sensor_report.ambiant_light_intensity = line_following_sensor.get_lightness()
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
    def trigger(self):
        rfid_sensor_report = RFID()
        rfid_sensor_report.header.frame_id="rfid_sensor_link"
        rfid_sensor_report.header.stamp = rospy.Time.now()
        #brick.stop_program()
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                b2.start_program('rfid_read.rxe')
                USBTransmissionSuccess = True
            except usb.core.USBError as e:
                if e.errno == 110:
                    continue
                else:
                    raise usb.core.USBError
        sleep(1)
        USBTransmissionSuccess = False
        while not USBTransmissionSuccess:
            try:
                read_data=nxt.brick.FileReader(b2, "rfid_read.txt")
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


def main():
    # Initializing ROS node
    rospy.loginfo("Initializing nxt_ros node...")
    rospy.init_node('nxt_sensors_ros')
    callback_handle_frequency = 10.0
    last_callback_handle = rospy.Time.now()

    # Connecting to NXT bricks
    NXT1IsConnected = False
    while not NXT1IsConnected :
        try :
            rospy.loginfo("Connecting to NXT1...")
            global b1
            b1 = nxt.locator.find_one_brick(name="NXT1")
            NXT1IsConnected = True
            rospy.loginfo("NXT1 connected")
        except usb.core.USBError as e :
            if e.errno == 110:
                continue
            else:
                raise usb.core.USBError
    sleep(1)
    NXT2IsConnected = False
    while not NXT2IsConnected:
        try:
            rospy.loginfo("Connecting to NXT2...")
            global b2
            b2 = nxt.locator.find_one_brick(name="NXT2")
            NXT2IsConnected = True
            rospy.loginfo("NXT2 connected")
        except usb.core.USBError as e :
            if e.errno == 110:
                continue
            else:
                raise usb.core.USBError
    sleep(1)

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
    global sound_sensor
    sound_sensor = nxt.sensor.Sound(b2, PORT_1)
    rospy.loginfo("Connecting to sound_sensor on NXT2_PORT_1")
    params_sound_sensor = {'type': 'sound', 'name': 'sound_sensor', 'port': 'PORT_1', 'brick': 'NXT2', 'desired_frequency': 5}
    components.append(SoundSensor(params_sound_sensor, b2))
    global color_sensor
    color_sensor = nxt.sensor.Color20(b2, PORT_2)
    rospy.loginfo("Connecting to color_sensor on NXT2_PORT_2")
    params_color_sensor = {'type': 'color', 'name': 'color_sensor', 'port': 'PORT_2', 'brick': 'NXT2',  'desired_frequency': 1}
    components.append(ColorSensor(params_color_sensor, b2))
    global line_following_sensor
    line_following_sensor = nxt.sensor.Light(b2, PORT_3)
    rospy.loginfo("Connecting to line_following_sensor on NXT2_PORT_3")
    params_line_following_sensor = {'type': 'light', 'name': 'line_following_sensor', 'port': 'PORT_3', 'brick': 'NXT2', 'desired_frequency': 5}
    components.append(LineFollowingSensor(params_line_following_sensor, b2))
    global rfid_sensor # the rfid_read program needs to be stored in the NXT2 brick
    rospy.loginfo("Connecting to RFID_sensor on NXT2_PORT_4")
    params_rfid_sensor = {'type': 'RFID', 'name': 'rfid_sensor', 'port': 'PORT_4', 'brick': 'NXT2', 'desired_frequency': 1}
    components.append(RFIDSensor(params_rfid_sensor, b2))

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

    # Define exit handler
    def cleanup_node():
        rospy.loginfo("Shutting down sensors connected to NXT ports...")
        line_following_sensor.set_illuminated(active=False)
        ultrasonic_sensor.command(nxt.sensor.Ultrasonic.Commands.OFF)
        color_sensor.set_light_color(Type.COLORNONE)
    rospy.on_shutdown(cleanup_node)

if __name__ == '__main__':
    main()