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

import roslib; roslib.load_manifest('nxt_ros')
import usb.core
import nxt.locator
import rospy
import math
from nxt.motor import Motor, SynchronizedMotors, PORT_A, PORT_B, PORT_C
from nxt.sensor import PORT_1, PORT_2, PORT_3, PORT_4
from nxt.sensor import Type
from nxt.sensor import *
from nxt.brick import Brick
from nxt.locator import find_one_brick
import nxt.sensor
import nxt.motor
import thread
import subprocess
from sensor_msgs.msg import JointState, Imu, Range
from std_msgs.msg import Bool, String
from nxt_msgs.msg import Contact, JointCommand, Color, Light, Gyro, Accelerometer, RFID, Sound, JointSpecificPosition
from PyKDL import Rotation
from time import sleep

global my_lock
my_lock = thread.allocate_lock()

def check_params(ns, params):
	for p in params:
		if not rospy.get_param(ns+'/'+p):
			return False
	return True


# base class for sensors
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


class Motor(Device):
	def __init__(self, params, comm):
		Device.__init__(self, params)
		# create motor
		self.name = params['name']
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				self.motor = nxt.motor.Motor(comm, eval(params['port']))
				USBTransmissionSuccess = True
			except usb.core.USBError as e :
				if e.errno == 110:
					continue
				else:
					raise usb.core.USBError
		self.power_to_nm = params['power_to_nm']
		self.power_max = params['power_max']
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
					self.motor.reset_position(False)
					USBTransmissionSuccess = True
				except usb.core.USBError as e :
					if e.errno == 110:
						continue
					else:
						raise usb.core.USBError

		# create general publisher
		self.pub = rospy.Publisher('joint_state', JointState, queue_size=10)
		self.last_js = None
		rospy.loginfo("Setting up unspecific motor publisher with name joint_state.")

		# create publishers for position name of torso, head, laser and arms joints
		if self.name == "torso_joint" or self.name == "head_joint" or self.name == "laser_joint" or self.name == "arms_joint":
			self.specificPub = rospy.Publisher(self.name + '_position_name', JointSpecificPosition, queue_size=1)
			rospy.loginfo("Setting up motor specific position name publisher with name " + self.name + "_position_name")
		# create subscriber
		self.sub = rospy.Subscriber('joint_command', JointCommand, self.cmd_cb, None, 2)

		# Register the shutdown method
		rospy.on_shutdown(self.shutdown)

	def cmd_cb(self, msg):
		if msg.name == self.name:
			# Store the commanded power value,
			# limited to the range +/-125
			cmd = msg.effort / self.power_to_nm
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
				rotation_count = self.motor.get_tacho().rotation_count
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
		js.effort.append(self.cmd * self.power_to_nm) # this is just the commanded effort
		vel = 0
		if self.last_js:
			vel = (js.position[0]-self.last_js.position[0])/(js.header.stamp-self.last_js.header.stamp).to_sec()
			js.velocity.append(vel)
		else:
			vel = 0
			js.velocity.append(vel)
		self.pub.publish(js)
		self.last_js = js

		# Send specific position name of torso, head, laser and arms joints
		if self.name == "torso_joint" or self.name == "head_joint" or self.name == "laser_joint" or self.name == "arms_joint":
			self.position_name = JointSpecificPosition()
			self.position_name.header.stamp = rospy.Time.now()
			USBTransmissionSuccess = False
			while not USBTransmissionSuccess:
				try:
					self.tacho_count = self.motor.get_tacho().rotation_count
					USBTransmissionSuccess = True
				except usb.core.USBError as e :
					if e.errno == 110:
						continue
					else:
						raise usb.core.USBError
			if self.name == "torso_joint" :
				if self.tacho_count == 0 :
					self.position_name.position_name = "torso_up"
				elif self.tacho_count == 950 :
					self.position_name.position_name = "torso_down"
				elif self.tacho_count == 450 :
					self.position_name.position_name = "torso_middle"
				elif self.tacho_count == 200 :
					self.position_name.position_name = "torso_middle_up"
				elif self.tacho_count == 700:
					self.position_name.position_name = "torso_middle_down"
			elif self.name == "head_joint" :
				if self.tacho_count == -410 :
					self.position_name.position_name = "head_max_left"
				elif self.tacho_count == 410 :
					self.position_name.position_name = "head_max_right"
				elif self.tacho_count == 0 :
					self.position_name.position_name = "head_center"
			elif self.name == "laser_joint" :
				if self.tacho_count == 95 :
					self.position_name.position_name = "laser_up"
				elif self.tacho_count == 0 :
					self.position_name.position_name = "laser_down"
			elif self.name == "arms_joint" :
				if self.tacho_count == 250 :
					self.position_name.position_name = "forearms_up"
				elif self.tacho_count == 180 :
					self.position_name.position_name = "hands_closed"
				elif self.tacho_count == 0 :
					self.position_name.position_name = "hands_open"
				elif self.tacho_count == -140 :
					self.position_name.position_name = "wrist_rotated"
			self.specificPub.publish(self.position_name)

		if self.use_absolute_position:
			# If motor has done a full rotation, reset the encoder to zero
			if (rotation_count >= self.counts_per_rev) \
			or (rotation_count <= -1.0*self.counts_per_rev):
				USBTransmissionSuccess = False
				while not USBTransmissionSuccess:
					try:
						self.motor.reset_position(False)
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
				self.motor.run(int(self.cmd), 0)
				USBTransmissionSuccess = True
			except usb.core.USBError as e :
				if e.errno == 110:
					continue
				else:
					raise usb.core.USBErro
		if self.brake:
			USBTransmissionSuccess = False
			while not USBTransmissionSuccess:
				try:
					self.motor.brake()
					USBTransmissionSuccess = True
				except usb.core.USBError as e:
					if e.errno == 110:
						continue
					else:
						raise usb.core.USBErro


	def shutdown(self):
		# Stop motors at ROS shutdown
		print("Killing the motor %s", self.name)
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				self.motor.run(0, 0)
				USBTransmissionSuccess = True
			except usb.core.USBError as e :
				if e.errno == 110:
					continue
				else:
					raise usb.core.USBError

class TouchSensor(Device):
	def __init__(self, params, comm):
		Device.__init__(self, params)
		# create touch sensor
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				self.touch = nxt.sensor.Touch(comm, eval(params['port']))
				USBTransmissionSuccess = True
			except usb.core.USBError as e :
				if e.errno == 110:
					continue
				else:
					raise usb.core.USBError
		self.frame_id = params['frame_id']

		# create publisher
		self.pub = rospy.Publisher(params['name'], Contact)
		rospy.loginfo("Setting up touch sensor publisher with name %s and header frame identity %s.", params['name'],
					  params['frame_id'])

	def trigger(self):
		ct = Contact()
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				ct.contact = self.touch.get_sample()
				USBTransmissionSuccess = True
			except usb.core.USBError as e :
				if e.errno == 110:
					continue
				else:
					raise usb.core.USBError
		ct.header.frame_id = self.frame_id
		ct.header.stamp = rospy.Time.now()
		self.pub.publish(ct)


class UltraSonicSensor(Device):
	def __init__(self, params, comm):
		Device.__init__(self, params)
		# Create ultrasonic sensor
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				self.ultrasonic = nxt.sensor.Ultrasonic(comm, eval(params['port']))
				USBTransmissionSuccess = True
			except usb.core.USBError as e :
				if e.errno == 110:
					continue
				else:
					raise usb.core.USBError
		self.frame_id = params['frame_id']
		self.spread = params['spread_angle']
		self.min_range = params['min_range']
		self.max_range = params['max_range']
		self.field_of_view = params['field_of_view']
		self.radiation_type = params['radiation_type']

		# Enable the sensor
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				mode = nxt.sensor.Ultrasonic.Commands.CONTINUOUS_MEASUREMENT
				self.ultrasonic.command(mode)
				USBTransmissionSuccess = True
			except usb.core.USBError as e :
				if e.errno == 110:
					continue
				else:
					raise usb.core.USBError

		# Create publisher
		self.pub = rospy.Publisher(params['name'], Range, queue_size=10)
		rospy.loginfo("Setting up ultrasonic sensor publisher with name %s and header frame identity %s.", params['name'],
					  params['frame_id'])

	def trigger(self):
		us = Range()
		us.header.frame_id = self.frame_id
		us.header.stamp = rospy.Time.now()

		# Read the distance and convert to meters
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				us.range = self.ultrasonic.get_sample()/100.0
				USBTransmissionSuccess = True
			except usb.core.USBError as e :
				if e.errno == 110:
					continue
				else:
					raise usb.core.USBError

		# Publish the ultrasonic sensor specifications as defined by the user
		us.min_range = self.min_range
		us.max_range = self.max_range
		us.radiation_type = self.radiation_type
		us.field_of_view = self.field_of_view
		self.pub.publish(us)


class GyroSensor(Device):
	"""
	This uses the HiTechnic gyro sensor to
	measure angular velocity around the x,y,z axes
	and calculate an estimate of the orientation.
	"""
	def __init__(self, params, comm):
		Device.__init__(self, params)
		#create gyro sensor
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				self.gyro = nxt.sensor.HTGyro(comm, eval(params['port']))
				USBTransmissionSuccess = True
			except usb.core.USBError as e :
				if e.errno == 110:
					continue
				else:
					raise usb.core.USBError
		self.frame_id = params['frame_id']
		self.orientation = 0.0
		self.offset = params['offset']
		self.prev_time = rospy.Time.now()

		# calibrate
		rospy.loginfo("Calibrating gyroscope. Please do not move the robot.")
		start_time = rospy.Time.now()
		cal_duration = rospy.Duration(2.0)
		offset = 0
		tmp_time = rospy.Time.now()
		while rospy.Time.now() < start_time + cal_duration:
			rospy.sleep(0.01)
			USBTransmissionSuccess = False
			while not USBTransmissionSuccess:
				try:
					sample = self.gyro.get_sample()
					USBTransmissionSuccess = True
				except usb.core.USBError as e :
					if e.errno == 110:
						continue
					else:
						raise usb.core.USBError
			now = rospy.Time.now()
			offset += (sample * (now - tmp_time).to_sec())
			tmp_time = now
		self.offset = offset / (tmp_time - start_time).to_sec()
		rospy.loginfo("Gyroscope calibrated with offset %f."%self.offset)

		# create publisher
		self.pub = rospy.Publisher(params['name'], Gyro, queue_size=10)
		rospy.loginfo("Setting up gyroscope publisher with name %s and header frame identity %s.", params['name'],
					  params['frame_id'])

		# create publisher
		self.pub2 = rospy.Publisher(params['name']+"_imu", Imu, queue_size=10)

	def trigger(self):
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				sample = self.gyro.get_sample()
				USBTransmissionSuccess = True
			except usb.core.USBError as e :
				if e.errno == 110:
					continue
				else:
					raise usb.core.USBError
		gs = Gyro()
		gs.header.frame_id = self.frame_id
		gs.header.stamp = rospy.Time.now()
		gs.calibration_offset.x = 0.0
		gs.calibration_offset.y = 0.0
		gs.calibration_offset.z = self.offset
		gs.angular_velocity.x = 0.0
		gs.angular_velocity.y = 0.0
		gs.angular_velocity.z = (sample-self.offset)*math.pi/180.0
		gs.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 1]
		self.pub.publish(gs)

		imu = Imu()
		imu.header.frame_id = self.frame_id
		imu.header.stamp = rospy.Time.now()
		imu.angular_velocity.x = 0.0
		imu.angular_velocity.y = 0.0
		imu.angular_velocity.z = (sample-self.offset)*math.pi/180.0
		imu.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 1]
		imu.orientation_covariance = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.1]
		self.orientation += imu.angular_velocity.z * (imu.header.stamp - self.prev_time).to_sec()
		self.prev_time = imu.header.stamp
		(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w) = Rotation.RotZ(self.orientation).GetQuaternion()
		self.pub2.publish(imu)


class AccelerometerSensor(Device):
	"""
	This uses the HiTechnic accelerometer sensor
	to measure acceleration on the x,y,z axes.
	"""
	def __init__(self, params, comm):
		Device.__init__(self, params)
		# create accelerometer sensor
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				self.accel = nxt.sensor.HTAccelerometer(comm, eval(params['port']))
				USBTransmissionSuccess = True
			except usb.core.USBError as e :
				if e.errno == 110:
					continue
				else:
					raise usb.core.USBError
		self.frame_id = params['frame_id']

		# create publisher
		self.pub = rospy.Publisher(params['name'], Accelerometer)
		rospy.loginfo("Setting up accelerometer sensor publisher with name %s and header frame identity %s.", params['name'],
					  params['frame_id'])

	def trigger(self):
		gs = Accelerometer()
		gs.header.frame_id = self.frame_id
		gs.header.stamp = rospy.Time.now()
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				x,y,z = self.accel.get_acceleration()
				USBTransmissionSuccess = True
			except usb.core.USBError as e :
				if e.errno == 110:
					continue
				else:
					raise usb.core.USBError

		gs.linear_acceleration.x = x*9.8
		gs.linear_acceleration.y = y*9.8
		gs.linear_acceleration.z = z*9.8
		gs.linear_acceleration_covariance = [1, 0, 0, 0, 1, 0, 0, 0, 1]
		self.pub.publish(gs)


class ColorSensor(Device):
	"""
	This uses the NXT 2.0 RGB color sensor
	to detect 1 of 6 preset color values.
	"""
	def __init__(self, params, comm):
		Device.__init__(self, params)
		# create color sensor
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				self.color = nxt.sensor.Color20(comm, eval(params['port']))
				USBTransmissionSuccess = True
			except usb.core.USBError as e :
				if e.errno == 110:
					continue
				else:
					raise usb.core.USBError
		self.frame_id = params['frame_id']

		# Turn on the LED light
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				self.color.set_light_color(Type.COLORFULL) # white
				USBTransmissionSuccess = True
			except usb.core.USBError as e :
				if e.errno == 110:
					continue
				else:
					raise usb.core.USBError

		# create publisher
		self.pub = rospy.Publisher(params['name'], Color)
		rospy.loginfo("Setting up color sensor publisher with name %s and header frame identity %s.", params['name'],
					  params['frame_id'])

	def trigger(self):
		co = Color()
		co.header.frame_id = self.frame_id
		co.header.stamp = rospy.Time.now()
		# Get the color value (an integer from 1 to 6)
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				color_code = self.color.get_color()
				USBTransmissionSuccess = True
			except usb.core.USBError as e :
				if e.errno == 110:
					continue
				else:
					raise usb.core.USBError
		if color_code == 1:  # black
			co.r = 0.0
			co.g = 0.0
			co.b = 0.0
			co.color = "black"
		elif color_code == 2: # blue
			co.r = 0.0
			co.g = 0.0
			co.b = 1.0
			co.color = "blue"
		elif color_code == 3: # green
			co.r = 0.0
			co.g = 1.0
			co.b = 0.0
			co.color = "green"
		elif color_code == 4: # yellow
			co.r = 1.0
			co.g = 1.0
			co.b = 0.0
			co.color = "yellow"
		elif color_code == 5: # red
			co.r = 1.0
			co.g = 0.0
			co.b = 0.0
			co.color = "red"
		elif color_code == 6: # white
			co.r = 1.0
			co.g = 1.0
			co.b = 1.0
			co.color = "white"
		else:
			rospy.logerr('Undefined color of color sensor.')
		self.pub.publish(co)


class IntensitySensor(Device):
	"""
	This uses the NXT 2.0 RGB color sensor to
	measure the intensity of reflected light,
	with the option to enable the red, green,
	or blue LEDs individually.
	"""
	def __init__(self, params, comm):
		Device.__init__(self, params)
		# create intensity sensor
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				self.intensity = nxt.sensor.Color20(comm, eval(params['port']))
				USBTransmissionSuccess = True
			except usb.core.USBError as e :
				if e.errno == 110:
					continue
				else:
					raise usb.core.USBError
		self.frame_id = params['frame_id']
		self.color_selected = "NONE"
		self.subscriber =  params['name'] + "_set_color"

		# create publisher
		self.pub = rospy.Publisher(params['name'], Color)
		rospy.loginfo("Setting up intensity sensor publisher with name %s and header frame identity %s.", params['name'],
					  params['frame_id'])

	def colorCallback(self, set_color):
		rospy.loginfo("Subscribing to %s publisher to turn on a specified color sensor light.", self.subscriber)
		if set_color.data != "":
			self.color_selected = set_color.data

	def trigger(self):
		co = Color()
		rospy.Subscriber(self.subscriber, String, self.colorCallback, None, 2)
		if self.color_selected == "RED":
			USBTransmissionSuccess = False
			while not USBTransmissionSuccess:
				try:
					self.intensity.set_light_color(Type.COLORRED)
					self.color = Type.COLORRED
					USBTransmissionSuccess = True
				except usb.core.USBError as e :
					if e.errno == 110:
						continue
					else:
						raise usb.core.USBError
		if self.color_selected == "GREEN":
			USBTransmissionSuccess = False
			while not USBTransmissionSuccess:
				try:
					self.intensity.set_light_color(Type.COLORGREEN)
					self.color = Type.COLORGREEN
					USBTransmissionSuccess = True
				except usb.core.USBError as e :
					if e.errno == 110:
						continue
					else:
						raise usb.core.USBError
		if self.color_selected == "BLUE":
			USBTransmissionSuccess = False
			while not USBTransmissionSuccess:
				try:
					self.intensity.set_light_color(Type.COLORBLUE)
					self.color = Type.COLORBLUE
					USBTransmissionSuccess = True
				except usb.core.USBError as e :
					if e.errno == 110:
						continue
					else:
						raise usb.core.USBError
		if self.color_selected == "NONE":
			USBTransmissionSuccess = False
			while not USBTransmissionSuccess:
				try:
					self.intensity.set_light_color(Type.COLORNONE)
					self.color = Type.COLORNONE
					USBTransmissionSuccess = True
				except usb.core.USBError as e :
					if e.errno == 110:
						continue
					else:
						raise usb.core.USBError
		if self.color_selected == "FULL":
			USBTransmissionSuccess = False
			while not USBTransmissionSuccess:
				try:
					self.intensity.set_light_color(Type.COLORFULL)
					self.color = Type.COLORFULL
					USBTransmissionSuccess = True
				except usb.core.USBError as e :
					if e.errno == 110:
						continue
					else:
						raise usb.core.USBError
		else:
			rospy.logerr('Invalid color specified for color sensor light')
		co.header.frame_id = self.frame_id
		co.header.stamp = rospy.Time.now()
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				co.intensity = self.intensity.get_reflected_light(self.color)
				USBTransmissionSuccess = True
			except usb.core.USBError as e :
				if e.errno == 110:
					continue
				else:
					raise usb.core.USBError
		self.pub.publish(co)


class LightSensor(Device):
	"""
	This uses the NXT1 light sensor to
	measure the intensity of ambient light,
	with the option to enable the LED to
	measure reflected light.
	"""
	def __init__(self, params, comm):
		Device.__init__(self, params)
		# Create light sensor
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				self.light = nxt.sensor.Light(comm, eval(params['port']))
				USBTransmissionSuccess = True
			except usb.core.USBError as e :
				if e.errno == 110:
					continue
				else:
					raise usb.core.USBError
		self.frame_id = params['frame_id']
		self.illuminated = False
		self.subscriber = params['name']+"_set_illuminated"

		# Create publisher
		self.pub = rospy.Publisher(params['name'], Light)
		rospy.loginfo("Setting up light sensor publisher with name %s and header frame identity %s.", params['name'],
					  params['frame_id'])

	def lightCallback(self, set_illuminated):
		self.illuminated = set_illuminated.data
		rospy.loginfo("Subscribing to %s publisher to control sensor light emission.", self.subscriber)

	def trigger(self):
		rospy.Subscriber(self.subscriber, Bool, self.lightCallback, None, 2)
		if self.illuminated:
			USBTransmissionSuccess = False
			while not USBTransmissionSuccess:
				try:
					self.light.set_illuminated(active=True)
					USBTransmissionSuccess = True
				except usb.core.USBError as e :
					if e.errno == 110:
						continue
					else:
						raise usb.core.USBError
		else:
			USBTransmissionSuccess = False
			while not USBTransmissionSuccess:
				try:
					self.light.set_illuminated(active=False)
					USBTransmissionSuccess = True
				except usb.core.USBError as e :
					if e.errno == 110:
						continue
					else:
						raise usb.core.USBError
		ls = Light()
		ls.header.frame_id = self.frame_id
		ls.header.stamp = rospy.Time.now()
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				ls.intensity = self.light.get_lightness()
				USBTransmissionSuccess = True
			except usb.core.USBError as e :
				if e.errno == 110:
					continue
				else:
					raise usb.core.USBError
		self.pub.publish(ls)


class SoundSensor(Device):
	def __init__(self, params, comm):
		Device.__init__(self, params)
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				self.sound =  nxt.sensor.Sound(comm, eval(params['port']))
				USBTransmissionSuccess = True
			except usb.core.USBError as e :
				if e.errno == 110:
					continue
				else:
					raise usb.core.USBError
		self.frame_id = params['frame_id']
		self.pub = rospy.Publisher(params['name'], Sound)
		rospy.loginfo("Setting up sound sensor publisher with name %s and header frame identity %s.", params['name'],params['frame_id'])
	def trigger(self):
		ss = Sound()
		ss.header.frame_id = self.frame_id
		ss.header.stamp = rospy.Time.now()
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				self.sound.set_adjusted(active=False)
				ss.data_db = self.sound.get_loudness()
				self.sound.set_adjusted(active=True)
				ss.data_dbA = self.sound.get_loudness()
				USBTransmissionSuccess = True
			except usb.core.USBError as e :
				if e.errno == 110:
					continue
				else:
					raise usb.core.USBError
		self.pub.publish(ss)


class RFIDSensor(Device):

	def __init__(self, params, comm):
		Device.__init__(self, params)
		# Workaround to create RFID sensor
		self.sensor_type = params['sensor_type']
		self.frame_id = params['frame_id']
		self.sensor_manufacturer = params['sensor_manufacturer']
		self.sensor_serial = params['sensor_serial']

		# Create publisher
		self.pub = rospy.Publisher(params['name'], RFID, queue_size=1)
		rospy.loginfo("Setting up RFID sensor publisher with name %s and header frame identity %s.", params['name'],
					  params['frame_id'])
		self.b = comm

	def trigger(self):
		rfid = RFID()
		rfid.header.frame_id = self.frame_id
		rfid.header.stamp = rospy.Time.now()

		# Workaround to read the RFID data using a rfid_read.rxe program stored on the brick
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				self.b.start_program('rfid_read.rxe')
				USBTransmissionSuccess = True
			except usb.core.USBError as e :
				if e.errno == 110:
					continue
				else:
					raise usb.core.USBError
		sleep(1)
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				self.read_data = nxt.brick.FileReader(self.b, "rfid_read.txt")
				USBTransmissionSuccess = True
			except usb.core.USBError as e :
				if e.errno == 110:
					continue
				else:
					raise usb.core.USBError
		self.data = self.read_data.read(bytes=None)
		self.rfid_read = self.data[0:12]
		rfid.data = self.rfid_read

		# Publish the RFID sensor specifications as defined by the user
		rfid.sensor_type = self.sensor_type
		rfid.sensor_manufacturer = self.sensor_manufacturer
		rfid.sensor_serial = self.sensor_serial
		self.pub.publish(rfid)


def main():
	# Initializing ROS node
	rospy.loginfo("Initializing nxt_ros node...")
	rospy.init_node('nxt_ros')

	# Connecting to NXT bricks
	NXT1IsConnected = False
	while not NXT1IsConnected :
		try :
			rospy.loginfo("Connecting to NXT1...")
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
			b2 = nxt.locator.find_one_brick(name="NXT2")
			NXT2IsConnected = True
			rospy.loginfo("NXT2 connected")
		except usb.core.USBError as e :
			if e.errno == 110:
				continue
			else:
				raise usb.core.USBError
	sleep(1)

	# Setting NXT1 and NXT2 motors and sensors to reset positions and values
	NXT1IsReset = False
	NXT2IsReset = False
	while not NXT1IsReset :
		try :
			rospy.loginfo("Resetting NXT1 motors and sensors...")
			b1.start_program('NXT1_calibrate.rxe')
			NXT1IsReset = True
		except usb.core.USBError as e :
			if e.errno == 110:
				continue
			else:
				raise usb.core.USBError
		sleep(1)
	while not NXT2IsReset:
		try:
			rospy.loginfo("Resetting NXT2 motors and sensors...")
			b2.start_program('NXT2_calibrate.rxe')
			sleep(15)
			b2.stop_program()
			NXT2IsReset = True
		except usb.core.USBError as e :
			if e.errno == 110:
				continue
			else:
				raise usb.core.USBError
		sleep(1)

	ns = 'nxt_components'
	config = []

	# Define exit handler
	def cleanup_node():
		print "Shutting down node"
		rospy.loginfo("Shutting down sensors connected to NXT ports...")
		for c in config:
			if c['type'] == 'color':
				# If there's a color sensor, turn off the LED light
				if c['brick'] == "NXT1":
					b = b1
				elif c['brick'] == "NXT2":
					b = b2
				else :
					rospy.logerr("Unknown NXT brick specified")
				cs = nxt.sensor.Color20(b, eval(c['port']))
				cs.set_light_color(Type.COLORNONE)
			elif c['type'] == 'intensity':
				# If there's an intensity sensor, turn off the LED light
				if c['brick'] == "NXT1":
					b = b1
				elif c['brick'] == "NXT2":
					b = b2
				else :
					rospy.logerr("Unknown NXT brick specified")
				s = nxt.sensor.Color20(b, eval(c['port']))
				s.set_light_color(Type.COLORNONE)
			elif c['type'] == 'light':
				# If there's a light sensor, turn off the LED light
				if c['brick'] == "NXT1":
					b = b1
				elif c['brick'] == "NXT2":
					b = b2
				else :
					rospy.logerr("Unknown NXT brick specified")
				ls = nxt.sensor.Light(b, eval(c['port']))
				ls.set_illuminated(active=False)
			elif c['type'] == 'ultrasonic':
				# If there's an ultrasonic sensor, turn it off
				if c['brick'] == "NXT1":
					b = b1
				elif c['brick'] == "NXT2":
					b = b2
				else :
					rospy.logerr("Unknown NXT brick specified")
				us = nxt.sensor.Ultrasonic(b, eval(c['port']))
				mode = nxt.sensor.Ultrasonic.Commands.OFF
				us.command(mode)
	rospy.on_shutdown(cleanup_node)

	config = rospy.get_param("~"+ns)
	components = []
	for c in config:
		rospy.loginfo("Connecting %s with name %s on %s %s...",c['type'],c['name'],c['brick'],c['port'])
		if c['type'] == 'motor':
			if c['brick'] == "NXT1":
				b = b1
			elif c['brick'] == "NXT2":
				b = b2
			else:
				rospy.logerr("Unknown NXT brick specified")
			components.append(Motor(c, b))
		elif c['type'] == 'touch':
			if c['brick'] == "NXT1":
				b = b1
			elif c['brick'] == "NXT2":
				b = b2
			else:
				rospy.logerr("Unknown NXT brick specified")
			components.append(TouchSensor(c, b))
		elif c['type'] == 'ultrasonic':
			if c['brick'] == "NXT1":
				b = b1
			elif c['brick'] == "NXT2":
				b = b2
			else:
				rospy.logerr("Unknown NXT brick specified")
			components.append(UltraSonicSensor(c, b))
		elif c['type'] == 'color':
			if c['brick'] == "NXT1":
				b = b1
			elif c['brick'] == "NXT2":
				b = b2
			else:
				rospy.logerr("Unknown NXT brick specified")
			components.append(ColorSensor(c, b))
		elif c['type'] == 'intensity':
			if c['brick'] == "NXT1":
				b = b1
			elif c['brick'] == "NXT2":
				b = b2
			else:
				rospy.logerr("Unknown NXT brick specified")
			components.append(IntensitySensor(c, b))
		elif c['type'] == 'light':
			if c['brick'] == "NXT1":
				b = b1
			elif c['brick'] == "NXT2":
				b = b2
			else:
				rospy.logerr("Unknown NXT brick specified")
			components.append(LightSensor(c, b))
		elif c['type'] == 'gyro':
			if c['brick'] == "NXT1":
				b = b1
			elif c['brick'] == "NXT2":
				b = b2
			else:
				rospy.logerr("Unknown NXT brick specified")
			components.append(GyroSensor(c, b))
		elif c['type'] == 'accelerometer':
			if c['brick'] == "NXT1":
				b = b1
			elif c['brick'] == "NXT2":
				b = b2
			else:
				rospy.logerr("Unknown NXT brick specified")
			components.append(AccelerometerSensor(c, b))
		elif c['type'] == 'rfid':
			if c['brick'] == "NXT1":
				b = b1
			elif c['brick'] == "NXT2":
				b = b2
			else:
				rospy.logerr("Unknown NXT brick specified")
			components.append(RFIDSensor(c, b))
		elif c['type'] == 'sound':
			if c['brick'] == "NXT1":
				b = b1
			elif c['brick'] == "NXT2":
				b = b2
			else:
				rospy.logerr("Unknown NXT brick specified")
			components.append(SoundSensor(c, b))
		else:
			rospy.logerr('Invalid sensor/actuator type %s.'%c['type'])

	callback_handle_frequency = 100.0
	last_callback_handle = rospy.Time.now()
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


if __name__ == '__main__':
	main()