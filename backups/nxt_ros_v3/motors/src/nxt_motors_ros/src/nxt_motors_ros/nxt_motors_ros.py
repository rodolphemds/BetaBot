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

import roslib; roslib.load_manifest('nxt_motors_ros')
import nxt.locator
import rospy
import math
from nxt.motor import Motor, SynchronizedMotors, PORT_A, PORT_B, PORT_C
from nxt.brick import Brick
from nxt.locator import find_one_brick
import nxt.motor
import thread
import subprocess
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Bool, String
from nxt_motors_msgs.msg import JointCommand, JointSpecificPosition
from PyKDL import Rotation
from time import sleep

global my_lock
my_lock = thread.allocate_lock()


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


class TorsoMotor(Device):
	def __init__(self, params_torso_motor, comm):
		Device.__init__(self, params_torso_motor)
		self.name = torso_joint
		self.power_to_nm = 0.01
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
		rospy.loginfo("Setting up unspecific motor publisher with name joint_state.")

		# create publishers for position name of torso joint
		self.specificPub = rospy.Publisher('torso_joint_position_name',JointSpecificPosition, queue_size=1)
		rospy.loginfo("Setting up motor specific position name publisher with name torso_joint_position_name")
		# create subscriber
		self.sub = rospy.Subscriber('joint_command', JointCommand, self.cmd_cb, None, 2)

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
		js.effort.append(self.cmd * self.power_to_nm)  # this is just the commanded effort
		vel = 0
		if self.last_js:
			vel = (js.position[0] - self.last_js.position[0]) / (js.header.stamp - self.last_js.header.stamp).to_sec()
			js.velocity.append(vel)
		else:
			vel = 0
			js.velocity.append(vel)
		self.pub.publish(js)
		self.last_js = js

		# Send specific position name
		self.position_name = JointSpecificPosition()
		self.position_name.header.stamp = rospy.Time.now()
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				self.tacho_count = torso_motor.get_tacho().rotation_count
				USBTransmissionSuccess = True
			except usb.core.USBError as e:
				if e.errno == 110:
					continue
				else:
					raise usb.core.USBError
			if self.name == "torso_joint":
				if self.tacho_count == 0:
					self.position_name.position_name = "torso_up"
				elif self.tacho_count == 950:
					self.position_name.position_name = "torso_down"
				elif self.tacho_count == 450:
					self.position_name.position_name = "torso_middle"
				elif self.tacho_count == 200:
					self.position_name.position_name = "torso_middle_up"
				elif self.tacho_count == 700:
					self.position_name.position_name = "torso_middle_down"

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
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				torso_joint.run(int(self.cmd), 0)
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
					torso_joint.brake()
					USBTransmissionSuccess = True
				except usb.core.USBError as e:
					if e.errno == 110:
						continue
					else:
						raise usb.core.USBError


class HeadMotor(Device):
	def __init__(self, params_head_motor, comm):
		Device.__init__(self, params_head_motor)
		self.name = head_joint
		self.power_to_nm = 0.01
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
		rospy.loginfo("Setting up unspecific motor publisher with name joint_state.")

		# create publishers for position name of head joint
		self.specificPub = rospy.Publisher('head_joint_position_name', JointSpecificPosition, queue_size=1)
		rospy.loginfo("Setting up motor specific position name publisher with name head_joint_position_name")
		# create subscriber
		self.sub = rospy.Subscriber('joint_command', JointCommand, self.cmd_cb, None, 2)

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
		js.effort.append(self.cmd * self.power_to_nm)  # this is just the commanded effort
		vel = 0
		if self.last_js:
			vel = (js.position[0] - self.last_js.position[0]) / (js.header.stamp - self.last_js.header.stamp).to_sec()
			js.velocity.append(vel)
		else:
			vel = 0
			js.velocity.append(vel)
		self.pub.publish(js)
		self.last_js = js

		# Send specific position name of head joint
		if self.name == "head_joint":
			self.position_name = JointSpecificPosition()
			self.position_name.header.stamp = rospy.Time.now()
			USBTransmissionSuccess = False
			while not USBTransmissionSuccess:
				try:
					self.tacho_count = head_motor.get_tacho().rotation_count
					USBTransmissionSuccess = True
				except usb.core.USBError as e:
					if e.errno == 110:
						continue
					else:
						raise usb.core.USBError
			if self.name == "head_joint":
				if self.tacho_count == -410:
					self.position_name.position_name = "head_max_left"
				elif self.tacho_count == 410:
					self.position_name.position_name = "head_max_right"
				elif self.tacho_count == 0:
					self.position_name.position_name = "head_center"
			self.specificPub.publish(self.position_name)

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
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				head_joint.run(int(self.cmd), 0)
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
					head_joint.brake()
					USBTransmissionSuccess = True
				except usb.core.USBError as e:
					if e.errno == 110:
						continue
					else:
						raise usb.core.USBError


class LaserMotor(Device):
	def __init__(self, params_laser_motor, comm):
		Device.__init__(self, params_laser_motor)
		self.name = laser_joint
		self.power_to_nm = 0.01
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
		rospy.loginfo("Setting up unspecific motor publisher with name joint_state.")

		# create publishers for position name of laser joint
		if self.name == "laser_joint" :
			self.specificPub = rospy.Publisher('laser_joint_position_name', JointSpecificPosition, queue_size=1)
			rospy.loginfo("Setting up motor specific position name publisher with name laser_joint_position_name")
		# create subscriber
		self.sub = rospy.Subscriber('joint_command', JointCommand, self.cmd_cb, None, 2)

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
		js.laserer.stamp = rospy.Time.now()
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
		js.effort.append(self.cmd * self.power_to_nm)  # this is just the commanded effort
		vel = 0
		if self.last_js:
			vel = (js.position[0] - self.last_js.position[0]) / (js.laserer.stamp - self.last_js.laserer.stamp).to_sec()
			js.velocity.append(vel)
		else:
			vel = 0
			js.velocity.append(vel)
		self.pub.publish(js)
		self.last_js = js

		# Send specific position name of laser, laser, laser and laser joints
		if self.name == "laser_joint" :
			self.position_name = JointSpecificPosition()
			self.position_name.laserer.stamp = rospy.Time.now()
			USBTransmissionSuccess = False
			while not USBTransmissionSuccess:
				try:
					self.tacho_count = laser_motor.get_tacho().rotation_count
					USBTransmissionSuccess = True
				except usb.core.USBError as e:
					if e.errno == 110:
						continue
					else:
						raise usb.core.USBError
			if self.name == "laser_joint":
				if self.tacho_count == 95:
					self.position_name.position_name = "laser_up"
				elif self.tacho_count == 0:
					self.position_name.position_name = "laser_down"
			self.specificPub.publish(self.position_name)

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
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				laser_joint.run(int(self.cmd), 0)
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
					laser_joint.brake()
					USBTransmissionSuccess = True
				except usb.core.USBError as e:
					if e.errno == 110:
						continue
					else:
						raise usb.core.USBError


class ArmsMotor(Device):
	def __init__(self, params_arms_motor, comm):
		Device.__init__(self, params_arms_motor)
		self.name = arms_joint
		self.power_to_nm = 0.01
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
		rospy.loginfo("Setting up unspecific motor publisher with name joint_state.")

		# create publishers for position name of arms, arms, laser and arms joints
		if self.name == "arms_joint" :
			self.specificPub = rospy.Publisher('arms_joint_position_name', JointSpecificPosition, queue_size=1)
			rospy.loginfo("Setting up motor specific position name publisher with name arms_joint_position_name")
		# create subscriber
		self.sub = rospy.Subscriber('joint_command', JointCommand, self.cmd_cb, None, 2)

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
		js.armser.stamp = rospy.Time.now()
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
		js.effort.append(self.cmd * self.power_to_nm)  # this is just the commanded effort
		vel = 0
		if self.last_js:
			vel = (js.position[0] - self.last_js.position[0]) / (js.armser.stamp - self.last_js.armser.stamp).to_sec()
			js.velocity.append(vel)
		else:
			vel = 0
			js.velocity.append(vel)
		self.pub.publish(js)
		self.last_js = js

		# Send specific position name of arms, arms, laser and arms joints
		if self.name == "arms_joint" :
			self.position_name = JointSpecificPosition()
			self.position_name.armser.stamp = rospy.Time.now()
			USBTransmissionSuccess = False
			while not USBTransmissionSuccess:
				try:
					self.tacho_count = arms_motor.get_tacho().rotation_count
					USBTransmissionSuccess = True
				except usb.core.USBError as e:
					if e.errno == 110:
						continue
					else:
						raise usb.core.USBError
			if self.name == "arms_joint":
				if self.tacho_count == 250:
					self.position_name.position_name = "forearms_up"
				elif self.tacho_count == 180:
					self.position_name.position_name = "hands_closed"
				elif self.tacho_count == 0:
					self.position_name.position_name = "hands_open"
				elif self.tacho_count == -140:
					self.position_name.position_name = "wrist_rotated"
			self.specificPub.publish(self.position_name)

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
		USBTransmissionSuccess = False
		while not USBTransmissionSuccess:
			try:
				arms_joint.run(int(self.cmd), 0)
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
					arms_joint.brake()
					USBTransmissionSuccess = True
				except usb.core.USBError as e:
					if e.errno == 110:
						continue
					else:
						raise usb.core.USBError


class LeftTreadMotor(Device):
	def __init__(self, params_left_tread_motor, comm):
		Device.__init__(self, params_left_tread_motor)
		self.name = left_tread_motor
		USBTransmissionSuccess = False
		self.power_to_nm = 0.01
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
					left_tread_joint.reset_position(False)
					USBTransmissionSuccess = True
				except usb.core.USBError as e:
					if e.errno == 110:
						continue
					else:
						raise usb.core.USBError

		# create general publisher
		self.pub = rospy.Publisher('joint_state', JointState, queue_size=10)
		self.last_js = None
		rospy.loginfo("Setting up unspecific motor publisher with name joint_state.")

		# create subscriber
		self.sub = rospy.Subscriber('joint_command', JointCommand, self.cmd_cb, None, 2)

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
				rotation_count = left_tread_joint.get_tacho().rotation_count
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
		js.effort.append(self.cmd * self.power_to_nm)  # this is just the commanded effort
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
						left_tread_joint.reset_position(False)
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
				left_tread_joint.run(int(self.cmd), 0)
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
					left_tread_joint.brake()
					USBTransmissionSuccess = True
				except usb.core.USBError as e:
					if e.errno == 110:
						continue
					else:
						raise usb.core.USBError


class RightTreadMotor(Device):
	def __init__(self, params_right_tread_motor, comm):
		Device.__init__(self, params_right_tread_motor)
		self.name = right_tread_joint
		USBTransmissionSuccess = False
		self.power_to_nm = 0.01
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
					right_tread_joint.reset_position(False)
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

		# create subscriber
		self.sub = rospy.Subscriber('joint_command', JointCommand, self.cmd_cb, None, 2)

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
				rotation_count = right_tread_joint.get_tacho().rotation_count
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

		if self.use_absolute_position:
			# If motor has done a full rotation, reset the encoder to zero
			if (rotation_count >= self.counts_per_rev) \
			or (rotation_count <= -1.0*self.counts_per_rev):
				USBTransmissionSuccess = False
				while not USBTransmissionSuccess:
					try:
						right_tread_joint.reset_position(False)
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
				right_tread_joint.run(int(self.cmd), 0)
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
					right_tread_joint.brake()
					USBTransmissionSuccess = True
				except usb.core.USBError as e:
					if e.errno == 110:
						continue
					else:
						raise usb.core.USBError

				
def main():
	# Initializing ROS node
	rospy.loginfo("Initializing nxt_motors_ros node...")
	rospy.init_node('nxt_motors_ros')
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

	# Setting NXT1 and NXT2 motors and sensors to reset positions and values
	NXT1IsReset = False
	NXT2IsReset = False
	while not NXT1IsReset :
		try :
			rospy.loginfo("Resetting NXT1...")
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
			rospy.loginfo("Resetting NXT2...")
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

	components = []
	global right_tread_joint
	right_tread_joint = Motor(b1, PORT_A)
	rospy.loginfo("Connecting to right_tread_joint on NXT1_PORT_A")
	params_right_tread_joint = {'type': 'motor', 'name': 'right_tread_joint', 'port': 'PORT_A', 'brick': 'NXT1', 'desired_frequency': 30}
	components.append(RightTreadMotor(params_right_tread_motor, b1))
	global torso_joint
	torso_joint = Motor(b1, PORT_B)
	rospy.loginfo("Connecting to torso_joint on NXT1_PORT_B")
	params_torso_joint = {'type': 'motor', 'name': 'torso_joint', 'port': 'PORT_B', 'brick': 'NXT1', 'desired_frequency': 1}
	components.append(TorsoMotor(params_torso_motor, b1))
	global left_tread_joint
	left_tread_joint = Motor(b1, PORT_C)
	rospy.loginfo("Connecting to left_tread_joint on NXT1_PORT_C")
	params_left_tread_joint = {'type': 'motor', 'name': 'left_tread_joint', 'port': 'PORT_C', 'brick': 'NXT1', 'desired_frequency': 30}
	components.append(LeftTreadMotor(params_left_tread_motor, b1))
	global head_joint
	head_joint = Motor(b2, PORT_A)
	rospy.loginfo("Connecting to head_joint on NX2_PORT_A")
	params_head_joint = {'type': 'motor', 'name': 'head_joint', 'port': 'PORT_A', 'brick': 'NXT2', 'desired_frequency': 5}
	components.append(HeadMotor(params_head_motor, b2))
	global laser_joint
	laser_joint = Motor(b2, PORT_B)
	rospy.loginfo("Connecting to laser_joint on NXT2_PORT_B")
	params_laser_joint = {'type': 'motor', 'name': 'laser_joint', 'port': 'PORT_B', 'brick': 'NXT2', 'desired_frequency': 1}
	components.append(LaserMotor(params_laser_motor, b2))
	global arms_joint
	arms_joint = Motor(b2, PORT_C)
	rospy.loginfo("Connecting to arms_joint on NXT2_PORT_C")
	params_arms_joint = {'type': 'motor', 'name': 'arms_joint', 'port': 'PORT_C', 'brick': 'NXT2', 'desired_frequency': 5}
	components.append(ArmsMotor(params_arms_motor, b2))

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
		rospy.loginfo("Shutting down motors...")
		right_tread_joint.run(0, 0)
		left_tread_joint.run(0, 0)
		torso_joint.run(0, 0)
		head_joint.run(0, 0)
		arms_joint.run(0, 0)
		laser_joint.run(0, 0)
	rospy.on_shutdown(cleanup_node)

if __name__ == '__main__':
	main()