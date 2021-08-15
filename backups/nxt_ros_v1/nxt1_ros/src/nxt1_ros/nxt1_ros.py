#!/usr/bin/env python

import roslib; roslib.load_manifest('nxt1_ros')
import nxt.locator
import rospy
import math
from nxt.motor import Motor, SynchronizedMotors, PORT_A, PORT_B, PORT_C
from nxt.sensor import PORT_1, PORT_2, PORT_3, PORT_4
from nxt.sensor import *
from nxt.brick import Brick
from nxt.locator import find_one_brick
import nxt.motor
import thread
import subprocess
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Bool
from nxt_msgs.msg import ArmState, ArmCommand, Contact, JointCommand, Color, Light, RFID, Sound, HeadState, HeadCommand, LaserState, LaserCommand, TorsoState, TorsoCommand
from PyKDL import Rotation
from time import sleep

power_to_nm = 0.01
power_max = 125
torso_lowest_position = 0
torso_highest_position = 950
torso_up = torso_lowest_position
torso_middle_up = 200
torso_middle = 450
torso_middle_down = 700
torso_down = torso_highest_position

global right_tread_motor_command_tacho_units
global right_tread_motor_command_timeout
global right_tread_motor_command_adjusted_power
global right_tread_motor_command_brake
global left_tread_motor_command_tacho_units
global left_tread_motor_command_timeout
global left_tread_motor_command_adjusted_power
global left_tread_motor_command_brake
global torso_motor_command_tacho_units
global torso_motor_command_timeout
global torso_motor_command_adjusted_power
global torso_motor_command_brake
global torso_motor_command_status_units
global torso_motor_command_status_name
global my_lock
right_tread_motor_command_adjusted_power = 0
right_tread_motor_command_brake = False
right_tread_motor_command_tacho_units = 0
right_tread_motor_command_timeout = 0
left_tread_motor_command_adjusted_power = 0
left_tread_motor_command_brake = False
left_tread_motor_command_tacho_units = 0
left_tread_motor_command_timeout = 0
torso_motor_command_adjusted_power = 0
torso_motor_command_brake = False
torso_motor_command_tacho_units = 0
torso_motor_command_timeout = 0
torso_motor_command_status_name = ""
torso_motor_command_status_units = 99999999

my_lock = thread.allocate_lock()

class Device :
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
            rospy.logwarn("%s not reaching desired frequency: actual %f, desired %f"%(self.name, 1.0/period, 1.0/self.desired_period))
        elif period > self.desired_period * 1.5:
            rospy.logerr("%s not reaching desired frequency: actual %f, desired %f"%(self.name, 1.0/period, 1.0/self.desired_period))
        return period > self.desired_period

    def do_trigger(self):
        try:
          rospy.logdebug('Trigger %s with current frequency %f'%(self.name, 1.0/self.period))
          now = rospy.Time.now()
          self.period = 0.9 * self.period + 0.1 * (now - self.last_run).to_sec()
          self.last_run = now
          self.trigger()
          rospy.logdebug('Trigger %s took %f mili-seconds'%(self.name, (rospy.Time.now() - now).to_sec()*1000))
        except nxt.error.DirProtError:
          rospy.logwarn("caught an exception nxt.error.DirProtError")
          pass
        except nxt.error.I2CError:
          rospy.logwarn("caught an exception nxt.error.I2CError")
          pass

class RightTreadMotor(Device):
    def __init__(self, params_right_tread_motor, comm):
        Device.__init__(self, params_right_tread_motor)
        self.last_right_tread_motor_state = None
    def right_tread_motor_command_callback(self, right_tread_motor_command):
        right_tread_motor_command_adjusted_power = 0
        right_tread_motor_command_brake = False
        right_tread_motor_command_tacho_units = 0
        right_tread_motor_command_timeout = 0
        if right_tread_motor_command.name == "right_tread_motor_command" :
            right_tread_motor_command_adjusted_power = (right_tread_motor_command.effort / power_to_nm)
            right_tread_motor_command_brake = right_tread_motor_command.brake
            right_tread_motor_command_tacho_units = right_tread_motor_command.tacho_units
            right_tread_motor_command_timeout = right_tread_motor_command.timeout
            if right_tread_motor_command_adjusted_power > power_max:
                right_tread_motor_command_adjusted_power = power_max
            elif right_tread_motor_command_adjusted_power < - power_max:
                right_tread_motor_command_adjusted_power = -power_max
            self.last_right_tread_motor_command = right_tread_motor_command  # save command
    def trigger(self):
        right_tread_motor_command_subscriber = rospy.Subscriber("right_tread_motor_command", JointCommand,
                                                                self.right_tread_motor_command_callback, None, 2)
        right_tread_motor_state = JointState()
        right_tread_motor_state.header.stamp = rospy.Time.now()
        right_tread_motor_state.name.append("right_tread_motor")
        (output_state, tacho_state) = right_tread_motor._read_state()
        power = output_state.power
        mode = output_state.mode
        regulation = output_state.regulation
        turn_ratio = output_state.turn_ratio
        run_state = output_state.run_state
        tacho_limit = output_state.tacho_limit
        tacho_count = tacho_state.tacho_count
        block_tacho_count = tacho_state.block_tacho_count
        rotation_count = tacho_state.rotation_count
        right_tread_motor_state.position.append(rotation_count * math.pi / 180.0)
        right_tread_motor_state.effort.append(power * power_to_nm)
        velocity = 0
        if self.last_right_tread_motor_state:
            velocity = (right_tread_motor_state.position[0] - self.last_right_tread_motor_state.position[0]) / (right_tread_motor_state.header.stamp - self.last_right_tread_motor_state.header.stamp).to_sec()
        else:
            velocity = 0
            right_tread_motor_state.velocity.append(velocity)
        right_tread_motor_state_publisher.publish(right_tread_motor_state)
        self.last_right_tread_motor_state = right_tread_motor_state

        if right_tread_motor_command_tacho_units !=0 and right_tread_motor_command_timeout ==0 :
            right_tread_motor.turn(power= -right_tread_motor_command_adjusted_power, tacho_units=right_tread_motor_command_tacho_units, brake=right_tread_motor_command_brake)
        elif right_tread_motor_command_timeout !=0  and right_tread_motor_command_tacho_units ==0 :
            right_tread_motor.turn(power= -right_tread_motor_command_adjusted_power, brake=right_tread_motor_command_brake, timeout=right_tread_motor_command_timeout)
        elif right_tread_motor_command_timeout ==0 and right_tread_motor_command_tacho_units ==0:
             right_tread_motor.run(-right_tread_motor_command_adjusted_power, 0)

class LeftTreadMotor(Device):
    def __init__(self, params_left_tread_motor, comm):
        Device.__init__(self, params_left_tread_motor)
        self.last_left_tread_motor_state = None
    def left_tread_motor_command_callback(self, left_tread_motor_command):
        left_tread_motor_command_adjusted_power = 0
        left_tread_motor_command_brake = False
        left_tread_motor_command_tacho_units = 0
        left_tread_motor_command_timeout = 0
        if left_tread_motor_command.name == "left_tread_motor_command":
            left_tread_motor_command_adjusted_power = left_tread_motor_command.effort / power_to_nm
            left_tread_motor_command_brake = left_tread_motor_command.brake
            left_tread_motor_command_tacho_units = left_tread_motor_command.tacho_units
            left_tread_motor_command_timeout = left_tread_motor_command.timeout
            if left_tread_motor_command_adjusted_power > power_max:
                left_tread_motor_command_adjusted_power = power_max
            elif left_tread_motor_command_adjusted_power < -power_max:
                left_tread_motor_command_adjusted_power = -power_max
            self.last_left_tread_motor_command = left_tread_motor_command  # save command
    def trigger(self):
        left_tread_motor_command_subscriber = rospy.Subscriber("left_tread_motor_command", JointCommand,
                                                               self.left_tread_motor_command_callback, None, 2)
        left_tread_motor_state = JointState()
        left_tread_motor_state.header.stamp = rospy.Time.now()
        left_tread_motor_state.name.append("left_tread_motor")
        (output_state, tacho_state) = left_tread_motor._read_state()
        power = output_state.power
        mode = output_state.mode
        regulation = output_state.regulation
        turn_ratio = output_state.turn_ratio
        run_state = output_state.run_state
        tacho_limit = output_state.tacho_limit
        tacho_count = tacho_state.tacho_count
        block_tacho_count = tacho_state.block_tacho_count
        rotation_count = tacho_state.rotation_count
        left_tread_motor_state.position.append(rotation_count * math.pi / 180.0)
        left_tread_motor_state.effort.append(power * power_to_nm)
        velocity = 0
        if self.last_left_tread_motor_state:
            velocity = (left_tread_motor_state.position[0] - self.last_left_tread_motor_state.position[0]) / (
                    left_tread_motor_state.header.stamp - self.last_left_tread_motor_state.header.stamp).to_sec()
        else:
            velocity = 0
            left_tread_motor_state.velocity.append(velocity)
        left_tread_motor_state_publisher.publish(left_tread_motor_state)
        self.last_left_tread_motor_state = left_tread_motor_state

        if left_tread_motor_command_tacho_units != 0 and not left_tread_motor_command_timeout ==0 :
            left_tread_motor.turn(power=-left_tread_motor_command_adjusted_power,
                                  tacho_units=left_tread_motor_command_tacho_units,
                                  brake=left_tread_motor_command_brake)
        elif left_tread_motor_command_timeout != 0 and not left_tread_motor_command_tacho_units ==0:
            left_tread_motor.turn(power=-left_tread_motor_command_adjusted_power, brake=left_tread_motor_command_brake,
                                  timeout=left_tread_motor_command_timeout)
        elif left_tread_motor_command_timeout ==0  and not left_tread_motor_command_tacho_units ==0:
            left_tread_motor.run(-left_tread_motor_command_adjusted_power, 0)

class TorsoMotor(Device):
    def __init__(self, params_torso_motor, comm):
        Device.__init__(self, params_torso_motor)
        self.last_torso_motor_state = None
    def torso_motor_command_callback(self, torso_motor_command):
        torso_motor_command_adjusted_power = 0
        torso_motor_command_brake = False
        torso_motor_command_tacho_units = 0
        torso_motor_command_timeout = 0
        torso_motor_command_status_name = ""
        torso_motor_command_status_units = 99999999
        if torso_motor_command.name == "torso_motor_command":
            torso_motor_command_adjusted_power = torso_motor_command.effort / power_to_nm
            torso_motor_command_brake = torso_motor_command.brake
            torso_motor_command_tacho_units = torso_motor_command.tacho_units
            torso_motor_command_timeout = torso_motor_command.timeout
            torso_motor_command_status_name = torso_motor_command.status_name
            torso_motor_command_status_units = torso_motor_command.status_units
            if torso_motor_command_adjusted_power > power_max:
                torso_motor_command_adjusted_power = power_max
            elif torso_motor_command_adjusted_power < -power_max:
                torso_motor_command_adjusted_power = -power_max
            self.last_torso_motor_command = torso_motor_command  # save command
            if torso_motor_command_status_name != "":
                if torso_motor_command_status_name == "torso_up":
                    torso_motor_command_status_units = torso_up
                if torso_motor_command_status_name == "torso_down":
                    torso_motor_command_status_units = torso_down
                if torso_motor_command_status_name == "torso_middle_up":
                    torso_motor_command_status_units = torso_middle_up
                if torso_motor_command_status_name == "torso_middle_down":
                    torso_motor_command_status_units = torso_middle_down
                if torso_motor_command_status_name == "torso_middle":
                    torso_motor_command_status_units = torso_middle

    def trigger(self):
        torso_motor_command_subscriber = rospy.Subscriber("torso_motor_command", TorsoCommand,
                                                          self.torso_motor_command_callback, None, 2)
        torso_motor_state = JointState()
        torso_motor_state_details = TorsoState()
        torso_motor_state.header.stamp = rospy.Time.now()
        torso_motor_state.name.append("torso_motor")
        (output_state, tacho_state) = torso_motor._read_state()
        power = output_state.power
        mode = output_state.mode
        regulation = output_state.regulation
        turn_ratio = output_state.turn_ratio
        run_state = output_state.run_state
        tacho_limit = output_state.tacho_limit
        tacho_count = tacho_state.tacho_count
        block_tacho_count = tacho_state.block_tacho_count
        rotation_count = tacho_state.rotation_count
        torso_motor_state.position.append(rotation_count * math.pi / 180.0)
        torso_motor_state.effort.append(power * power_to_nm)
        velocity = 0
        if self.last_torso_motor_state:
            velocity = (torso_motor_state.position[0] - self.last_torso_motor_state.position[0]) / (
                    torso_motor_state.header.stamp - self.last_torso_motor_state.header.stamp).to_sec()
        else:
            velocity = 0
            torso_motor_state.velocity.append(velocity)
        torso_motor_state_publisher.publish(torso_motor_state)
        self.last_torso_motor_state = torso_motor_state

        if torso_motor_command_status_units != 99999999 :
            distance_to_status_units = torso_motor_command_status_units - rotation_count
            if distance_to_status_units > 0:
                torso_motor.turn(power=torso_motor_command_adjusted_power, tacho_units=distance_to_status_units,
                                 brake=torso_motor_command_brake)
            if distance_to_status_units < 0:
                distance_to_status_units = abs(distance_to_status_units)
                torso_motor.turn(power= - torso_motor_command_adjusted_power, tacho_units=distance_to_status_units,
                                 brake=torso_motor_command_brake)
            elif distance_to_status_units == 0:
                rospy.logwarn("Torso already at the specified position")

        if rotation_count <= torso_up and torso_motor_command_adjusted_power < 0:
            rospy.logwarn("Torso already at the lowest position")
        elif rotation_count >= torso_down and torso_motor_command_adjusted_power > 0:
            rospy.logwarn("Torso already at the highest position")
        elif (rotation_count < torso_up and torso_motor_command_adjusted_power >= 0) or (rotation_count > torso_down and torso_motor_command_adjusted_power <= 0):
            if torso_motor_command_tacho_units != 0 and torso_motor_command_timeout ==0:
                torso_motor.turn(power=torso_motor_command_adjusted_power,
                             tacho_units=torso_motor_command_tacho_units, brake=torso_motor_command_brake)
            elif torso_motor_command_timeout !=0 and torso_motor_command_tacho_units ==0:
                torso_motor.turn(power=torso_motor_command_adjusted_power, brake=torso_motor_command_brake,
                             timeout=torso_motor_command_timeout)
            elif torso_motor_command_timeout ==0 and torso_motor_command_tacho_units ==0:
                torso_motor.run(torso_motor_command_adjusted_power, 0)
        (output_state, tacho_state) = torso_motor._read_state()
        power = output_state.power
        mode = output_state.mode
        regulation = output_state.regulation
        turn_ratio = output_state.turn_ratio
        run_state = output_state.run_state
        tacho_limit = output_state.tacho_limit
        tacho_count = tacho_state.tacho_count
        block_tacho_count = tacho_state.block_tacho_count
        rotation_count = tacho_state.rotation_count
        if rotation_count == torso_up:
            torso_motor_state_details.status.append("torso_up")
        if rotation_count == torso_middle_up:
            torso_motor_state_details.status.append("torso_middle_up")
        if rotation_count == torso_middle:
            torso_motor_state_details.status.append("torso_middle")
        if rotation_count == torso_middle_down:
            torso_motor_state_details.status.append("torso_middle_down")
        if rotation_count == torso_down:
            torso_motor_state_details.status.append("torso_down")
        torso_motor_state_details_publisher.publish(torso_motor_state_details)

class RightTouchSensor(Device):
    def __init__(self, params_right_bumper, comm):
        Device.__init__(self, params_right_bumper)
    def trigger(self):
        right_bumper_report = Contact()
        right_bumper_report.header.frame_id="right_bumper"
        right_bumper_report.header.stamp = rospy.Time.now()
        right_bumper_report.contact = right_bumper.get_sample()
        right_bumper_publisher.publish(right_bumper_report)

class LeftTouchSensor(Device):
    def __init__(self, params_left_bumper, comm):
        Device.__init__(self, params_left_bumper)
    def trigger(self):
        left_bumper_report = Contact()
        left_bumper_report.header.frame_id="left_bumper"
        left_bumper_report.header.stamp = rospy.Time.now()
        left_bumper_report.contact = left_bumper.get_sample()
        left_bumper_publisher.publish(left_bumper_report)

class BackTouchSensor(Device):
    def __init__(self, params_back_bumper, comm):
        Device.__init__(self, params_back_bumper)
    def trigger(self):
        back_bumper_report = Contact()
        back_bumper_report.header.frame_id="back_bumper"
        back_bumper_report.header.stamp = rospy.Time.now()
        back_bumper_report.contact = back_bumper.get_sample()
        back_bumper_publisher.publish(back_bumper_report)

class UltrasonicSensor(Device):
    def __init__(self, params_ultrasonic_sensor, comm):
        Device.__init__(self, params_ultrasonic_sensor)
    def trigger(self):
        ultrasonic_sensor_report = Range()
        ultrasonic_sensor_report.header.frame_id = "ultrasonic_sensor"
        ultrasonic_sensor_report.header.stamp = rospy.Time.now()
        ultrasonic_sensor_report.radiation_type = 0
        ultrasonic_sensor_report.field_of_view = 0.5235987756 # rad
        ultrasonic_sensor_report.min_range = 0.07
        ultrasonic_sensor_report.max_range = 2.54
        ultrasonic_sensor_report.range = ultrasonic_sensor.get_sample() / 100.0
        ultrasonic_sensor_publisher.publish(ultrasonic_sensor_report)

# Publishers definition
right_tread_motor_state_publisher = rospy.Publisher("right_tread_motor_state", JointState, queue_size=1)
rospy.loginfo("Setup publisher on right_tread_motor_state [sensor_msgs/JointState]")
left_tread_motor_state_publisher = rospy.Publisher("left_tread_motor_state", JointState, queue_size=1)
rospy.loginfo("Setup publisher on left_tread_motor_state [sensor_msgs/JointState]")
torso_motor_state_publisher = rospy.Publisher("torso_motor_state", JointState, queue_size=1)
rospy.loginfo("Setup publisher on torso_motor_state [sensor_msgs/JointState]")
torso_motor_state_details_publisher = rospy.Publisher("torso_motor_state_details", TorsoState, queue_size=1)
rospy.loginfo("Setup publisher on torso_motor_state_details [nxt_msgs/TorsoState]")
right_bumper_publisher = rospy.Publisher("right_bumper", Contact, queue_size=1)
rospy.loginfo("Setup publisher on right_bumper [nxt_msgs/Contact]")
left_bumper_publisher = rospy.Publisher("left_bumper", Contact, queue_size=1)
rospy.loginfo("Setup publisher on left_bumper [nxt_msgs/Contact]")
back_bumper_publisher = rospy.Publisher("back_bumper", Contact, queue_size=1)
rospy.loginfo("Setup publisher on back_bumper [nxt_msgs/Contact]")
ultrasonic_sensor_publisher = rospy.Publisher("ultrasonic_sensor", Range, queue_size=1)
rospy.loginfo("Setup publisher on ultrasonic_sensor [nxt_msgs/Range]")

def main():
    global brick
    brick = nxt.locator.find_one_brick(name="NXT1")
    rospy.logdebug("Connected to NXT1")
    rospy.init_node("nxt1_ros")
    callback_handle_frequency = 10.0
    last_callback_handle = rospy.Time.now()

    components = []
    global right_tread_motor
    right_tread_motor = Motor(brick, PORT_A)
    rospy.loginfo("Connecting to right_tread_motor on NXT1_PORT_A")
    params_right_tread_motor = {'type': 'Motor', 'name': 'right_tread_motor', 'port': 'PORT_A', 'desired_frequency': 30}
    components.append(RightTreadMotor(params_right_tread_motor, brick))
    global torso_motor
    torso_motor = Motor(brick, PORT_B)
    rospy.loginfo("Connecting to torso_motor on NXT1_PORT_B")
    params_torso_motor = {'type': 'Motor', 'name': 'torso_motor', 'port': 'PORT_B', 'desired_frequency': 1}
    components.append(TorsoMotor(params_torso_motor, brick))
    global left_tread_motor
    left_tread_motor = Motor(brick, PORT_C)
    rospy.loginfo("Connecting to left_tread_motor on NXT1_PORT_C")
    params_left_tread_motor = {'type': 'Motor', 'name': 'left_tread_motor', 'port': 'PORT_C', 'desired_frequency': 30}
    components.append(LeftTreadMotor(params_left_tread_motor, brick))
    global right_bumper
    right_bumper = Touch(brick, PORT_1)
    rospy.loginfo("Connecting to right_bumper on NXT1_PORT_1")
    params_right_bumper = {'type': 'Touch', 'name': 'right_bumper', 'port': 'PORT_1', 'desired_frequency': 2}
    components.append(RightTouchSensor(params_right_bumper, brick))
    global left_bumper
    left_bumper = Touch(brick, PORT_2)
    rospy.loginfo("Connecting to left_bumper on NXT1_PORT_2")
    params_left_bumper = {'type': 'Touch', 'name': 'left_bumper', 'port': 'PORT_2', 'desired_frequency': 2}
    components.append(LeftTouchSensor(params_left_bumper, brick))
    global back_bumper
    back_bumper = Touch(brick, PORT_3)
    rospy.loginfo("Connecting to back_bumper on NXT1_PORT_3")
    params_back_bumper = {'type': 'Touch', 'name': 'back_bumper', 'port': 'PORT_3', 'desired_frequency': 2}
    components.append(BackTouchSensor(params_back_bumper, brick))
    global ultrasonic_sensor
    ultrasonic_sensor = Ultrasonic(brick, PORT_4)
    rospy.loginfo("Connecting to ultrasonic_sensor on NXT1_PORT_4")
    params_ultrasonic_sensor = {'type': 'Ultrasonic', 'name': 'ultrasonic_sensor', 'port': 'PORT_4', 'desired_frequency': 3}
    components.append(UltrasonicSensor(params_ultrasonic_sensor, brick))

    while not rospy.is_shutdown():
        my_lock.acquire()
        triggered = False
        for c in components:
            if c.needs_trigger() and not triggered:
                c.do_trigger()
                triggered = True
        my_lock.release()
        now = rospy.Time.now()
        if (now - last_callback_handle).to_sec() > 1.0 / callback_handle_frequency:
            last_callback_handle = now
            rospy.sleep(0.01)

    def cleanup_node():
        rospy.loginfo("Shutting down sensors and motors connected to NXT1 ports")
        right_tread_motor.run(0, 0)
        left_tread_motor.run(0, 0)
        torso_motor.run(0, 0)
        ultrasonic_sensor.command(nxt.sensor.Ultrasonic.Commands.OFF)
    rospy.on_shutdown(cleanup_node)

if __name__ == '__main__':
    main()