#!/usr/bin/env python

import roslib; roslib.load_manifest('nxt2_ros')
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
from std_msgs.msg import Bool, String
from nxt_msgs.msg import ArmState, ArmCommand, Contact, JointCommand, Color, Light, RFID, Sound, HeadState, HeadCommand, LaserState, LaserCommand, TorsoState, TorsoCommand
from PyKDL import Rotation
from time import sleep

power_to_nm = 0.01
power_max = 125
head_max_left = -410
head_max_right= 410
head_center=0
laser_up = 95
laser_down = 0
forearms_up = 250
hands_closed = 180
hands_open = 0
wrists_rotated = -140

global head_motor_command_tacho_units
global head_motor_command_timeout
global head_motor_command_adjusted_power
global head_motor_command_brake
global head_motor_command_status_units
global head_motor_command_status_name
global laser_motor_command_tacho_units
global laser_motor_command_timeout
global laser_motor_command_adjusted_power
global laser_motor_command_brake
global laser_motor_command_status_units
global laser_motor_command_status_name
global arms_motor_command_tacho_units
global arms_motor_command_timeout
global arms_motor_command_adjusted_power
global arms_motor_command_brake
global arms_motor_command_status_units
global arms_motor_command_status_name
global line_following_sensor_illuminated
global color_sensor_command_color
global my_lock
head_motor_command_adjusted_power = 0
head_motor_command_brake = False
head_motor_command_tacho_units = 0
head_motor_command_timeout = 0
head_motor_command_status_name = ""
head_motor_command_status_units = 99999999
laser_motor_command_adjusted_power = 0
laser_motor_command_brake = False
laser_motor_command_tacho_units = 0
laser_motor_command_timeout = 0
laser_motor_command_status_name = ""
laser_motor_command_status_units = 99999999
arms_motor_command_adjusted_power = 0
arms_motor_command_brake = False
arms_motor_command_tacho_units = 0
arms_motor_command_timeout = 0
arms_motor_command_status_name = ""
arms_motor_command_status_units = 99999999
line_following_sensor_illuminated = False
color_sensor_command_color = ""

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

class HeadMotor(Device):
    def __init__(self, params_head_motor, comm):
        Device.__init__(self, params_head_motor)
        self.last_head_motor_state = None

    def head_motor_command_callback(self, head_motor_command):
        head_motor_command_adjusted_power = 0
        head_motor_command_brake = False
        head_motor_command_tacho_units = 0
        head_motor_command_timeout = 0
        head_motor_command_status_name = ""
        head_motor_command_status_units = 99999999
        if head_motor_command.name == "head_motor_command":
            head_motor_command_adjusted_power = head_motor_command.effort / power_to_nm
            head_motor_command_brake = head_motor_command.brake
            head_motor_command_tacho_units = head_motor_command.tacho_units
            head_motor_command_timeout = head_motor_command.timeout
            head_motor_command_status_name = head_motor_command.status_name
            head_motor_command_status_units = head_motor_command.status_units
            if head_motor_command_adjusted_power > power_max:
                head_motor_command_adjusted_power = power_max
            elif head_motor_command_adjusted_power < -power_max:
                head_motor_command_adjusted_power = -power_max
            self.last_head_motor_command = head_motor_command  # save command
            if head_motor_command_status_name != "":
                if head_motor_command_status_name == "head_max_left":
                    head_motor_command_status_units = head_max_left
                if head_motor_command_status_name == "head_max_right":
                    head_motor_command_status_units = head_max_right
                if head_motor_command_status_name == "head_center":
                    head_motor_command_status_units = head_center

    def trigger(self):
        head_motor_command_subscriber = rospy.Subscriber("head_motor_command", JointCommand,
                                                         self.head_motor_command_callback, None, 2)
        head_motor_state = JointState()
        head_motor_state_details = HeadState()
        head_motor_state.header.stamp = rospy.Time.now()
        head_motor_state.name.append("head_motor")
        (output_state, tacho_state) = head_motor._read_state()
        power = output_state.power
        mode = output_state.mode
        regulation = output_state.regulation
        turn_ratio = output_state.turn_ratio
        run_state = output_state.run_state
        tacho_limit = output_state.tacho_limit
        tacho_count = tacho_state.tacho_count
        block_tacho_count = tacho_state.block_tacho_count
        rotation_count = tacho_state.rotation_count
        head_motor_state.position.append(rotation_count * math.pi / 180.0)
        head_motor_state.effort.append(power * power_to_nm)
        velocity = 0
        if self.last_head_motor_state:
            velocity = (head_motor_state.position[0] - self.last_head_motor_state.position[0]) / (
                    head_motor_state.header.stamp - self.last_head_motor_state.header.stamp).to_sec()
        else:
            velocity = 0
            head_motor_state.velocity.append(velocity)
        head_motor_state_publisher.publish(head_motor_state)
        self.last_head_motor_state = head_motor_state

        if head_motor_command_status_units != 99999999 :
            distance_to_status_units = head_motor_command_status_units - rotation_count
            if distance_to_status_units > 0:
                head_motor.turn(power=head_motor_command_adjusted_power, tacho_units=distance_to_status_units,
                                 brake=head_motor_command_brake)
            if distance_to_status_units < 0:
                distance_to_status_units = abs(distance_to_status_units)
                head_motor.turn(power= - head_motor_command_adjusted_power, tacho_units=distance_to_status_units,
                                 brake=head_motor_command_brake)
            elif distance_to_status_units == 0:
                rospy.logwarn("Head already at the specified position")

        if rotation_count >= head_max_right and head_motor_command_adjusted_power >  0:
            rospy.logerr("Head already at the maximal right position")
        elif rotation_count <= head_max_left and head_motor_command_adjusted_power < 0:
            rospy.logerr("Head already at the maximal left position")
        elif (rotation_count < head_max_right and head_motor_command_adjusted_power >= 0) or (
                rotation_count > head_max_left and head_motor_command_adjusted_power <= 0):
            if head_motor_command_tacho_units != 0 and head_motor_command_timeout ==0:
                head_motor.turn(power=head_motor_command_adjusted_power,
                                tacho_units=head_motor_command_tacho_units, brake=head_motor_command_brake)
            elif head_motor_command_timeout != 0 and head_motor_command_tacho_units == 0:
                head_motor.turn(power=head_motor_command_adjusted_power, brake=head_motor_command_brake,
                                timeout=head_motor_command_timeout)
            elif head_motor_command_timeout == 0 and head_motor_command_tacho_units ==0:
                head_motor.run(head_motor_command_adjusted_power, 0)
            (output_state, tacho_state) = head_motor._read_state()
            power = output_state.power
            mode = output_state.mode
            regulation = output_state.regulation
            turn_ratio = output_state.turn_ratio
            run_state = output_state.run_state
            tacho_limit = output_state.tacho_limit
            tacho_count = tacho_state.tacho_count
            block_tacho_count = tacho_state.block_tacho_count
            rotation_count = tacho_state.rotation_count
            if rotation_count == head_center:
                head_motor_state_details.status.append("head_center")
            if rotation_count == head_max_right:
                head_motor_state_details.status.append("head_max_right")
            if rotation_count == head_max_left:
                head_motor_state_details.status.append("head_max_left")
            head_motor_state_details_publisher.publish(head_motor_state_details)

class LaserMotor(Device):
    def __init__(self, params_laser_motor, comm):
        Device.__init__(self, params_laser_motor)
        self.last_laser_motor_state = None

    def laser_motor_command_callback(self, laser_motor_command):
        laser_motor_command_adjusted_power = 0
        laser_motor_command_brake = False
        laser_motor_command_tacho_units = 0
        laser_motor_command_timeout = 0
        laser_motor_command_status_name = ""
        laser_motor_command_status_units = 99999999
        if laser_motor_command.name == "laser_motor_command":
            laser_motor_command_adjusted_power = laser_motor_command.effort / power_to_nm
            laser_motor_command_brake = laser_motor_command.brake
            laser_motor_command_tacho_units = laser_motor_command.tacho_units
            laser_motor_command_timeout = laser_motor_command.timeout
            laser_motor_command_status_name = laser_motor_command.status_name
            laser_motor_command_status_units = laser_motor_command.status_units
            if laser_motor_command_adjusted_power > power_max:
                laser_motor_command_adjusted_power = power_max
            elif laser_motor_command_adjusted_power < -power_max:
                laser_motor_command_adjusted_power = -power_max
            self.last_laser_motor_command = laser_motor_command  # save command
            if laser_motor_command_status_name != "":
                if laser_motor_command_status_name == "laser_up":
                    laser_motor_command_status_units = laser_up
                if laser_motor_command_status_name == "laser_down":
                    laser_motor_command_status_units = laser_down

    def trigger(self):
        laser_motor_command_subscriber = rospy.Subscriber("laser_motor_command", JointCommand,
                                                          self.laser_motor_command_callback, None, 2)
        laser_motor_state = JointState()
        laser_motor_state_details = LaserState()
        laser_motor_state.header.stamp = rospy.Time.now()
        laser_motor_state.name.append("laser_motor")
        (output_state, tacho_state) = laser_motor._read_state()
        power = output_state.power
        mode = output_state.mode
        regulation = output_state.regulation
        turn_ratio = output_state.turn_ratio
        run_state = output_state.run_state
        tacho_limit = output_state.tacho_limit
        tacho_count = tacho_state.tacho_count
        block_tacho_count = tacho_state.block_tacho_count
        rotation_count = tacho_state.rotation_count
        laser_motor_state.position.append(rotation_count * math.pi / 180.0)
        laser_motor_state.effort.append(power * power_to_nm)
        velocity = 0
        if self.last_laser_motor_state:
            velocity = (laser_motor_state.position[0] - self.last_laser_motor_state.position[0]) / (
                    laser_motor_state.header.stamp - self.last_laser_motor_state.header.stamp).to_sec()
        else:
            velocity = 0
            laser_motor_state.velocity.append(velocity)
        laser_motor_state_publisher.publish(laser_motor_state)
        self.last_laser_motor_state = laser_motor_state

        if laser_motor_command_status_units != 99999999 :
            distance_to_status_units = laser_motor_command_status_units - rotation_count
            if distance_to_status_units > 0:
                laser_motor.turn(power=laser_motor_command_adjusted_power, tacho_units=distance_to_status_units,
                                 brake=laser_motor_command_brake)
            if distance_to_status_units < 0:
                distance_to_status_units = abs(distance_to_status_units)
                laser_motor.turn(power= - laser_motor_command_adjusted_power, tacho_units=distance_to_status_units,
                                 brake=laser_motor_command_brake)
            elif distance_to_status_units == 0:
                rospy.logwarn("Laser already at the specified position")

        if rotation_count >= laser_up and laser_motor_command_adjusted_power > 0:
            rospy.logerr("Laser already at up position")
        elif rotation_count <= laser_down and laser_motor_command_adjusted_power < 0:
            rospy.logerr("Laser already at down position")
        elif (rotation_count > laser_up and laser_motor_command_adjusted_power <= 0) or (
                rotation_count > laser_down and laser_motor_command_adjusted_power >= 0):
            if laser_motor_command_tacho_units != 0 and laser_motor_command_timeout == 0:
                laser_motor.turn(power=laser_motor_command_adjusted_power,
                                 tacho_units=laser_motor_command_tacho_units, brake=laser_motor_command_brake)
            elif laser_motor_command_timeout !=0 and laser_motor_command_tacho_units ==0 :
                laser_motor.turn(power=laser_motor_command_adjusted_power, brake=laser_motor_command_brake,
                                 timeout=laser_motor_command_timeout)
            elif laser_motor_command_timeout ==0  and laser_motor_command_tacho_units ==0:
                laser_motor.run(laser_motor_command_adjusted_power, 0)
            (output_state, tacho_state) = laser_motor._read_state()
            power = output_state.power
            mode = output_state.mode
            regulation = output_state.regulation
            turn_ratio = output_state.turn_ratio
            run_state = output_state.run_state
            tacho_limit = output_state.tacho_limit
            tacho_count = tacho_state.tacho_count
            block_tacho_count = tacho_state.block_tacho_count
            rotation_count = tacho_state.rotation_count
            if rotation_count == laser_up:
                laser_motor_state_details.status.append("laser_up")
            if rotation_count == laser_down:
                laser_motor_state_details.status.append("laser_down")
            laser_motor_state_details_publisher.publish(laser_motor_state_details)

class ArmsMotor(Device):
    def __init__(self, params_arms_motor, comm):
        Device.__init__(self, params_arms_motor)
        self.last_arms_motor_state = None

    def arms_motor_command_callback(self, arms_motor_command):
        arms_motor_command_adjusted_power = 0
        arms_motor_command_brake = False
        arms_motor_command_tacho_units = 0
        arms_motor_command_timeout = 0
        arms_motor_command_status_name = ""
        arms_motor_command_status_units = 99999999
        if arms_motor_command.name == "arms_motor_command":
            arms_motor_command_adjusted_power = arms_motor_command.effort / power_to_nm
            arms_motor_command_brake = arms_motor_command.brake
            arms_motor_command_tacho_units = arms_motor_command.tacho_units
            arms_motor_command_timeout = arms_motor_command.timeout
            arms_motor_command_status_name = arms_motor_command.status_name
            arms_motor_command_status_units = arms_motor_command.status_units
            if arms_motor_command_adjusted_power > power_max:
                arms_motor_command_adjusted_power = power_max
            elif arms_motor_command_adjusted_power < -power_max:
                arms_motor_command_adjusted_power = -power_max
            self.last_arms_motor_command = arms_motor_command  # save command
            if arms_motor_command_status_name != "":
                if arms_motor_command_status_name == "forearms_up":
                    arms_motor_command_status_units = forearms_up
                if arms_motor_command_status_name == "hands_closed":
                    arms_motor_command_status_units = hands_closed
                if arms_motor_command_status_name == "hands_open":
                    arms_motor_command_status_units = hands_open
                if arms_motor_command_status_name == "wrists_rotated":
                    arms_motor_command_status_units = wrists_rotated

    def trigger(self):
        arms_motor_command_subscriber = rospy.Subscriber("arms_motor_command", TorsoCommand,
                                                         self.arms_motor_command_callback, None, 2)
        arms_motor_state = JointState()
        arms_motor_state_details = ArmState()
        arms_motor_state.header.stamp = rospy.Time.now()
        arms_motor_state.name.append("arms_motor")
        (output_state, tacho_state) = arms_motor._read_state()
        power = output_state.power
        mode = output_state.mode
        regulation = output_state.regulation
        turn_ratio = output_state.turn_ratio
        run_state = output_state.run_state
        tacho_limit = output_state.tacho_limit
        tacho_count = tacho_state.tacho_count
        block_tacho_count = tacho_state.block_tacho_count
        rotation_count = tacho_state.rotation_count
        arms_motor_state.position.append(rotation_count * math.pi / 180.0)
        arms_motor_state.effort.append(power * power_to_nm)
        velocity = 0
        if self.last_arms_motor_state:
            velocity = (arms_motor_state.position[0] - self.last_arms_motor_state.position[0]) / (
                    arms_motor_state.header.stamp - self.last_arms_motor_state.header.stamp).to_sec()
        else:
            velocity = 0
            arms_motor_state.velocity.append(velocity)
        arms_motor_state_publisher.publish(arms_motor_state)
        self.last_arms_motor_state = arms_motor_state

        if arms_motor_command_status_units != 99999999 :
            distance_to_status_units = arms_motor_command_status_units - rotation_count
            if distance_to_status_units > 0:
                arms_motor.turn(power=arms_motor_command_adjusted_power, tacho_units=distance_to_status_units,
                                 brake=arms_motor_command_brake)
            if distance_to_status_units < 0:
                distance_to_status_units = abs(distance_to_status_units)
                arms_motor.turn(power= - arms_motor_command_adjusted_power, tacho_units=distance_to_status_units,
                                 brake=arms_motor_command_brake)
            elif distance_to_status_units == 0:
                rospy.logwarn("Arms already at the specified position")

        if rotation_count <= wrists_rotated and arms_motor_command_adjusted_power < 0:
            rospy.logerr("Arms already at minimum position")
        elif rotation_count >= forearms_up and arms_motor_command_adjusted_power > 0:
            rospy.logerr("Arms already at maximum position")
        elif (rotation_count < forearms_up and arms_motor_command_adjusted_power <= 0) or (
                rotation_count > wrists_rotated and arms_motor_command_adjusted_power >= 0):
            if arms_motor_command_tacho_units != 0 and arms_motor_command_timeout ==0:
                arms_motor.turn(power=arms_motor_command_adjusted_power,
                                tacho_units=arms_motor_command_tacho_units, brake=arms_motor_command_brake)
            elif arms_motor_command_timeout != 0 and arms_motor_command_tacho_units ==0:
                arms_motor.turn(power=arms_motor_command_adjusted_power, brake=arms_motor_command_brake,
                                timeout=arms_motor_command_timeout)
            elif arms_motor_command_timeout ==0 and arms_motor_command_tacho_units ==0:
                arms_motor.run(arms_motor_command_adjusted_power, 0)
            (output_state, tacho_state) = arms_motor._read_state()
            power = output_state.power
            mode = output_state.mode
            regulation = output_state.regulation
            turn_ratio = output_state.turn_ratio
            run_state = output_state.run_state
            tacho_limit = output_state.tacho_limit
            tacho_count = tacho_state.tacho_count
            block_tacho_count = tacho_state.block_tacho_count
            rotation_count = tacho_state.rotation_count
            if rotation_count == forearms_up:
                arms_motor_state_details.status.append("forearms_up")
            if rotation_count == hands_closed:
                arms_motor_state_details.status.append("hands_closed")
            if rotation_count == hands_open:
                arms_motor_state_details.status.append("hands_open")
            if rotation_count == wrists_rotated:
                arms_motor_state_details.status.append("wrists_rotated")
            arms_motor_state_details_publisher.publish(arms_motor_state_details)

class SoundSensor(Device):
    def __init__(self, params_sound_sensor, comm):
        Device.__init__(self, params_sound_sensor)
    def trigger(self):
        sound_sensor_report = Sound()
        sound_sensor_report.header.frame_id="sound_sensor"
        sound_sensor_report.header.stamp = rospy.Time.now()
        sound_sensor.set_adjusted(active=False)
        sound_sensor_report.data_db =sound_sensor.get_loudness()
        sound_sensor.set_adjusted(active=True)
        sound_sensor_report.data_dbA = sound_sensor.get_loudness()
        sound_sensor_publisher.publish(sound_sensor_report)

class ColorSensor(Device):
    def __init__(self, params_color_sensor, comm):
        Device.__init__(self, params_color_sensor)
    def color_sensor_command_callback(self, color_sensor_command):
        color_sensor_command_color = ""
        color_sensor_command_color = color_sensor_command.data
    def trigger(self):
        color_sensor_command_subscriber = rospy.Subscriber("color_sensor_command", String,
                                                           self.color_sensor_command_callback, None, 2)
        if color_sensor_command_color != "":
            if color_sensor_command_color == "RED":
                color_sensor.set_light_color(Type.COLORRED)
            if color_sensor_command_color == "GREEN":
                color_sensor.set_light_color(Type.COLORGREEN)
            if color_sensor_command_color == "BLUE":
                color_sensor.set_light_color(Type.COLORBLUE)
            if color_sensor_command_color == "NONE":
                color_sensor.set_light_color(Type.COLORNONE)
            if color_sensor_command_color == "FULL":
                color_sensor.set_light_color(Type.COLORFULL)
        color_sensor_report = Color()
        color_sensor_report.header.frame_id = "color_sensor"
        color_sensor_report.header.stamp = rospy.Time.now()
        color_sensor_report.intensity = color_sensor.get_reflected_light(Type.COLORFULL)
        color_code =  color_sensor.get_color()
        if color_code == 1:  # black
            color_sensor_report.r = 0.0
            color_sensor_report.g = 0.0
            color_sensor_report.b = 0.0
        elif color_code == 2:  # blue
            color_sensor_report.r = 0.0
            color_sensor_report.g = 0.0
            color_sensor_report.b = 1.0
        elif color_code == 3:  # green
            color_sensor_report.r = 0.0
            color_sensor_report.g = 1.0
            color_sensor_report.b = 0.0
        elif color_code == 4:  # yellow
            color_sensor_report.r = 1.0
            color_sensor_report.g = 1.0
            color_sensor_report.b = 0.0
        elif color_code == 5:  # red
            color_sensor_report.r = 1.0
            color_sensor_report.g = 0.0
            color_sensor_report.b = 1.0
        elif color_code == 6:  # white
            color_sensor_report.r = 1.0
            color_sensor_report.g = 1.0
            color_sensor_report.b = 1.0
        else:
            rospy.logerr('Undefined color of color sensor')
        color_sensor_report.light_color = color_sensor.get_light_color()
        color_sensor_publisher.publish(color_sensor_report)

class LineFollowingSensor(Device):
    def __init__(self, params_line_following_sensor, comm):
        Device.__init__(self, params_line_following_sensor)
    def line_following_sensor_command_callback(self, line_following_sensor_light_on):
        line_following_sensor_illuminated = False
        line_following_sensor_illuminated = line_following_sensor_light_on.data
    def trigger(self):
        line_following_sensor_command_subscriber = rospy.Subscriber("line_following_sensor_light_on", Bool,
                                                                    self.line_following_sensor_command_callback, None, 2)
        if line_following_sensor_illuminated:
            line_following_sensor.set_illuminated(active=True)
        line_following_sensor_report = Light()
        line_following_sensor_report.header.frame_id = "line_following_sensor"
        line_following_sensor_report.header.stamp = rospy.Time.now()
        line_following_sensor.set_illuminated(active=False)
        line_following_sensor_report.ambiant_light_intensity =line_following_sensor.get_lightness()
        line_following_sensor.set_illuminated(active=True)
        line_following_sensor_report.active_light_intensity = line_following_sensor.get_lightness()
        line_following_sensor_publisher.publish(line_following_sensor_report)

class RFIDSensor(Device):
    def __init__(self, params_rfid_sensor, comm):
        Device.__init__(self, params_rfid_sensor)
    def trigger(self):
        rfid_sensor_report = RFID()
        rfid_sensor_report.header.frame_id="rfid_sensor"
        rfid_sensor_report.header.stamp = rospy.Time.now()
        #brick.stop_program()
        brick.start_program('rfid_read.rxe')
        sleep(1)
        read_data=nxt.brick.FileReader(brick, "rfid_read.txt")
        data=read_data.read(bytes=None)
        rfid_read=data[0:12]
        rfid_sensor_report.data=rfid_read
        rfid_sensor_report.sensor_type = "RFID"
        rfid_sensor_report.sensor_manufacturer ="Codatex"
        rfid_sensor_report.firmware_version = "V1.0"
        rfid_sensor_report.sensor_serial = "08010204"
        rfid_sensor_publisher.publish(rfid_sensor_report)

# Publishers definition
head_motor_state_publisher = rospy.Publisher("head_motor_state", JointState, queue_size=1)
rospy.loginfo("Setup publisher on head_motor_state [sensor_msgs/JointState]")
head_motor_state_details_publisher = rospy.Publisher("head_motor_state_details", HeadState, queue_size=1)
rospy.loginfo("Setup publisher on head_motor_state_details [nxt_msgs/HeadState]")
laser_motor_state_publisher = rospy.Publisher("laser_motor_state", JointState, queue_size=1)
rospy.loginfo("Setup publisher on laser_motor_state [sensor_msgs/JointState]")
laser_motor_state_details_publisher = rospy.Publisher("laser_motor_state_details", LaserState, queue_size=1)
rospy.loginfo("Setup publisher on laser_motor_state_details [nxt_msgs/LaserState]")
arms_motor_state_publisher = rospy.Publisher("arms_motor_state", JointState, queue_size=1)
rospy.loginfo("Setup publisher on arms_motor_state [sensor_msgs/JointState]")
arms_motor_state_details_publisher = rospy.Publisher("arms_motor_state_details", ArmState, queue_size=1)
rospy.loginfo("Setup publisher on arms_motor_state_details [nxt_msgs/ArmState]")
color_sensor_publisher = rospy.Publisher("color_sensor", Color, queue_size=1)
rospy.loginfo("Setup publisher on color_sensor [nxt_msgs/Color")
sound_sensor_publisher = rospy.Publisher("sound_sensor", Sound, queue_size=1)
rospy.loginfo("Setup publisher on sound_sensor [nxt_msgs/Sound]")
line_following_sensor_publisher = rospy.Publisher("line_following_sensor", Light, queue_size=1)
rospy.loginfo("Setup publisher on line_following_sensor [nxt_msgs/Light]")
rfid_sensor_publisher = rospy.Publisher("rfid_sensor", RFID, queue_size=1)
rospy.loginfo("Setup publisher on rfid_sensor [nxt_msgs/RFID]")

def main():
    global brick
    brick = nxt.locator.find_one_brick(name="NXT2")
    rospy.logdebug("Connected to NXT2")
    rospy.init_node("nxt2_ros")
    callback_handle_frequency = 10.0
    last_callback_handle = rospy.Time.now()

    components = []
    global head_motor
    head_motor = Motor(brick, PORT_A)
    rospy.loginfo("Connecting to head_motor on NX2_PORT_A")
    params_head_motor = {'type': 'Motor', 'name': 'head_motor', 'port': 'PORT_A', 'desired_frequency': 5}
    components.append(HeadMotor(params_head_motor, brick))
    global laser_motor
    laser_motor = Motor(brick, PORT_B)
    rospy.loginfo("Connecting to laser_motor on NXT2_PORT_B")
    params_laser_motor = {'type': 'Motor', 'name': 'laser_motor', 'port': 'PORT_B', 'desired_frequency': 1}
    components.append(LaserMotor(params_laser_motor, brick))
    global arms_motor
    arms_motor = Motor(brick, PORT_C)
    rospy.loginfo("Connecting to arms_motor on NXT2_PORT_C")
    params_arms_motor = {'type': 'Motor', 'name': 'arms_motor', 'port': 'PORT_C', 'desired_frequency': 5}
    components.append(ArmsMotor(params_arms_motor, brick))
    global sound_sensor
    sound_sensor = nxt.sensor.Sound(brick, PORT_1)
    rospy.loginfo("Connecting to sound_sensor on NXT2_PORT_1")
    params_sound_sensor = {'type': 'Sound', 'name': 'sound_sensor', 'port': 'PORT_1', 'desired_frequency': 5}
    components.append(SoundSensor(params_sound_sensor, brick))
    global color_sensor
    color_sensor = nxt.sensor.Color20(brick, PORT_2)
    rospy.loginfo("Connecting to color_sensor on NXT2_PORT_2")
    params_color_sensor = {'type': 'Color', 'name': 'color_sensor', 'port': 'PORT_2', 'desired_frequency': 1}
    components.append(ColorSensor(params_color_sensor, brick))
    global line_following_sensor
    line_following_sensor = nxt.sensor.Light(brick, PORT_3)
    rospy.loginfo("Connecting to line_following_sensor on NXT2_PORT_3")
    params_line_following_sensor = {'type': 'Light', 'name': 'line_following_sensor', 'port': 'PORT_3', 'desired_frequency': 5}
    components.append(LineFollowingSensor(params_line_following_sensor, brick))

    rfid_sensor = "rfid_read" # the rfid_read program needs to be stored in the NXT2 brick
    rospy.loginfo("Connecting to RFID_sensor on NXT2_PORT_4")
    params_rfid_sensor = {'type': 'RFID', 'name': 'rfid_sensor', 'port': 'PORT_4', 'desired_frequency': 1}
    components.append(RFIDSensor(params_rfid_sensor, brick))

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
        rospy.loginfo("Shutting down sensors and motors connected to NXT2 ports")
        head_motor.run(0, 0)
        arms_motor.run(0, 0)
        laser_motor.run(0, 0)
        color_sensor.set_light_color(Type.COLORNONE)
        line_following_sensor.set_illuminated(active=False)
    rospy.on_shutdown(cleanup_node)

if __name__ == '__main__':
    main()
