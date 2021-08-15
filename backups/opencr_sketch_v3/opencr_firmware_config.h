/*******************************************************************************
* Copyright 2020 Rodolphe MATIAS DE SOUSA.
* Based on TurtleBot3 openCR script from ROBOTIS
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Rodolphe MATIAS DE SOUSA */

/* Version : 20200214_v3 */

#ifndef OPENCR_FIRMWARE_CONFIG_H_
#define OPENCR_FIRMWARE_CONFIG_H_

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Byte.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/RelativeHumidity.h>

#include <TurtleBot3.h>

#include <math.h>
#include <dht.h>

/*******************************************************************************
* Define sensors publish frequency 
*******************************************************************************/
#define IMU_PUBLISH_FREQUENCY                  50  //hz
#define MAG_PUBLISH_FREQUENCY                  50  //hz
#define BATTERY_PUBLISH_FREQUENCY              5   //hz
#define TEMP_PUBLISH_FREQUENCY                 0.2 //hz
#define HUMID_PUBLISH_FREQUENCY                0.2 //hz
#define BUTTONS_PUBLISH_FREQUENCY              5   //hz

/*******************************************************************************
* Define hardware map
*******************************************************************************/
#define PUSH_BUTTON_1                          34
#define PUSH_BUTTON_2                          35
#define USER_LED_1                             22
#define USER_LED_2                             23
#define USER_LED_3                             24
#define USER_LED_4                             25
#define BUZZER                                 31
#define TEMPERATURE_SENSOR                     0
#define HUMIDITY_SENSOR                        0
#define CURRENT_SENSOR                         2

/*******************************************************************************
* Define deug serial 
*******************************************************************************/                        
#define DEBUG_SERIAL                     SerialBT2

/*******************************************************************************
* Function prototypes
*******************************************************************************/
// Callback function prototypes
void buzzerMelodyCallback(const std_msgs::String& buzzerMelody_msg);
void ledsStatusCallback(const std_msgs::String& ledsStatus_msg);
void setLedOnCallback(const std_msgs::Byte& setLedOn_msg);
void setLedOffCallback(const std_msgs::Byte& setLedOff_msg);
// Function prototypes
void publishOpenCRButtonsStatus(void);
void publishImuMsg(void);
void publishMagMsg(void);
void publishBatteryStateMsg(void);
void publishAmbiantTempMsg(void);
void publishRelativeHumidityMsg(void);

void batteryMonitor(void);
/*******************************************************************************
* Variable prototypes
*******************************************************************************/
ros::Time rosNow(void);
ros::Time addMicros(ros::Time & t, uint32_t _micros); // deprecated
void updateVariable(bool isConnected);
void updateTime(void);
void updateGyroCali(bool isConnected);
void sendLogMsg(void);
void waitForSerialLink(bool isConnected);

/*******************************************************************************
* ROS NodeHandle
* We need to instantiate the node handle, which allows our program to 
* create publishers and subscribers. The node handle also takes care of serial 
* port communications.
*******************************************************************************/
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

/*******************************************************************************
* Subscriber
* We need to instantiate the subscribers that we will be using.
*******************************************************************************/
// Buzzer melody : BOOT, SHUTDOWN, LOW_BATTERY, ERROR, BUTTON1, BUTTON2, DEFFAULT
ros::Subscriber<std_msgs::String> buzzer_melody_sub("buzzer_melody", buzzerMelodyCallback);

// User leds status
ros::Subscriber<std_msgs::String> leds_status_sub("leds_status", ledsStatusCallback);

// User led manual control
ros::Subscriber<std_msgs::Byte> set_led_on_sub("set_led_on", setLedOnCallback); 
ros::Subscriber<std_msgs::Byte> set_led_off_sub("set_led_off", setLedOffCallback);

/*******************************************************************************
* Publisher
* We need to instantiate the publishers that we will be using.
*******************************************************************************/
// IMU
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu_data", &imu_msg);

// Battery state
sensor_msgs::BatteryState battery_state_msg;
ros::Publisher battery_state_pub("battery_state", &battery_state_msg);

// Magnetic field
sensor_msgs::MagneticField mag_msg;
ros::Publisher mag_pub("magnetic_field", &mag_msg);

// Temperature
sensor_msgs::Temperature ambiant_temp_msg;
ros::Publisher ambiant_temperature_pub("ambiant_temperature", &ambiant_temp_msg);

// Relative humidity
sensor_msgs::RelativeHumidity relative_humidity_msg;
ros::Publisher relative_humidity_pub("relative_humidity", &relative_humidity_msg);

// OpenCR buttons
std_msgs::Byte openCR_buttons_status_msg;
ros::Publisher openCR_buttons_status_pub("openCR_buttons_status", &openCR_buttons_status_msg);

// Emergency stop
std_msgs::String emergency_shutdown_msg;
ros::Publisher emergency_shutdown_pub("emergency_shutdown_request", &emergency_shutdown_msg);

/*******************************************************************************
* SoftwareTimer of OpenCR
*******************************************************************************/
static uint32_t tTime[10];

/*******************************************************************************
* Declaration for sensors
*******************************************************************************/
Turtlebot3Sensor sensors;

/*******************************************************************************
* Declaration for Battery
*******************************************************************************/
bool setup_end        = false;
uint8_t battery_state = 0;

#endif // OPENCR_FIRMWARE_CONFIG_H_
