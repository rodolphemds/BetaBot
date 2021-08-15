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

/* Version : 20200308_v4 */

/*******************************************************************************
* As a part of every ROS Arduino program, you need to include the ros.h header 
* file and header files for any messages that you will be using.
*******************************************************************************/
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Byte.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/RelativeHumidity.h>
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <IMU.h>
#include <DHT_U.h>
#include <DHT.h>
#include <QuickMedianLib.h>

#include <Arduino.h>

/*******************************************************************************
* Define sensors publish frequency 
*******************************************************************************/
#define IMU_PUBLISH_FREQUENCY                  50  //hz
#define MAG_PUBLISH_FREQUENCY                  50  //hz
#define BATTERY_PUBLISH_FREQUENCY              5   //hz
#define TEMP_HUMID_PUBLISH_FREQUENCY           1   //hz
#define BUTTONS_PUBLISH_FREQUENCY              5   //hz
#define SERIAL_DEBUG_FREQUENCY                 100   //hz

/*******************************************************************************
* ROS NodeHandle
* Next, we need to instantiate the node handle, which allows our program to 
* create publishers and subscribers. The node handle also takes care of serial 
* port communications.
*******************************************************************************/
ros::NodeHandle nh;

#define DHTTYPE DHT11 
#define DHTPIN 2 
DHT dht(DHTPIN, DHTTYPE);
cIMU imu;
bool batteryIsPresent = false;
bool ledManualCtrl = false;
bool setup_done = false;
bool  batteryLowAdvertised = false;
bool battery25Advertised = false;
bool batteryErrorAdvertised = false;

/*******************************************************************************
* Callback function for led control
* Possible led status : BATTERY_100%, BATTERY_75%, BATTERY_50%, BATTERY_25%, 
* BATTERY_VERY_LOW, BOOT, SEQUENCE, NONE
*******************************************************************************/
// User leds status : BATTERY_100%, BATTERY_75%, BATTERY_50%, BATTERY_25%, BOOT, SEQUENCE, NONE 
void setLedStatus(const String& ledStatus)
{
  if (ledStatus == "SEQUENCE") 
  {
    int led_pin_user[4] = { BDPIN_LED_USER_1, BDPIN_LED_USER_2, BDPIN_LED_USER_3, BDPIN_LED_USER_4 };
    int i;
    for (i=0; i<4; i++)
    {
      digitalWrite(led_pin_user[i], LOW);
      delay(100);
    }
    for (i=0; i<4; i++)
    {
      digitalWrite(led_pin_user[i], HIGH);
      delay(100);
    }
   }
   if (ledStatus == "BOOT")
   {
    int led_pin_user[4] = { BDPIN_LED_USER_1, BDPIN_LED_USER_2, BDPIN_LED_USER_3, BDPIN_LED_USER_4 };
    int i;
    for (i=0; i<4; i++)
    {
      digitalWrite(led_pin_user[i], LOW);
      delay(500);
    }
    for (i=0; i<4; i++)
    {
      digitalWrite(led_pin_user[i], HIGH);
    }
   }
   if (ledStatus == "BATTERY_100%") 
   {
    int led_pin_user[4] = { BDPIN_LED_USER_1, BDPIN_LED_USER_2, BDPIN_LED_USER_3, BDPIN_LED_USER_4 };
    int i;
    for (i=0; i<4; i++)
    {
      digitalWrite(led_pin_user[i], HIGH);
    }
    for (i=0; i<4; i++)
    {
      digitalWrite(led_pin_user[i], LOW);
    }
   }
   if (ledStatus == "BATTERY_75%") 
   {
    int led_pin_user[4] = { BDPIN_LED_USER_1, BDPIN_LED_USER_2, BDPIN_LED_USER_3, BDPIN_LED_USER_4 };
    int i;
    for (i=0; i<4; i++)
    {
      digitalWrite(led_pin_user[i], HIGH);
    }
    for (i=0; i<3; i++)
    {
      digitalWrite(led_pin_user[i], LOW);
    }
   }
        if (ledStatus == "BATTERY_50%") {
        int led_pin_user[4] = { BDPIN_LED_USER_1, BDPIN_LED_USER_2, BDPIN_LED_USER_3, BDPIN_LED_USER_4 };
        int i;
        for (i=0; i<4; i++)
        {
        digitalWrite(led_pin_user[i], HIGH);
        }
        for (i=0; i<2; i++)
        {
        digitalWrite(led_pin_user[i], LOW);
        }
        }
        if (ledStatus == "BATTERY_25%") {
        int led_pin_user[4] = { BDPIN_LED_USER_1, BDPIN_LED_USER_2, BDPIN_LED_USER_3, BDPIN_LED_USER_4 };
        int i;
        for (i=0; i<4; i++)
        {
        digitalWrite(led_pin_user[i], HIGH);
        }
        for (i=0; i<1; i++)
        {
        digitalWrite(led_pin_user[i], LOW);
        }
        }
        if (ledStatus == "BATTERY_VERY_LOW" || ledStatus == "NONE" || ledStatus == ""  ) {
        int led_pin_user[4] = { BDPIN_LED_USER_1, BDPIN_LED_USER_2, BDPIN_LED_USER_3, BDPIN_LED_USER_4 };
        int i;
        for (i=0; i<4; i++)
        {
        digitalWrite(led_pin_user[i], HIGH);
        }
        }
      }
   
    // User led on/off
    void turnLedOn(const std_msgs::Byte& ledNumber)
    {
      uint32_t ledPinNumber;
      if (ledNumber.data == 1)
      {
        ledPinNumber = BDPIN_LED_USER_1;
      }
      if (ledNumber.data == 2)
      {
        ledPinNumber = BDPIN_LED_USER_2;
      }
      if (ledNumber.data == 3)
      {
        ledPinNumber = BDPIN_LED_USER_3;
      }
      if (ledNumber.data == 4)
      {
        ledPinNumber = BDPIN_LED_USER_4;
      }
      digitalWrite(ledPinNumber, LOW);
    }
    void turnLedOff(const std_msgs::Byte& ledNumber)
    {
      uint32_t ledPinNumber;
      if (ledNumber.data == 1)
      {
        ledPinNumber = BDPIN_LED_USER_1;
      }
      if (ledNumber.data == 2)
      {
        ledPinNumber = BDPIN_LED_USER_2;
      }
      if (ledNumber.data == 3)
      {
        ledPinNumber = BDPIN_LED_USER_3;
      }
      if (ledNumber.data == 4)
      {
        ledPinNumber = BDPIN_LED_USER_4;
      }
      digitalWrite(ledPinNumber, HIGH);
    }
// Callback function
void ledsStatusCallback(const std_msgs::String& ledsStatus_msg)
{
  setLedStatus(ledsStatus_msg.data);
  bool ledManualCtrl = true; 
}
void setLedOnCallback(const std_msgs::Byte& setLedOn_msg)
{
  turnLedOn(setLedOn_msg);
  bool ledManualCtrl = true; 
}
  void setLedOffCallback(const std_msgs::Byte& setLedOff_msg)
{
  turnLedOff(setLedOff_msg);
  bool ledManualCtrl = true; 
}
    
/*******************************************************************************
* Callback function for playing buzzer melody
* Possible melodies : BOOT, SHUTDOWN, LOW_BATTERY, ERROR, BUTTON1, BUTTON2,
* DEFAULT
*******************************************************************************/
   // Melody player function
   void melodyPlayerFunction(uint16_t* note, uint8_t note_num, uint8_t* durations)
   {
   for (int thisNote = 0; thisNote < note_num; thisNote++) 
   {
     // to calculate the note duration, take one second
     // divided by the note type.
     //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
     int noteDuration = 1000 / durations[thisNote];
     tone(BDPIN_BUZZER, note[thisNote], noteDuration);
     // to distinguish the notes, set a minimum time between them.
     // the note's duration + 30% seems to work well:
     int pauseBetweenNotes = noteDuration * 1.30;
     delay(pauseBetweenNotes);
     // stop the tone playing:
     noTone(BDPIN_BUZZER);
    }
   }
   // Define melodies 
   void playMelody(const String& melodyName)
   {
     // define specific notes
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978   
     // define usefull melodies : BOOT, SHUTDOWN, LOW_BATTERY, ERROR, BUTTON1, BUTTON2, DEFAULT
       if (melodyName == "BOOT") {
       byte note_num = 8;
       uint16_t note[note_num] = {0, 0};
       uint8_t  duration[note_num] = {0, 0};
       note[0] = NOTE_C4;   duration[0] = 4;
       note[1] = NOTE_D4;   duration[1] = 4;
       note[2] = NOTE_E4;   duration[2] = 4;
       note[3] = NOTE_F4;   duration[3] = 4;
       note[4] = NOTE_G4;   duration[4] = 4;
       note[5] = NOTE_A4;   duration[5] = 4;
       note[6] = NOTE_B4;   duration[6] = 4;
       note[7] = NOTE_C5;   duration[7] = 4;
       melodyPlayerFunction(note, note_num, duration);   
       }
       if (melodyName == "SHUTDOWN") {
       byte note_num = 8;
       uint16_t note[note_num] = {0, 0};
       uint8_t  duration[note_num] = {0, 0};
       note[0] = NOTE_C5;   duration[0] = 4;
       note[1] = NOTE_B4;   duration[1] = 4;
       note[2] = NOTE_A4;   duration[2] = 4;
       note[3] = NOTE_G4;   duration[3] = 4;
       note[4] = NOTE_F4;   duration[4] = 4;
       note[5] = NOTE_E4;   duration[5] = 4;
       note[6] = NOTE_D4;   duration[6] = 4;
       note[7] = NOTE_C4;   duration[7] = 4;
       melodyPlayerFunction(note, note_num, duration);  
       }
       if (melodyName == "LOW_BATTERY") {
       byte note_num = 8;
       uint16_t note[note_num] = {0, 0};
       uint8_t  duration[note_num] = {0, 0};
       note[0] = 1000;      duration[0] = 1;
       note[1] = 1000;      duration[1] = 1;
       note[2] = 1000;      duration[2] = 1;
       note[3] = 1000;      duration[3] = 1;
       note[4] = 0;         duration[4] = 8;
       note[5] = 0;         duration[5] = 8;
       note[6] = 0;         duration[6] = 8;
       note[7] = 0;         duration[7] = 8;
       melodyPlayerFunction(note, note_num, duration);
       }
       if (melodyName == "ERROR") {
       byte note_num = 8;
       uint16_t note[note_num] = {0, 0};
       uint8_t  duration[note_num] = {0, 0};
       note[0] = 1000;      duration[0] = 3;
       note[1] = 500;       duration[1] = 3;
       note[2] = 1000;      duration[2] = 3;
       note[3] = 500;       duration[3] = 3;
       note[4] = 1000;      duration[4] = 3;
       note[5] = 500;       duration[5] = 3;
       note[6] = 1000;      duration[6] = 3;
       note[7] = 500;       duration[7] = 3;
       melodyPlayerFunction(note, note_num, duration);
       }
       if (melodyName == "BUTTON1") {
       byte note_num = 2;
       uint16_t note[note_num] = {0, 0};
       uint8_t  duration[note_num] = {0, 0};
       note[0] = NOTE_C5;   duration[0] = 2;
       note[1] = NOTE_C5;   duration[0] = 2;
       melodyPlayerFunction(note, note_num, duration);
       }
       if (melodyName ==  "BUTTON2") {
       byte note_num = 2;
       uint16_t note[note_num] = {0, 0};
       uint8_t  duration[note_num] = {0, 0};
       note[0] = NOTE_C5;   duration[0] = 2;
       note[1] = NOTE_C5;   duration[0] = 2;
       melodyPlayerFunction(note, note_num, duration);
       }
       if (melodyName == "DEFAULT") {
       byte note_num = 8;
       uint16_t note[note_num] = {0, 0};
       uint8_t  duration[note_num] = {0, 0};
       note[0] = NOTE_C4;   duration[0] = 4;
       note[1] = NOTE_D4;   duration[1] = 4;
       note[2] = NOTE_E4;   duration[2] = 4;
       note[3] = NOTE_F4;   duration[3] = 4;
       note[4] = NOTE_G4;   duration[4] = 4;
       note[5] = NOTE_A4;   duration[5] = 4;
       note[6] = NOTE_B4;   duration[6] = 4;
       note[7] = NOTE_C4;   duration[7] = 4; 
       melodyPlayerFunction(note, note_num, duration);
       }
   }
// Callback function  
void buzzerMelodyCallback(const std_msgs::String& buzzerMelody_msg)
{
  playMelody(buzzerMelody_msg.data);
}

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
ros::Publisher battery_state_pub("power_state", &battery_state_msg);

// Magnetic field
sensor_msgs::MagneticField mag_msg;
ros::Publisher mag_pub("magnetic_field_sensor", &mag_msg);

// Temperature
sensor_msgs::Temperature ambient_temp_msg;
ros::Publisher ambient_temperature_pub("ambient_temperature_sensor", &ambient_temp_msg);

// Relative humidity
sensor_msgs::RelativeHumidity relative_humidity_msg;
ros::Publisher relative_humidity_pub("relative_humidity_sensor", &relative_humidity_msg);

// OpenCR buttons
std_msgs::Byte openCR_buttons_status_msg;
ros::Publisher openCR_buttons_status_pub("openCR_buttons_status", &openCR_buttons_status_msg);

// Emergency stop
std_msgs::String emergency_shutdown_msg;
ros::Publisher emergency_shutdown_pub("emergency_shutdown_request", &emergency_shutdown_msg);

/*******************************************************************************
* Publish OpenCR buttons status
*******************************************************************************/
uint8_t publishOpenCRButtonsStatus(void)
{
  uint8_t reading = 0;
  if (digitalRead(BDPIN_PUSH_SW_1) == HIGH && digitalRead(BDPIN_PUSH_SW_2) == LOW) 
  {
    reading |= 0x01;
    playMelody("BUTTON1");
  }
  
  else if (digitalRead(BDPIN_PUSH_SW_2) == HIGH && digitalRead(BDPIN_PUSH_SW_1) == LOW)
  {
    reading |= 0x02;
    playMelody("BUTTON2");
  }
  else if (digitalRead(BDPIN_PUSH_SW_2) == LOW && digitalRead(BDPIN_PUSH_SW_1) == LOW)
  {
    reading = 0;
  }
  
  else if (digitalRead(BDPIN_PUSH_SW_1) == HIGH && digitalRead(BDPIN_PUSH_SW_2) == HIGH)
    {
     reading |= 0x03;
     playMelody("BUTTON1");
    }
   openCR_buttons_status_msg.data = reading;
   openCR_buttons_status_pub.publish(&openCR_buttons_status_msg);
   return reading;
  }

/*******************************************************************************
* Publish imu data : angular velocity, linear acceleration, orientation
*******************************************************************************/
void publishImuMsg(void)
{
  #define ACCEL_FACTOR 0.000598550415 // (ADC_Value / Scale) * 9.80665 => Range : +- 2[g] Scale : +- 16384
  #define GYRO_FACTOR 0.0010642 // (ADC_Value/Scale) * (pi/180) => Range : +- 2000[deg/s] Scale : +- 16.4[deg/s]
  
  imu_msg.header.stamp    = nh.now();
  imu_msg.header.frame_id = "imu_link";

  imu_msg.angular_velocity.x = imu.SEN.gyroADC[0] * GYRO_FACTOR;
  imu_msg.angular_velocity.y = imu.SEN.gyroADC[1] * GYRO_FACTOR;
  imu_msg.angular_velocity.z = imu.SEN.gyroADC[2] * GYRO_FACTOR;
  
  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;
  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[5] = 0;
  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0.02;

  imu_msg.linear_acceleration.x = imu.SEN.accADC[0] * ACCEL_FACTOR;
  imu_msg.linear_acceleration.y = imu.SEN.accADC[1] * ACCEL_FACTOR;
  imu_msg.linear_acceleration.z = imu.SEN.accADC[2] * ACCEL_FACTOR;
  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;
  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[5] = 0;
  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0.04;

  imu_msg.orientation.w = imu.quat[0];
  imu_msg.orientation.x = imu.quat[1];
  imu_msg.orientation.y = imu.quat[2];
  imu_msg.orientation.z = imu.quat[3];

  imu_msg.orientation_covariance[0] = 0.0025;
  imu_msg.orientation_covariance[1] = 0;
  imu_msg.orientation_covariance[2] = 0;
  imu_msg.orientation_covariance[3] = 0;
  imu_msg.orientation_covariance[4] = 0.0025;
  imu_msg.orientation_covariance[5] = 0;
  imu_msg.orientation_covariance[6] = 0;
  imu_msg.orientation_covariance[7] = 0;
  imu_msg.orientation_covariance[8] = 0.0025;

  imu_pub.publish(&imu_msg);
}
  
/*******************************************************************************
* Publish magnetic sensor data
*******************************************************************************/
void publishMagMsg(void)
{
  #define MAG_FACTOR 15e-8
  mag_msg.header.stamp = nh.now();
  mag_msg.header.frame_id = "magnetic_field_sensor_link";
  mag_msg.magnetic_field.x = imu.SEN.magADC[0] * MAG_FACTOR;
  mag_msg.magnetic_field.y = imu.SEN.magADC[1] * MAG_FACTOR;
  mag_msg.magnetic_field.z = imu.SEN.magADC[2] * MAG_FACTOR;
  mag_msg.magnetic_field_covariance[0] = 0.0048;
  mag_msg.magnetic_field_covariance[1] = 0;
  mag_msg.magnetic_field_covariance[2] = 0;
  mag_msg.magnetic_field_covariance[3] = 0;
  mag_msg.magnetic_field_covariance[4] = 0.0048;
  mag_msg.magnetic_field_covariance[5] = 0;
  mag_msg.magnetic_field_covariance[6] = 0;
  mag_msg.magnetic_field_covariance[7] = 0;
  mag_msg.magnetic_field_covariance[8] = 0.0048;
  mag_pub.publish(&mag_msg);
}
    
/*******************************************************************************
* Publish temperature sensor data
*******************************************************************************/
  float publishRelativeHumidityAmbientTempMsg(void)
  {
    ambient_temp_msg.temperature = dht.readTemperature();
    relative_humidity_msg.relative_humidity = dht.readHumidity();
    ambient_temp_msg.header.stamp = nh.now();
    ambient_temp_msg.header.frame_id = "ambient_temperature_sensor_link";
    relative_humidity_msg.header.stamp = ambient_temp_msg.header.stamp;
    relative_humidity_msg.header.frame_id = "relative_humidity_sensor_link";
    relative_humidity_msg.variance = 0;
    relative_humidity_pub.publish(&relative_humidity_msg);
    ambient_temp_msg.variance = 0;
    ambient_temperature_pub.publish(&ambient_temp_msg);
    return dht.readTemperature();
  }
  
/*******************************************************************************
* Publish battery state data
*******************************************************************************/
  void publishBatteryStateMsg(void)
  {
    battery_state_msg.header.stamp = nh.now();
    battery_state_msg.voltage = getBatteryVoltage(); // in V
    // battery_state_msg.cell_voltage = ["NaN","NaN"]; // in V
    if (batteryIsPresent == true) 
    {
      battery_state_msg.present = true;
      battery_state_msg.current = getBatteryCurrent(); // in A / Negative when discharging
      battery_state_msg.percentage = getBatteryPercentage() ; // between 0 and 1
      battery_state_msg.charge = (battery_state_msg.capacity * battery_state_msg.percentage) ; // current charge in Ah
      battery_state_msg.capacity = 1,6; // in Ah
      battery_state_msg.design_capacity = 1,6; // in Ah
      battery_state_msg.power_supply_status = 2; // 2 = discharging
      battery_state_msg.power_supply_health = 0; // 0 = unknown, 1 = good, 2 = overheat, 3 = dead, 4 = overvoltage, 5 = unspec failure, 6 = cold
      battery_state_msg.power_supply_technology = 3; // 3 = LiPo
      battery_state_msg.location = "robot battery compartment"; //Turnigy 1600mAh 2S 20C Losi Mini SCT Pack (Part LOSB1212)
      battery_state_msg.serial_number ="T1600.2S.20LS"; // 
    }
    else 
    {
      battery_state_msg.present = false;
      battery_state_msg.current = 0; // in A / Negative when discharging
      battery_state_msg.percentage = getBatteryPercentage() ; // between 0 and 1
      battery_state_msg.charge = 0 ; // current charge in Ah
      battery_state_msg.capacity = 0; // in Ah
      battery_state_msg.design_capacity = 0; // in Ah
      battery_state_msg.power_supply_status = 0; // 0 = unknown
      battery_state_msg.power_supply_health = 0; // 0 = unknown, 1 = good, 2 = overheat, 3 = dead, 4 = overvoltage, 5 = unspec failure, 6 = cold
      battery_state_msg.power_supply_technology = 0; // 0 = unknown
      battery_state_msg.location = "NaN"; 
      battery_state_msg.serial_number ="NaN";
    }
    battery_state_pub.publish(&battery_state_msg);
  }
    
/*******************************************************************************
* Battery monitoring and emergency shutdown
*******************************************************************************/
  float getBatteryVoltage(void)
  {
    if (batteryIsPresent == true) 
    {
    //int adc_value;
    //float vol_value;
    //adc_value = analogRead(BDPIN_BAT_PWR_ADC);
    //voltage_value = map(adc_value, 0, 1023, 0, 330*57/10);
    //voltage_value = voltage_value/100;
    float voltage_value = getPowerInVoltage();
    return voltage_value;
    }
    else
    {
      float voltage_value = getPowerInVoltage();
      return voltage_value;
    }
  }
  
  double getBatteryPercentage(void)
  {
    double BatteryPercentage = 0;
    if (batteryIsPresent == true) 
    {
      float x = getBatteryVoltage();
      if (x >= 8.40)
      {
        BatteryPercentage = 1;
      }
      else if (x < 8.40 && x >= 8.20)
      {
        BatteryPercentage = 0.9;
      }
      else if (x < 8.20 && x >= 7.94)
      {
        BatteryPercentage = 0.8;
      }
      else if (x < 7.94 && x >= 7.84)
      {
        BatteryPercentage = 0.7;
      }
      else if (x < 7.84 && x >= 7.74)
      {
        BatteryPercentage = 0.6;
      }
      else if (x < 7.74 && x >= 7.66)
      {
        BatteryPercentage = 0.5;
      }
      else if (x < 7.66 && x >= 7.58)
      {
        BatteryPercentage = 0.4;
      }
      else if (x < 7.58 && x >= 7.50)
      {
        BatteryPercentage = 0.3;
      }
      else if (x < 7.50 && x >= 7.40)
      {
        BatteryPercentage = 0.2;
      }
      else if (x < 7.40 && x >= 7.20)
      {
        BatteryPercentage = 0.1;
      }
      else if (x < 7.20 && x >= 6.60)
      {
        BatteryPercentage = 0.05;
      }
      else
      {
        BatteryPercentage = 0.0;
      }
    }
    else 
    {
      BatteryPercentage = 1;
    }
    return BatteryPercentage;
  }
  
  float getBatteryCurrent(void)
  {
  float mVperAmp = 66; // from the 30A ACS712 datasheet, 66mV per Amp
  float sample = 0.0;
  float sampleArray[150] = {0};
  float median = 0.0;
  float mamps = 0.0;
  float acoffset = 2490; // True Vcc out at 5V pin is 4.98V
  for (int i = 0; i < 150; i++)
    { //Get 150 samples
      sample = analogRead(A0);     //Read current sensor values between 0 and 1023 (1024 possible values)
      sampleArray[i] = sample;  //Add samples to an array
    }
  //
  median = QuickMedian<float>::GetMedian(sampleArray, 150);
  mamps = ((median * (5000 / 1024.0)) - acoffset )/mVperAmp; //((average * (5.0 / 1024.0)) is converitng the read voltage in 0-5 volts
  float mamps_offset = 0; //get 0.000 A when passing to the isBatteryPresent condition
  float amps = 0;
  amps = 0 - (mamps - mamps_offset)/1000; //negative as current is taken from the battery
  return amps;
  }
  
  void batteryMonitor(void)
  {
    if (batteryIsPresent == true)
    {
  if (getBatteryPercentage() > 0.75)
  {
    if (ledManualCtrl==false) {
    setLedStatus("BATTERY_100%");
    emergency_shutdown_msg.data = "";
    emergency_shutdown_pub.publish(&emergency_shutdown_msg);
    }
  }
  if (getBatteryPercentage() > 0.50 && getBatteryPercentage() <= 0.75)
  {
    if (ledManualCtrl==false) {
    setLedStatus("BATTERY_75%");
    emergency_shutdown_msg.data = "";
    emergency_shutdown_pub.publish(&emergency_shutdown_msg);
    }
  }
  if (getBatteryPercentage() > 0.25 && getBatteryPercentage() <= 0.50)
  {
    if (ledManualCtrl==false) {
    setLedStatus("BATTERY_50%");
    emergency_shutdown_msg.data = "";
    emergency_shutdown_pub.publish(&emergency_shutdown_msg);
    }
  }
  if (getBatteryPercentage() > 0.10 && getBatteryPercentage() <= 0.25 &&  battery25Advertised == false)
  {
    if (ledManualCtrl==false) {
    setLedStatus("BATTERY_25%");
    }
    battery25Advertised = true;
    playMelody("LOW_BATTERY");
    emergency_shutdown_msg.data = "";
    emergency_shutdown_pub.publish(&emergency_shutdown_msg);
    
  }
  if (getBatteryVoltage() < 7 && getBatteryVoltage() >= 6.6 && batteryLowAdvertised == false)
  {
    if (ledManualCtrl==false) {
      setLedStatus("BATTERY_VERY_LOW");
    }
    batteryLowAdvertised = true;
    playMelody("LOW_BATTERY");
    emergency_shutdown_msg.data = "";
    emergency_shutdown_pub.publish(&emergency_shutdown_msg);
    
  }
  if (getBatteryVoltage() < 6.6 && batteryErrorAdvertised == false)
  {
    batteryErrorAdvertised = true;
    emergency_shutdown_msg.data = "LOW_BATTERY";
    playMelody("ERROR");
    emergency_shutdown_pub.publish(&emergency_shutdown_msg);
  }
  }
  else
  {
    setLedStatus("NONE");
  }
  
  }
/*******************************************************************************
* Send log message
*******************************************************************************/
void sendLogMsg(void)
{
  static bool log_flag = false;
  char log_msg[100];  

  if (nh.connected())
  {
    if (log_flag == false)
    {      
      sprintf(log_msg, "Initializing opencr_ros node...");
      nh.loginfo(log_msg);
      sprintf(log_msg, "Connecting to OpenCR board...");
      nh.loginfo(log_msg);
      sprintf(log_msg, "Setting up imu publisher with name imu_data and header frame imu_link. Publishing frequency = 50 Hz");
      nh.loginfo(log_msg);
      sprintf(log_msg, "Setting up battery monitoring publisher with name battery_state. Publishing frequency = 10 Hz.");
      nh.loginfo(log_msg);
      sprintf(log_msg, "Setting up magnetic field sensor publisher with name magnetic_field and header frame magnetic_sensor_link. Publishing frequency = 50 Hz.");
      nh.loginfo(log_msg);
      sprintf(log_msg, "Setting up temperature sensor publisher with name ambient_temperature and header frame temperature_sensor_link. Publishing frequency = 0,2 Hz.");
      nh.loginfo(log_msg); 
      sprintf(log_msg, "Setting up relative humidity sensor publisher with name relative_humidity and header frame humidity_sensor_link. Publishing frequency = 0,2 Hz.");
      nh.loginfo(log_msg);
      sprintf(log_msg, "Setting up openCR buttons status publisher with name openCR_buttons_status. Publishing frequency = 5 Hz.");
      nh.loginfo(log_msg);
      sprintf(log_msg, "Setting up emergency shutdown publisher with name emergency_shutdown_request. Publishing frequency = 5 Hz.");
      nh.loginfo(log_msg);
      log_flag = true;

      if (batteryIsPresent == true)
      {
        sprintf(log_msg, "Robot is running from battery. You must carefully monitor battery drain.");
        nh.logwarn(log_msg);
      }
      else if (batteryIsPresent == false)
      {
        sprintf(log_msg, "Robot is running from external power source.");
        nh.loginfo(log_msg);
      }
}
  }
  else
  {
    log_flag = false;
  }
}


/*******************************************************************************
* Setup function
* Run once when the firmware is launched.
*******************************************************************************/
void setup() 
{
  /*******************************************************************************
  * In the Arduino setup function you then need to initialize your ROS node 
  * handle, advertise any topics being published, and subscribe to any topics 
  * you wish to listen to.
  *******************************************************************************/
  nh.initNode();
  nh.getHardware()->setBaud(115200);

  nh.subscribe(buzzer_melody_sub);
  nh.subscribe(leds_status_sub);
  nh.subscribe(set_led_on_sub);
  nh.subscribe(set_led_off_sub);

  nh.advertise(imu_pub);
  nh.advertise(battery_state_pub);
  nh.advertise(mag_pub);
  nh.advertise(ambient_temperature_pub);
  nh.advertise(relative_humidity_pub);
  nh.advertise(openCR_buttons_status_pub);
  nh.advertise(emergency_shutdown_pub);
  sendLogMsg();

  imu.begin();
  dht.begin();
  pinMode(BDPIN_PUSH_SW_1, INPUT);
  pinMode(BDPIN_PUSH_SW_2, INPUT);
  int led_pin_user[4] = { BDPIN_LED_USER_1, BDPIN_LED_USER_2, BDPIN_LED_USER_3, BDPIN_LED_USER_4 };
  pinMode(led_pin_user[0], OUTPUT);
  pinMode(led_pin_user[1], OUTPUT);
  pinMode(led_pin_user[2], OUTPUT);
  pinMode(led_pin_user[3], OUTPUT);

  Serial.begin(9600);
  setup_done = true;
}

/*******************************************************************************
* Loop function to publish data and run the callback functions
*******************************************************************************/
void loop() 
{
  Serial.print("test");
  if (getPowerInVoltage() <= 8.5) // The maximum voltage of the battery is 8.4V.  
  {
     batteryIsPresent = true;
  }
  else if (getPowerInVoltage() > 8.5)
  {
      batteryIsPresent = false;
  }
  if (setup_done == false)
  {
    setup();
  }
  imu.update();
  static uint32_t bat_pre_time;
  static uint32_t temp_humid_pre_time;
  static uint32_t imu_pre_time;
  static uint32_t mag_pre_time;
  static uint32_t but_pre_time;
  bool ledManualCtrl = false;
  
  if ((millis()-bat_pre_time) >= (1000 / BATTERY_PUBLISH_FREQUENCY))
  {
   publishBatteryStateMsg();
   //Serial.println("---Battery current (A)---");
   //Serial.println(getBatteryCurrent(), 5);
   //Serial.println("---Battery voltage (V)---");
   //Serial.println(getBatteryVoltage());
   //Serial.println("---Battery percentage (0-1)---");
   //Serial.println(getBatteryPercentage());
   bat_pre_time = millis();
  }
  if ((millis()-temp_humid_pre_time) >= (1000 / TEMP_HUMID_PUBLISH_FREQUENCY))
  {
   //publishRelativeHumidityAmbientTempMsg();
   Serial.println("---Ambient temperature (°C) ---");
   Serial.println(publishRelativeHumidityAmbientTempMsg());
   temp_humid_pre_time = millis();
  }
  if ((millis()-imu_pre_time) >= (1000 / IMU_PUBLISH_FREQUENCY))
  {
    publishImuMsg();
   // Serial.println("---IMU (velocity (x,y,z) and acceleration (x,y,z))---");
   // TO DO Serial.println(publishImuMsg());
   imu_pre_time=millis();
  }
  if ((millis()-mag_pre_time) >= (1000 / MAG_PUBLISH_FREQUENCY))
  {
    publishMagMsg();
   // Serial.println("---Magnetic field (x,y,z)---");
   // TO DO Serial.println("publishMagMsg()");
   mag_pre_time = millis();
  }
 if ((millis()-but_pre_time) >= (1000 / BUTTONS_PUBLISH_FREQUENCY))
  {
    //publishOpenCRButtonsStatus();
    Serial.println("---OpenCR button---");
    Serial.println(publishOpenCRButtonsStatus());
    but_pre_time=millis();
  }
    
    
    // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();

  // Check battery
  batteryMonitor();

  // When ROS connection is lost, prepare for retrying connection
  if (nh.connected() == false)
  {
    setup_done == false;
  }
}
