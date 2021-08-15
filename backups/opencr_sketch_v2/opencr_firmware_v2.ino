/*******************************************************************************
* Copyright 2019 Rodolphe MATIAS DE SOUSA.
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

/* Version : 20191201_v2 */

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
#include <IMU.h>
#include <dht.h>
#include <Arduino.h>

/*******************************************************************************
* ROS NodeHandle
* Next, we need to instantiate the node handle, which allows our program to 
* create publishers and subscribers. The node handle also takes care of serial 
* port communications.
*******************************************************************************/
ros::NodeHandle nh;


/*******************************************************************************
* Define hardware map. 
* On a next version, this will be done automaticaly using the hardware_map ros 
* topic.
*******************************************************************************/
byte push_button_1 = 34;
byte push_button_2 = 35;
byte user_led_1 = 22;
byte user_led_2 = 23;
byte  user_led_3 = 24;
byte  user_led_4 = 25;
byte  buzzer = 31;
byte  temperature_sensor = 0;
byte  humidity_sensor = 0;
byte current_sensor = 2;
  
/*******************************************************************************
* Callback function for led control
* Possible led status : BATTERY_100%, BATTERY_75%, BATTERY_50%, BATTERY_25%, 
* BATTERY_VERY_LOW, BOOT, SEQUENCE
* Manually controling leds 
*******************************************************************************/
// User leds status : BATTERY_100%, BATTERY_75%, BATTERY_50%, BATTERY_25%, BOOT, SEQUENCE 
void setLedStatus(const String& ledStatus)
{
  if (ledStatus == "SEQUENCE") 
  {
    int led_pin_user[4] = { user_led_1, user_led_2, user_led_3, user_led_4 };
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
    int led_pin_user[4] = { user_led_1, user_led_2, user_led_3, user_led_4 };
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
    int led_pin_user[4] = { user_led_1, user_led_2, user_led_3, user_led_4 };
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
    int led_pin_user[4] = { user_led_1, user_led_2, user_led_3, user_led_4 };
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
        int led_pin_user[4] = { user_led_1, user_led_2, user_led_3, user_led_4 };
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
        int led_pin_user[4] = { user_led_1, user_led_2, user_led_3, user_led_4 };
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
        if (ledStatus == "BATTERY_VERY_LOW") {
        int led_pin_user[4] = { user_led_1, user_led_2, user_led_3, user_led_4 };
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
}
void setLedOnCallback(const std_msgs::Byte& setLedOn_msg)
{
  turnLedOn(setLedOn_msg);
}
void setLedOffCallback(const std_msgs::Byte& setLedOff_msg)
{
  turnLedOff(setLedOff_msg);
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
     tone(buzzer, note[thisNote], noteDuration);
     // to distinguish the notes, set a minimum time between them.
     // the note's duration + 30% seems to work well:
     int pauseBetweenNotes = noteDuration * 1.30;
     delay(pauseBetweenNotes);
     // stop the tone playing:
     noTone(buzzer);
    }
   }
   // Define melodies 
   void playMelody(const String& melodyName)
   {
     // define specific notes
     const uint16_t NOTE_C4 = 262;
     const uint16_t NOTE_D4 = 294;
     const uint16_t NOTE_E4 = 330;
     const uint16_t NOTE_F4 = 349;
     const uint16_t NOTE_G4 = 392;
     const uint16_t NOTE_A4 = 440;
     const uint16_t NOTE_B4 = 494;
     const uint16_t NOTE_C5 = 523;
     const uint16_t NOTE_C6 = 1047;    
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
       byte note_num = 1;
       uint16_t note[note_num] = {0, 0};
       uint8_t  duration[note_num] = {0, 0};
       note[0] = NOTE_C5;   duration[0] = 4;
       melodyPlayerFunction(note, note_num, duration);
       }
       if (melodyName ==  "BUTTON2") {
       byte note_num = 1;
       uint16_t note[note_num] = {0, 0};
       uint8_t  duration[note_num] = {0, 0};
       note[0] = NOTE_C5;   duration[0] = 4;
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
ros::Publisher imu_pub("imu", &imu_msg);

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
* Publish OpenCR buttons status
*******************************************************************************/
void publishOpenCRButtonsStatus(void)
{

  uint8_t reading = 0;
  static uint32_t pre_time;
  if (digitalRead(push_button_1) == HIGH)
  {
    playMelody("BUTTON1");
    reading |= 0x01;
  }
  if (digitalRead(push_button_2) == HIGH)
  {
    playMelody("BUTTON2");
    reading |= 0x02;
  }
  if (digitalRead(push_button_1) == HIGH && digitalRead(push_button_2) == HIGH)
    {
     playMelody("BUTTON1");
     playMelody("BUTTON2");
     reading |= 0x03;
    }

  if (millis()-pre_time >= 50)
  {
    openCR_buttons_status_msg.data = reading;
    openCR_buttons_status_pub.publish(&openCR_buttons_status_msg);
    pre_time = millis();
  }
}

/*******************************************************************************
* Publish IMU data : angular velocity, linear acceleration, orientation
*******************************************************************************/
void publishImuMsg(void)
{
  cIMU IMU;
  IMU.update();
  #define ACCEL_FACTOR 0.000598550415; // (ADC_Value / Scale) * 9.80665 => Range : +- 2[g] Scale : +- 16384
  #define GYRO_FACTOR 0.0010642; // (ADC_Value/Scale) * (pi/180) => Range : +- 2000[deg/s] Scale : +- 16.4[deg/s]
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = "imu_corrected_data";

  imu_msg.angular_velocity.x = IMU.SEN.gyroADC[0] * GYRO_FACTOR;
  imu_msg.angular_velocity.y = IMU.SEN.gyroADC[1] * GYRO_FACTOR;
  imu_msg.angular_velocity.z = IMU.SEN.gyroADC[2] * GYRO_FACTOR;
  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;
  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[5] = 0;
  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0.02;

  imu_msg.linear_acceleration.x = IMU.SEN.accADC[0] * ACCEL_FACTOR;
  imu_msg.linear_acceleration.y = IMU.SEN.accADC[1] * ACCEL_FACTOR;
  imu_msg.linear_acceleration.z = IMU.SEN.accADC[2] * ACCEL_FACTOR;
  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;
  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[5] = 0;
  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0.04;

  imu_msg.orientation.w = IMU.quat[0];
  imu_msg.orientation.x = IMU.quat[1];
  imu_msg.orientation.y = IMU.quat[2];
  imu_msg.orientation.z = IMU.quat[3];

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
  cIMU IMU;
  IMU.update();
  #define MAG_FACTOR 15e-8;
  mag_msg.header.stamp = nh.now();
  mag_msg.header.frame_id = "mag_corrected_data";
  mag_msg.magnetic_field.x = IMU.SEN.magADC[0] * MAG_FACTOR;
  mag_msg.magnetic_field.y = IMU.SEN.magADC[1] * MAG_FACTOR;
  mag_msg.magnetic_field.z = IMU.SEN.magADC[2] * MAG_FACTOR;
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
void publishAmbiantTempMsg(void)
{
  dht DHT;
  byte DHT_PIN = temperature_sensor;
  int readData = DHT.read11(DHT_PIN);
  ambiant_temp_msg.temperature = DHT.temperature;
  ambiant_temperature_pub.publish(&ambiant_temp_msg);
  }

/*******************************************************************************
* Publish humidity sensor data
*******************************************************************************/
void publishRelativeHumidityMsg(void)
{
  dht DHT;
  byte DHT_PIN = humidity_sensor;
  int readData = DHT.read11(DHT_PIN);
  relative_humidity_msg.relative_humidity = DHT.humidity;
  relative_humidity_pub.publish(&relative_humidity_msg);
}
  
/*******************************************************************************
* Publish battery state data
*******************************************************************************/
  void publishBatteryStateMsg(void)
  {
    battery_state_msg.header.stamp = nh.now();
    battery_state_msg.voltage = getBatteryVoltage(); // in V
    // battery_state_msg.cell_voltage = ["NaN","NaN"]; // in V
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
    if (getBatteryCurrent==0)
      battery_state_msg.present = false;
    else
      battery_state_msg.present = true;
    battery_state_pub.publish(&battery_state_msg);
    }
    
/*******************************************************************************
* Battery monitoring and emergency shutdown
*******************************************************************************/
  float getBatteryVoltage()
  {
    //float voltage_value = getPowerInVoltage(); //get the board power voltage either battery, usb or external
    float adc_value;
    float voltage_value;
    adc_value = analogRead(BDPIN_BAT_PWR_ADC);
    voltage_value = map(adc_value, 0, 1023, 0, 330*57/10);
    voltage_value = voltage_value/100;
    return voltage_value;
  }
  
  float getBatteryPercentage()
  {
    float BatteryPercentage = (1 - (8.4 - getBatteryVoltage()));
    return BatteryPercentage;
  }
  
  float getBatteryCurrent()
  {
  float mVperAmp = 66; // from the 30A ACS712 datasheet, 66mV per Amp
  float amps = 0;// Current measuring
  float voltage;
  float analogReading = analogRead(current_sensor);
  voltage = ((analogReading / 1024) * 5000); // Gets you mV
  amps = 0 - ((voltage - 2500) / mVperAmp); //in A, 2500 is the ACSoffset, negative because battery is discharging
  return amps;
  }
  
  void batteryMonitor(void)
  {
  if (getBatteryCurrent() > 0 && getBatteryPercentage() > 0.75)
  {
    setLedStatus("BATTERY_100%");
  }
  if (getBatteryCurrent() > 0 && getBatteryPercentage() > 0.50 && getBatteryPercentage() <= 0.75)
  {
    setLedStatus("BATTERY_75%");
  }
  if (getBatteryCurrent() > 0 && getBatteryPercentage() > 0.25 && getBatteryPercentage() <= 0.50)
  {
    setLedStatus("BATTERY_50%");
  }
  if (getBatteryCurrent() > 0 && getBatteryPercentage() > 0.10 && getBatteryPercentage() <= 0.25)
  {
    setLedStatus("BATTERY_25%");
    playMelody("LOW_BATTERY");
  }
  if (getBatteryCurrent() > 0 && getBatteryVoltage() > 6.6 && getBatteryVoltage() < 7)
  {
    setLedStatus("BATTERY_VERY_LOW");
    playMelody("LOW_BATTERY");
  }
  if (getBatteryCurrent() > 0 && getBatteryVoltage() > 1 && getBatteryVoltage() < 6.6)
  {
    playMelody("ERROR");
    emergency_shutdown_msg.data = "LOW_BATTERY";
    emergency_shutdown_pub.publish(&emergency_shutdown_msg);
    delay(500);
  }
  }
  
/*******************************************************************************
* Function prototypes
*******************************************************************************/
// void buzzerMelodyCallback(const std_msgs::String& buzzerMelody_msg);
// void ledsStatusCallback(const std_msgs::String& ledsStatus_msg);
// void setLedOnCallback(const std_msgs::Byte& setLedOn_msg);
// void setLedOffCallback(const std_msgs::Byte& setLedOff_msg);
// void publishOpenCRButtonsStatus(void);
// void publishImuMsg(void);
// void publishMagMsg(void);
// void publishBatteryStateMsg(void);
// void publishAmbiantTempMsg(void);
// void publishRelativeHumidityMsg(void);
// void batteryMonitor(void);

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
  // nh.subscribe(hardware_map_sub);

  nh.advertise(imu_pub);
  nh.advertise(battery_state_pub);
  nh.advertise(mag_pub);
  nh.advertise(ambiant_temperature_pub);
  nh.advertise(relative_humidity_pub);
  nh.advertise(openCR_buttons_status_pub);
  nh.advertise(emergency_shutdown_pub);

  cIMU IMU;
  IMU.begin();

  pinMode(push_button_1, INPUT);
  pinMode(push_button_2, INPUT);
  pinMode(user_led_1, OUTPUT);
  pinMode(user_led_2, OUTPUT);
  pinMode(user_led_3, OUTPUT);
}


/*******************************************************************************
* Loop function to publish data and run the callback functions
*******************************************************************************/
void loop() 
{
  static uint32_t pre_time;
  if (millis()-pre_time >= 50)
  {
    publishOpenCRButtonsStatus();
    publishImuMsg();
    publishMagMsg();
    publishBatteryStateMsg();
    publishAmbiantTempMsg();
    publishRelativeHumidityMsg();
    batteryMonitor();
    pre_time = millis();
  }
  
    // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();
}
