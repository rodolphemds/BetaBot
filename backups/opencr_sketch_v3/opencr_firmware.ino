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

#include "opencr_firmware_config.h"


/*******************************************************************************
* Setup function
* Run once when the firmware is launched.
*******************************************************************************/
void setup() {
  
 DEBUG_SERIAL.begin(57600);

  // Initialize ROS node handle, advertise any topics beings published and subscribe to the topics you wish to listen to
  nh.initNode();
  nh.getHardware()->setBaud(115200);

  nh.subscribe(buzzer_melody_sub);
  nh.subscribe(leds_status_sub);
  nh.subscribe(set_led_on_sub);
  nh.subscribe(set_led_off_sub);
  
  nh.advertise(imu_pub);
  nh.advertise(battery_state_pub);
  nh.advertise(mag_pub);
  nh.advertise(ambiant_temperature_pub);
  nh.advertise(relative_humidity_pub);
  nh.advertise(openCR_buttons_status_pub);
  nh.advertise(emergency_shutdown_pub);
  
  char log_msg[100];

   // Setting for IMU
  sensors.init();

  unsigned long prev_update_time;
  prev_update_time = millis();

  pinMode(PUSH_BUTTON_1, INPUT);
  pinMode(PUSH_BUTTON_2, INPUT);
  pinMode(USER_LED_1, OUTPUT);
  pinMode(USER_LED_2, OUTPUT);
  pinMode(USER_LED_3, OUTPUT);
  pinMode(LED_WORKING_CHECK, OUTPUT);
  
  setup_end = true;
}


/*******************************************************************************
* Loop function
*******************************************************************************/
void loop() 
{
  uint32_t t = millis();
  updateTime();
  updateVariable(nh.connected());

 if ((t-tTime[0]) >= (1000 / BATTERY_PUBLISH_FREQUENCY))
  {
    publishBatteryStateMsg();
    tTime[0] = t;
  }
  if ((t-tTime[1]) >= (1000 / TEMP_PUBLISH_FREQUENCY))
  {
    publishAmbiantTempMsg();
    tTime[1] = t;
  }
  if ((t-tTime[2]) >= (1000 / HUMID_PUBLISH_FREQUENCY))
  {
    publishRelativeHumidityMsg();
    tTime[2] = t;
  }
  if ((t-tTime[3]) >= (1000 / IMU_PUBLISH_FREQUENCY))
  {
    publishImuMsg();
    tTime[3] = t;
  }
  if ((t-tTime[4]) >= (1000 / MAG_PUBLISH_FREQUENCY))
  {
    publishMagMsg();
    tTime[4] = t;
  }
 if ((t-tTime[5]) >= (1000 / BUTTONS_PUBLISH_FREQUENCY))
  {
    publishOpenCRButtonsStatus();
    tTime[5] = t;
  }
  
 // Send log message after ROS connection
  sendLogMsg();

  // Update the IMU unit
  sensors.updateIMU();

  // Start Gyro Calibration after ROS connection
  updateGyroCali(nh.connected());

  // Monitor battery state
  batteryMonitor();

  // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();

  // Wait the serial link time to process
  waitForSerialLink(nh.connected());

}


/*******************************************************************************
* Buzzer functions
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
     tone(BUZZER, note[thisNote], noteDuration);
     // to distinguish the notes, set a minimum time between them.
     // the note's duration + 30% seems to work well:
     int pauseBetweenNotes = noteDuration * 1.30;
     delay(pauseBetweenNotes);
     // stop the tone playing:
     noTone(BUZZER);
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

/*******************************************************************************
* OpenCR led control
*******************************************************************************/
// User leds status : BATTERY_100%, BATTERY_75%, BATTERY_50%, BATTERY_25%, BOOT, SEQUENCE 
void setLedStatus(const String& ledStatus)
{
  if (ledStatus == "SEQUENCE") 
  {
    int led_pin_user[4] = { USER_LED_1, USER_LED_2, USER_LED_3, USER_LED_4 };
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
    int led_pin_user[4] = { USER_LED_1, USER_LED_2, USER_LED_3, USER_LED_4 };
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
    int led_pin_user[4] = { USER_LED_1, USER_LED_2, USER_LED_3, USER_LED_4 };
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
    int led_pin_user[4] = { USER_LED_1, USER_LED_2, USER_LED_3, USER_LED_4 };
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
        int led_pin_user[4] = { USER_LED_1, USER_LED_2, USER_LED_3, USER_LED_4 };
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
        int led_pin_user[4] = { USER_LED_1, USER_LED_2, USER_LED_3, USER_LED_4 };
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
        int led_pin_user[4] = { USER_LED_1, USER_LED_2, USER_LED_3, USER_LED_4 };
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
  float analogReading = analogRead(CURRENT_SENSOR);
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
    char log_msg[100];
            sprintf(log_msg, "Battery < 75%.");
      nh.logwarn(log_msg);
  }
  if (getBatteryCurrent() > 0 && getBatteryPercentage() > 0.25 && getBatteryPercentage() <= 0.50)
  {
    setLedStatus("BATTERY_50%");
    char log_msg[100];
            sprintf(log_msg, "Battery < 50%.");
      nh.logwarn(log_msg);
  }
  if (getBatteryCurrent() > 0 && getBatteryPercentage() > 0.10 && getBatteryPercentage() <= 0.25)
  {
    setLedStatus("BATTERY_25%");
    playMelody("LOW_BATTERY");
    char log_msg[100];
        sprintf(log_msg, "Battery < 25%.");
      nh.logwarn(log_msg);
  }
  if (getBatteryCurrent() > 0 && getBatteryVoltage() > 6.6 && getBatteryVoltage() < 7)
  {
    setLedStatus("BATTERY_VERY_LOW");
    playMelody("LOW_BATTERY");
    char log_msg[100];
        sprintf(log_msg, "Low battery.");
      nh.logwarn(log_msg);
  }
  if (getBatteryCurrent() > 0 && getBatteryVoltage() > 1 && getBatteryVoltage() < 6.6)
  {
    playMelody("ERROR");
    emergency_shutdown_msg.data = "LOW_BATTERY";
    emergency_shutdown_pub.publish(&emergency_shutdown_msg);
    char log_msg[100];
    sprintf(log_msg, "Emergency shutdown required to preserve battery health.");
      nh.logwarn(log_msg);
    delay(500);
  }
  }
/*******************************************************************************
* Callback function for sound and melody msgs
* Possible melodies : BOOT, SHUTDOWN, LOW_BATTERY, ERROR, BUTTON1, BUTTON2,
* DEFAULT
*******************************************************************************/

void buzzerMelodyCallback(const std_msgs::String& buzzerMelody_msg)
{
  playMelody(buzzerMelody_msg.data);
}

/*******************************************************************************
* Callback functions for led control 
* Possible led status : BATTERY_100%, BATTERY_75%, BATTERY_50%, BATTERY_25%, 
* BATTERY_VERY_LOW, BOOT, SEQUENCE
* Manually controling leds turning them on and off by sending their number
*******************************************************************************/
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
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg = sensors.getIMU();

  imu_msg.header.stamp    = rosNow();
  imu_msg.header.frame_id = "imu_link";

  imu_pub.publish(&imu_msg);
}

/*******************************************************************************
* Publish msgs (Magnetic data)
*******************************************************************************/
void publishMagMsg(void)
{
  mag_msg = sensors.getMag();
  mag_msg.header.stamp    = rosNow();
  mag_msg.header.frame_id = "magnetic_sensor_link";

  mag_pub.publish(&mag_msg);
}

/*******************************************************************************
* Publish msgs (Temperature data)
*******************************************************************************/
void publishAmbiantTempMsg(void)
{
  dht DHT;
  byte DHT_PIN = TEMPERATURE_SENSOR;
  int readData = DHT.read11(DHT_PIN);
  ambiant_temp_msg.header.stamp = rosNow();
  ambiant_temp_msg.header.frame_id = "temperature_sensor_link";
  ambiant_temp_msg.temperature = DHT.temperature;
  ambiant_temperature_pub.publish(&ambiant_temp_msg);
  }

/*******************************************************************************
* Publish msgs (Relative humidity data)
*******************************************************************************/
void publishRelativeHumidityMsg(void)
{
  dht DHT;
  byte DHT_PIN = HUMIDITY_SENSOR;
  int readData = DHT.read11(DHT_PIN);
  relative_humidity_msg.header.stamp = rosNow();
  relative_humidity_msg.header.frame_id = "humidity_sensor_link";
  relative_humidity_msg.relative_humidity = DHT.humidity;
  relative_humidity_pub.publish(&relative_humidity_msg);
}

/*******************************************************************************
* Publish msgs (Battery state data)
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
* Publish msg (Button state)
*******************************************************************************/
void publishOpenCRButtonsStatus(void)
{

  uint8_t reading = 0;
  static uint32_t pre_time;
  if (digitalRead(PUSH_BUTTON_1) == HIGH)
  {
    playMelody("BUTTON1");
    reading |= 0x01;
  }
  if (digitalRead(PUSH_BUTTON_2) == HIGH)
  {
    playMelody("BUTTON2");
    reading |= 0x02;
  }
  if (digitalRead(PUSH_BUTTON_1) == HIGH && digitalRead(PUSH_BUTTON_2) == HIGH)
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
* Update variable (initialization)
*******************************************************************************/
void updateVariable(bool isConnected)
{
  static bool variable_flag = false;
  
  if (isConnected)
  {
    if (variable_flag == false)
    {      
      sensors.initIMU();

      variable_flag = true;
    }
  }
  else
  {
    variable_flag = false;
  }
}

/*******************************************************************************
* Wait for Serial Link
*******************************************************************************/
void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false;
  
  if (isConnected)
  {
    if (wait_flag == false)
    {      
      delay(10);

      wait_flag = true;
    }
  }
  else
  {
    wait_flag = false;
  }
}

/*******************************************************************************
* Update the base time for interpolation
*******************************************************************************/
void updateTime()
{
  current_offset = millis();
  current_time = nh.now();
}

/*******************************************************************************
* ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return nh.now();
}

/*******************************************************************************
* Time Interpolation function (deprecated)
*******************************************************************************/
ros::Time addMicros(ros::Time & t, uint32_t _micros)
{
  uint32_t sec, nsec;

  sec  = _micros / 1000 + t.sec;
  nsec = _micros % 1000000000 + t.nsec;

  return ros::Time(sec, nsec);
}

/*******************************************************************************
* Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(bool isConnected)
{
  static bool isEnded = false;
  char log_msg[50];

  (void)(isConnected);

  if (nh.connected())
  {
    if (isEnded == false)
    {
      sprintf(log_msg, "Calibrating gyroscope. Please do not move the robot.");
      nh.loginfo(log_msg);

      sensors.calibrationGyro();

      sprintf(log_msg, "Gyroscope calibrated.");
      nh.loginfo(log_msg);

      isEnded = true;
    }
  }
  else
  {
    isEnded = false;
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
      sprintf(log_msg, "Setting up IMU publisher with name imu_data and header frame imu_link. Publishing frequency = 50 Hz");
      nh.loginfo(log_msg);
      sprintf(log_msg, "Setting up battery monitoring publisher with name battery_state. Publishing frequency = 10 Hz.");
      nh.loginfo(log_msg);
      sprintf(log_msg, "Setting up magnetic field sensor publisher with name magnetic_field and header frame magnetic_sensor_link. Publishing frequency = 50 Hz.");
      nh.loginfo(log_msg);
      sprintf(log_msg, "Setting up temperature sensor publisher with name ambiant_temperature and header frame temperature_sensor_link. Publishing frequency = 0,2 Hz.");
      nh.loginfo(log_msg); 
      sprintf(log_msg, "Setting up relative humidity sensor publisher with name relative_humidity and header frame humidity_sensor_link. Publishing frequency = 0,2 Hz.");
      nh.loginfo(log_msg);
      sprintf(log_msg, "Setting up openCR buttons status publisher with name openCR_buttons_status. Publishing frequency = 5 Hz.");
      nh.loginfo(log_msg);
      sprintf(log_msg, "Setting up emergency shutdown publisher with name emergency_shutdown_request. Publishing frequency = 5 Hz.");
      nh.loginfo(log_msg);
      log_flag = true;
}
  }
  else
  {
    log_flag = false;
  }
}
