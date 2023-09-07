#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_VL53L1X.h>

#include "BMX055.h"
#include "AS5600.h"

// Drive Motor
Servo drive;
const int drvie_phase = 33;
const int drive_enable = 25;
const int drive_min = 500;
const int drive_max = 2400;
int drive_pos = 90;
// Steer Servo
Servo steer;
const int steer_phase = 26;
const int steer_enable = 27;
const int steer_min = 500;
const int steer_max = 2400;
int steer_pos = 90;
// Right ToF Sensor
const uint8_t rtof_address = 0x28;
const int rtof_shut_pin = 5;
Adafruit_VL53L1X right_tof = Adafruit_VL53L1X(rtof_shut_pin);
// Center ToF Sensor
const uint8_t ctof_address = 0x27;
const int ctof_shut_pin = 18;
Adafruit_VL53L1X center_tof = Adafruit_VL53L1X(ctof_shut_pin);
// Left ToF Sensor
const uint8_t ltof_address = 0x26;
const int ltof_shut_pin = 19;
Adafruit_VL53L1X left_tof = Adafruit_VL53L1X(ltof_shut_pin);


void setup() {
  // debug
  Serial.begin(115200);

  // Jetson interface
  Serial2.begin(115200);

  // Drive motor initialize
  pinMode(drvie_phase, OUTPUT);
  digitalWrite(drvie_phase, LOW);
  drive.setPeriodHertz(50);
  drive.attach(drive_enable, drive_min, drive_max);

  // Steer servo initialize
  pinMode(steer_phase, OUTPUT);
  digitalWrite(steer_phase, LOW);
  steer.setPeriodHertz(50);
  steer.attach(steer_enable, steer_min, steer_max);

  // I2C interface initialize
  Wire.begin();
  Wire.setClock(400000);

  // IMU initialize
  BMX055_init(&Wire);

  // Encoder initialize
  AS5600_init(&Wire);

  // ToF initlialize
  right_tof.VL53L1X_Off();
  center_tof.VL53L1X_Off();
  left_tof.VL53L1X_Off();

  if(!right_tof.begin(rtof_address, &Wire))
  {
    Serial.println("right tof initialize error.");
    while(1) { }
  }
  Serial.print("right tof id: "); Serial.println(right_tof.sensorID(), HEX);

  if(!center_tof.begin(ctof_address, &Wire))
  {
    Serial.println("center tof initialize error.");
    while(1) { }
  }
  Serial.print("center tof id: "); Serial.println(center_tof.sensorID(), HEX);

  if(!left_tof.begin(ltof_address, &Wire))
  {
    Serial.println("left tof initialize error.");
    while(1) { }
  }
  Serial.print("left tof id: "); Serial.println(left_tof.sensorID(), HEX);


  if(!right_tof.startRanging()){
    Serial.println("right tof can not start ranging.");
    while(1){ }
  }
  right_tof.setTimingBudget(100);  

  if(!center_tof.startRanging()){
    Serial.println("center tof can not start ranging.");
    while(1){ }
  }
  center_tof.setTimingBudget(100);

  if(!left_tof.startRanging()){
    Serial.println("left tof can not start ranging.");
    while(1){ }
  }
  left_tof.setTimingBudget(100);  

}

void loop() {

  Serial2.println("adrc com");

  int count = 0;
  int buffer[65];
  while(Serial.available()){
    int key = Serial.read();

    buffer[count] = key;
    count++;
  }

  switch(count)
  {
    case 1:
      break;
    case 3:
      if(buffer[0] == 27 && buffer[1] == 91 && buffer[2] == 65)
      { 
        drive_pos += 1;
      }
      else if(buffer[0] == 27 && buffer[1] == 91 && buffer[2] == 66)
      { 
        drive_pos -= 1;
      }
      else if(buffer[0] == 27 && buffer[1] == 91 && buffer[2] == 67)
      { 
        steer_pos += 10;
      }
      else if(buffer[0] == 27 && buffer[1] == 91 && buffer[2] == 68)
      { 
        steer_pos -= 10;
      }

      break;
  }


  drive.write(drive_pos);
  steer.write(steer_pos);

  float acc[3];
  BMX055_acc(acc);

  float gyro[0];
  BMX055_gyro(gyro);

  float mag[0];
  BMX055_mag(mag);

  uint16_t angle = AS5600_angle();

  int16_t right_range = -1;
  if(right_tof.dataReady())
  {
    right_range = right_tof.distance();
  }

  int16_t center_range = -1;
  if(center_tof.dataReady())
  {
    center_range = center_tof.distance();
  }

  int16_t left_range = -1;
  if(left_tof.dataReady())
  {
    left_range = left_tof.distance();
  }

  // display information
  Serial.print("drive pos: "); Serial.print(drive_pos);
  Serial.print("; steer pos: "); Serial.print(steer_pos);
  Serial.println();

  Serial.print("acc: ");
  Serial.print(acc[0]); Serial.print(", ");
  Serial.print(acc[1]); Serial.print(", ");
  Serial.print(acc[2]); Serial.println(" [m/s]");

  Serial.print("gyr: ");
  Serial.print(gyro[0]); Serial.print(", ");
  Serial.print(gyro[1]); Serial.print(", ");
  Serial.print(gyro[2]); Serial.println(" [deg/s]");

  Serial.print("mag: ");
  Serial.print(mag[0]); Serial.print(", ");
  Serial.print(mag[1]); Serial.print(", ");
  Serial.print(mag[2]); Serial.println();

  Serial.print("ang: ");
  Serial.print(angle); Serial.println();

  Serial.print("r c l: ");
  Serial.print(right_range); Serial.print(", ");  
  Serial.print(center_range); Serial.print(", ");
  Serial.print(left_range); 
  Serial.println();

  delay(100);
}

