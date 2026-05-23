#include <Arduino.h>
#include <Wire.h>
#include "Config.h"
#include "MotorController.h"
#include "IMUManager.h"
#include "LineSensor.h"
#include "LineFollower.h"

MotorController motors;
IMUManager imu;
LineSensor sensors;
LineFollower fireball(motors, imu, sensors);

void setup() {
  Serial.begin(9600);
  delay(100);

  Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
  Wire.setClock(Config::I2C_SPEED_HZ);

  if (!imu.begin()) {
    Serial.println("Critical Error! BMI160 not detected!");
    while(1);
  }

  fireball.begin();
  Serial.println("Firmware Fireball v0.02 initiated successfully");
}

void loop() {
  imu.update();
  fireball.update();
}