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
  Serial.begin(115200);
  delay(100);

  Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
  Wire.setClock(Config::I2C_SPEED_HZ);

  if (!imu.begin()) {
    Serial.println("Critical Error! BMI160 not detected!");
    while(1);
  }

  Serial.println("Firmware Fireball v0.05 initiated successfully");

  Serial.println("Place car on the line! Starting in 3 seconds...");
  delay(3000);

  fireball.begin();
  Serial.println("GO!");
}

void loop() {
  static unsigned long last_loop = 0;
  
  if (micros() - last_loop >= 2000) { // Fixa o loop em 500Hz (2ms)
    last_loop = micros();
    
    imu.update();
    fireball.update();
  }
  
  yield();
}