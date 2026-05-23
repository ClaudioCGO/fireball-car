/* 
Fireball robot firmware

Author: jhgpereira
Version: 0.01 alpha
Date: 21/05/2026
Description: Implementation of basic functions for Fireball line follower 
robot, including:
- H Bridge pin definition and logical functions;
- DC Motors controlls;
- IR sensor reading;
- Gy BMI 160 Reading;
*/

/*
//--//--//--//--//--//
//      IMPORTS     //
//--//--//--//--//--//


#include <Wire.h>
#include <BMI160Gen.h>


//--//--//--//--//--//
//     CONSTANTS    //
//--//--//--//--//--//


// H BRIDGE
const uint8_t STBY_PIN = 3;
const uint8_t AIN1_PIN = 21;
const uint8_t AIN2_PIN = 20;
const uint8_t BIN1_PIN = 1; // this and the bellow pin are physically switched.
const uint8_t BIN2_PIN = 2;
const uint8_t PWMA_PIN = 10;
const uint8_t PWMB_PIN = 0;

// I2C
const uint8_t SDA_PIN = 8;
const uint8_t SCL_PIN = 9;

// BMI160
const int i2c_addr = 0x69;

// IR SENSORS
uint8_t LE_IR_PIN = 5;
uint8_t LI_IR_PIN = 6;
uint8_t RI_IR_PIN = 7;
uint8_t RE_IR_PIN = 4;


//--//--//--//--//--//
//     VARIABLES    //
//--//--//--//--//--//


// I2C
uint8_t i2c_hz = 100000;


//--//--//--//--//--//
//      SETUP       //
//--//--//--//--//--//


void setup() {
  
  // Start Serial Communication
  Serial.begin(9600);
  delay(100);

  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Setuo I/O of H bridge
  hbSetup();

  // Initialize the BMI module
  if (!BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr)) {
    Serial.println("BMI160 initialization failed!");
    while (1); // Halt if initialization fails
  }
 
  Serial.println("BMI160 initialized successfully in I2C mode!");
}


//--//--//--//--//--//
//       LOOP       //
//--//--//--//--//--//


void loop() {
  int gx, gy, gz; // Raw gyroscope values
  int ax, ay, az; // Raw accelerometer values
 
  // Read raw gyroscope measurements from the BMI160
  BMI160.readGyro(gx, gy, gz);
 
  // Read raw accelerometer measurements from the BMI160
  BMI160.readAccelerometer(ax, ay, az);
 
  // Display the gyroscope values (X, Y, Z) on the Serial Monitor
  Serial.print("Gyroscope Data (X, Y, Z): ");
  Serial.print(gx);
  Serial.print(", ");
  Serial.print(gy);
  Serial.print(", ");
  Serial.println(gz);
 
  // Display the accelerometer values (X, Y, Z) on the Serial Monitor
  Serial.print("Accelerometer Data (X, Y, Z): ");
  Serial.print(ax);
  Serial.print(", ");
  Serial.print(ay);
  Serial.print(", ");
  Serial.println(az);
 
  delay(2000); // Delay for readability (500ms)
}


//--//--//--//--//--//
//      HELPERS     //
//--//--//--//--//--//


// Define HB Pin modes
void hbSetup(){
  pinMode(STBY_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);
}


// Active the motors output
void hbSActiveMode(){
  digitalWrite(STBY_PIN, HIGH);
}


// Deactive the motors output
void hbStandbyMode(){
  digitalWrite(STBY_PIN, LOW);
}


// Motor A rotate clockwise
void MAClockwiseMode(){
  digitalWrite(AIN1_PIN, HIGH);
  digitalWrite(AIN2_PIN, LOW);
}


// Motor B rotate clockwise
void MBClockwiseMode(){
  digitalWrite(BIN1_PIN, HIGH);
  digitalWrite(BIN2_PIN, LOW);
}


// Motor A rotate counter-clockwise
void MACounterClockwiseMode(){
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(AIN2_PIN, HIGH);
}


// Motor B rotate counter-clockwise
void MBCounterClockwiseMode(){
  digitalWrite(BIN1_PIN, LOW);
  digitalWrite(BIN2_PIN, HIGH);
}


// Control motor A pwm with an normalised value
void MApwm (float coefficient){
  int pwma_value = constrain(round(coefficient * 255), 0, 255); 
  analogWrite(PWMA_PIN, pwma_value);
}


// Control motor B pwm with an normalised value
void MBpwm (float coefficient){
  int pwmb_value = constrain(round(coefficient * 255), 0, 255); 
  analogWrite(PWMB_PIN, pwmb_value);
}


// Stop Motor A
void StopMA(){
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(AIN1_PIN, LOW);
}


// Stop Motor B
void StopMB(){
  digitalWrite(BIN1_PIN, LOW);
  digitalWrite(BIN1_PIN, LOW);
}


// Brake Motor A
void BrakeMA(){
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(AIN1_PIN, LOW);
}


// Brake Motor B
void BrakeMB(){
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(AIN1_PIN, LOW);
}


// Stop Motors
void StopRun(){
  StopMA();
  StopMB();
}


// Brake the Motors
void Brake(){
  BrakeMA();
  BrakeMB();
}


// Motors rotate clockwise
void RunForward(){
  MAClockwiseMode();
  MBClockwiseMode();
};


// Motors rotate counter-clockwise
void RunBackward(){
  MACounterClockwiseMode();
  MBCounterClockwiseMode();
};


// Motor A rotate clockwise and motor B rotate counter-clockwise
void RotateToRight(){
  MAClockwiseMode();
  MBCounterClockwiseMode();
}


// Motor A rotate counter-clockwise and motor B rotate clockwise
void RotateToLeft(){
  MACounterClockwiseMode();
  MBClockwiseMode();
}


//--//--//--//--//--//
//     FUNCTIONS    //
//--//--//--//--//--//


// Time count for testing
void TestingCount(){
  Serial.printf("Testing function in...\n");
  delay(1000);
  Serial.printf("1");
  delay(300);
  Serial.printf(".");
  delay(300);
  Serial.printf(".");
  delay(300);
  Serial.printf(".");
  delay(100);
  Serial.printf("2");
  delay(300);
  Serial.printf(".");
  delay(300);
  Serial.printf(".");
  delay(300);
  Serial.printf(".");
  delay(100);
  Serial.printf("3");
  delay(300);
  Serial.printf(".");
  delay(300);
  Serial.printf(".");
  delay(300);
  Serial.printf(".\n");
  delay(100);
}

*/