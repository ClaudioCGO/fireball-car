#pragma once
#include <Arduino.h>

// Hardware
namespace Pins {
    // H bridge and motors
    const uint8_t HB_STANDBY = 3;

    const uint8_t LEFT_MOTOR_IN1 = 21;
    const uint8_t LEFT_MOTOR_IN2 = 20;
    const uint8_t LEFT_MOTOR_PWM = 10;
    
    const uint8_t RIGHT_MOTOR_IN1 = 1; // this and the bellow pin are physically switched.
    const uint8_t RIGHT_MOTOR_IN2 = 2;
    const uint8_t RIGHT_MOTOR_PWM = 0;

    // I2C
    const uint8_t I2C_SDA = 8;
    const uint8_t I2C_SCL = 9;

    // IR SENSORS
    const uint8_t LEFT_IR_SENSOR = 6;
    const uint8_t RIGHT_IR_SENSOR = 7;    
}

// Software
namespace Config {
    const int I2C_SPEED_HZ = 100000;
    const int BMI160_I2C_ADDR = 0x69;

    // Scales
    static constexpr int PWM_MAX = 255;
    static constexpr float GYRO_SCALE = 16.384f;    // +-2000º/s
    static constexpr float ACCEL_SCALE = 16384.0f;  // +-2g
    static constexpr float ACCEL_SMOOTH_ALPHA = 0.2f;

    static constexpr float DEADZONE_THRESHOLD = 1.2f;


    const float LEFT_MOTOR_TRIM = 1.0;
    const float RIGHT_MOTOR_TRIM = 1.0;

    static const uint8_t LINE_DETECT_STATE = HIGH;

    // Speeds
    const float SPEED_STRAIGHT = 0.50;
    const float SPEED_CURVE = 0.75;
    const float SPEED_CROSSROAD = 0.55;

    const float MOTOR_POWER_DIFF = 1.0;

    // GYR PID
    const float KP_GYRO = 0.015;
    //const float KI_GYRO = 0.000;
    //const float KP_GYRO = 0.005;
    //const float MAX_INTEGRAL_GYRO = 10.0;

    // Line PID
    //const float KP_LINE = 0.030;
    //const float KI_LINE = 0.000;
    //const float KP_LINE = 0.15;
    //const float MAX_INTEGRAL_LINE = 5.0;

    const unsigned long LOST_LINE_TIMEOUT_MS = 150;
    const unsigned long CROSSING_TIMEOUT_MS = 50;
    const unsigned long LAP_END_SAFEGUARD_MS = 30000;
}