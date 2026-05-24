#pragma once
#include <Arduino.h>
#include "Config.h"

class MotorController {
public:
    MotorController() {}

    void begin() {
        pinMode(Pins::HB_STANDBY, OUTPUT);

        pinMode(Pins::LEFT_MOTOR_IN1, OUTPUT);
        pinMode(Pins::LEFT_MOTOR_IN2, OUTPUT);
        pinMode(Pins::LEFT_MOTOR_PWM, OUTPUT);
        
        pinMode(Pins::RIGHT_MOTOR_IN1, OUTPUT);
        pinMode(Pins::RIGHT_MOTOR_IN2, OUTPUT);
        pinMode(Pins::RIGHT_MOTOR_PWM, OUTPUT);
        
        setStandby(false);
    }

    void setStandby(bool state) {
        digitalWrite(Pins::HB_STANDBY, state ? LOW: HIGH);
    }

    void move(float left_coef, float right_coef) {
        left_coef *= Tuning::LEFT_MOTOR_TRIM;
        right_coef *= Tuning::RIGHT_MOTOR_TRIM;

        setMotor(left_coef, Pins::LEFT_MOTOR_IN1, Pins::LEFT_MOTOR_IN2, Pins::LEFT_MOTOR_PWM);
        setMotor(right_coef, Pins::RIGHT_MOTOR_IN1, Pins::RIGHT_MOTOR_IN2, Pins::RIGHT_MOTOR_PWM);
    }

    void hardBrake() {
        digitalWrite(Pins::LEFT_MOTOR_IN1, HIGH);
        digitalWrite(Pins::LEFT_MOTOR_IN2, HIGH);
        analogWrite(Pins::LEFT_MOTOR_PWM, Config::PWM_MAX);

        digitalWrite(Pins::RIGHT_MOTOR_IN1, HIGH);
        digitalWrite(Pins::RIGHT_MOTOR_IN2, HIGH);
        analogWrite(Pins::RIGHT_MOTOR_PWM, Config::PWM_MAX);
    }

private:
    void setMotor(float coeficient, uint8_t in1_pin, uint8_t in2_pin, uint8_t pwm_pin) {
        coeficient = constrain(coeficient, -1.0f, 1.0f);

        if (coeficient == 0.0f) {
            digitalWrite(in1_pin, LOW);
            digitalWrite(in2_pin, LOW);
            analogWrite(pwm_pin, 0);
            return;
        }

        if (coeficient > 0) {
            digitalWrite(in1_pin, HIGH);
            digitalWrite(in2_pin, LOW);
        } else {
            digitalWrite(in1_pin, LOW);
            digitalWrite(in2_pin, HIGH);
        }

        analogWrite(pwm_pin, static_cast<int>(abs(coeficient) * Config::PWM_MAX));
    }
};