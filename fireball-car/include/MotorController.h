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
        pinMode(Pins::LEFT_MOTOR_PWM, OUTPUT);
        setStandby(false);
    }

    void setStandby(bool state) {
        digitalWrite(Pins::HB_STANDBY, state ? LOW: HIGH);
    }

    void move(float left_coef, float right_coef) {
        // Apply the difference in one wrong motor ( dont know wich)
        //left_coef = Config::MOTOR_POWER_DIFF * right_coef;
        setStandby(false);

        setMotorLeft(left_coef);
        setMotorRight(right_coef);

        setStandby(true);
    }

    void brake() {
        setStandby(false);

        digitalWrite(Pins::LEFT_MOTOR_IN1, LOW);
        digitalWrite(Pins::LEFT_MOTOR_IN2, LOW);
        analogWrite(Pins::LEFT_MOTOR_PWM, 0);

        digitalWrite(Pins::RIGHT_MOTOR_IN1, LOW);
        digitalWrite(Pins::RIGHT_MOTOR_IN2, LOW);
        analogWrite(Pins::RIGHT_MOTOR_PWM, 0);

        setStandby(true);
    }

private:
    void setMotorLeft(float coeficient) {
        coeficient = constrain(coeficient, -1.0f, 1.0f);
        if (coeficient >= 0) {
            digitalWrite(Pins::LEFT_MOTOR_IN1, HIGH);
            digitalWrite(Pins::LEFT_MOTOR_IN2, LOW);
        } else {
            digitalWrite(Pins::LEFT_MOTOR_IN1, LOW);
            digitalWrite(Pins::LEFT_MOTOR_IN2, HIGH);
        }

        analogWrite(Pins::LEFT_MOTOR_PWM, round(abs(coeficient) * Config::PWM_MAX));
    }

    void setMotorRight(float coeficient) {
        coeficient = constrain(coeficient, -1.0f, 1.0f);
        if (coeficient >= 0) {
            digitalWrite(Pins::RIGHT_MOTOR_IN1, HIGH);
            digitalWrite(Pins::RIGHT_MOTOR_IN2, LOW);
        } else {
            digitalWrite(Pins::RIGHT_MOTOR_IN1, LOW);
            digitalWrite(Pins::RIGHT_MOTOR_IN2, HIGH);
        }

        analogWrite(Pins::RIGHT_MOTOR_PWM, round(abs(coeficient) * Config::PWM_MAX));
    }
};