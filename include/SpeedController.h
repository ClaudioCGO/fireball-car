#pragma once
#include <Arduino.h>
#include "Config.h"
#include "LogisticRegression.h"

struct MotorSpeeds {
    float left;
    float right;
};

class SpeedController {
public:
    SpeedController(float initial_kp, float initial_ki, float initial_kd)
    : Kp(initial_kp), Ki(initial_ki), Kd(initial_kd), previous_error(0.0f), integral(0.0f), last_time(0) {}

    void reset() {
        integral = 0.0f;
        previous_error = 0.0f;
        last_time = micros();
    }

    float compute(float error) {
        unsigned long now = micros();
        float dt = (now - last_time) / 1000000.0f;

        if (dt <= 0.0f) dt = 0.001f;

        integral += error * dt;
        float derivative = (error - previous_error) / dt;

        float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

        previous_error = error;
        last_time = now;

        return output;
    }

private:
    float Kp, Ki, Kd;
    float previous_error;
    float integral;
    unsigned long last_time;
};