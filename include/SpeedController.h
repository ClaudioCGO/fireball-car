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

    MotorSpeeds compute(float error, bool allow_turbo, float safe_prob) {
        float speed = Tuning::BASE_SPEED;

        if (allow_turbo && safe_prob >= ML::CONFIDENCE_THRESHOLD) {
            speed = ML::BOOST_SPEED;
        }
        
        unsigned long now = micros();
        float dt = (now - last_time) / Config::US_TO_SEC;
        if (dt <= 0.0f) dt = Config::PID_DT_FALLBACK;


        if (error == 0.0f) {
            integral = 0.0f;
        } else {
            integral += error * dt;
            integral = constrain(integral, -1.0f, 1.0f);
        }

        float derivative = (error - previous_error) / dt;

        float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

        previous_error = error;
        last_time = now;

        MotorSpeeds target;
        target.left = speed + correction;
        target.right = speed - correction;

        target.left = constrain(target.left, -1.0f, 1.0f);
        target.right = constrain(target.right, -1.0f, 1.0f);

        return target;
    }

private:
    float Kp, Ki, Kd;
    float previous_error;
    float integral;
    unsigned long last_time;
};