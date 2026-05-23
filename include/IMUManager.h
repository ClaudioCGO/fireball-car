#pragma once
#include <BMI160Gen.h>
#include "Config.h"

class IMUManager {
public:
    IMUManager() : current_yaw(0.0f), previous_time(0),
                   ax(0.0f), ay(0.0f), az(0.0f),
                   gx(0.0f), gy(0.0f), gz(0.0f) {}

    bool begin() {
        if (!BMI160.begin(BMI160GenClass::I2C_MODE, Config::BMI160_I2C_ADDR)) {
            return false;
        }
        previous_time = micros();
        return true;
    }

    void update() {
        int raw_ax, raw_ay, raw_az;
        int raw_gx, raw_gy, raw_gz;

        BMI160.readAccelerometer(raw_ax, raw_ay, raw_az);
        BMI160.readGyro(raw_gx, raw_gy, raw_gz);

        unsigned long current_time = micros();

        float dt = (current_time - previous_time) * 1000000.0f;
        previous_time = current_time;

        ax = applySmoothing((float)raw_ax / Config::ACCEL_SCALE, ax);
        ay = applySmoothing((float)raw_ay / Config::ACCEL_SCALE, ay);
        az = applySmoothing((float)raw_az / Config::ACCEL_SCALE, az);

        gx = applyDeadzone((float)raw_gx / Config::GYRO_SCALE, Config::DEADZONE_THRESHOLD);
        gy = applyDeadzone((float)raw_gy / Config::GYRO_SCALE, Config::DEADZONE_THRESHOLD);
        gz = applyDeadzone((float)raw_gz / Config::GYRO_SCALE, Config::DEADZONE_THRESHOLD);

        // Angular velocity
        current_yaw += gz * dt;
    }




    float getAccelX () const { return ax; }
    float getAccelY () const { return ay; }
    float getAccelZ () const { return az; }

    float getGyroX () const { return gx; }
    float getGyroY () const { return gy; }
    float getGyroZ () const { return gz; }

    float getYaw () const { return current_yaw; }

private:
    float ax, ay, az;
    float gx, gy, gz, current_yaw;
    unsigned long previous_time;

    // Exponential Moving Average
    float applySmoothing(float new_value, float current_value) {
        return (Config::ACCEL_SMOOTH_ALPHA * new_value) + 
        ((1.0f - Config::ACCEL_SMOOTH_ALPHA) * current_value);
    }

    // Remove static noise
    float applyDeadzone(float value, float limit) {
        return (abs(value) < limit) ? 0.0f : value;
    }
};
