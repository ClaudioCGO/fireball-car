#pragma once
#include "Config.h"
#include "MotorController.h"
#include "IMUManager.h"
#include "LineSensor.h"
#include "DataGathering.h"
#include "LogisticRegression.h"
#include "SpeedController.h"

class LineFollower {
public:
    enum State { FOLLOWING, CROSSING, SEARCHING, WAITING };

    LineFollower(MotorController &m, IMUManager &i, LineSensor &s) 
        : motors(m), imu(i), sensors(s), 
        current_state(FOLLOWING), target_yaw(0.0f),
        execution_beginning_time(0), last_line_seen(0), last_line_time(0),
        lap_count(1),
        pid(Tuning::KP_LINE, Tuning::KI_LINE, Tuning::KD_LINE),
        trained_this_lap(false), wait_start_time(0) {}

    DataGatherer gatherer;
    LogisticRegression ml_model;
    SpeedController pid;
    int lap_count;

    void begin() {
        motors.begin();
        sensors.begin();
        pid.reset();
        execution_beginning_time = millis();
    }

    void update() {
        SensorState line = sensors.read();

        switch (current_state) {
        case FOLLOWING: followProcess(line.left_on_line, line.right_on_line); break;
        case CROSSING: crossProcess(line.left_on_line, line.right_on_line); break;
        case SEARCHING: searchProcess(line.left_on_line, line.right_on_line); break;
        case WAITING: waitingProcess(); break;
        default: break;
        }
    }

    const char* getStateName() {
        switch (current_state) {
        case FOLLOWING: return "FOLLOWING";
        case CROSSING:  return "CROSSING";
        case SEARCHING: return "SEARCHING";
        case WAITING:   return "WAITING";
        default:        return "UNKNOWN";
        }
    }

private:
    MotorController &motors;
    IMUManager &imu;
    LineSensor &sensors;

    State current_state;
    float target_yaw;
    unsigned long execution_beginning_time;
    int last_line_seen;
    unsigned long last_line_time;
    unsigned long crossing_start_time;

    bool trained_this_lap;
    unsigned long wait_start_time;

    void followProcess(bool left_black, bool right_black) {
        float current_label;
        float base_speed = Tuning::SPEED_STRAIGHT;
        float error = 0.0f;

        // Two Sensors active
        if (left_black && right_black) {
            
            // Lap end check
            unsigned long total_race_time = millis() - execution_beginning_time;

            if (total_race_time > Tuning::LAP_END_SAFEGUARD_MS) {
                motors.hardBrake();    
                current_state = WAITING;
                return;
            } 

            else {
                current_state = CROSSING;
                target_yaw = imu.getYaw();
                crossing_start_time = millis();
                current_label = 1.0f;
            }
        }

        // Only Left Sensor active
        else if (left_black && !right_black) {
            error = -1.0f;
            last_line_seen = -1;
            last_line_time = millis();
            current_label = 0.0f;
        }

        // Only Right Sensor active
        else if (!left_black && right_black) {
            error = 1.0f;
            last_line_seen = 1;
            last_line_time = millis();
            current_label = 0.0f;
        }
        
        // No Sensors active
        else if (!left_black && !right_black) {
            error = 0.0f;
            current_label = 1.0f;

            if (millis() - last_line_time > Tuning::LOST_LINE_TIMEOUT_MS) {
                current_state = SEARCHING;
            }

            if (lap_count > 1) {
                float safe_prob = ml_model.predict(imu.getAccelX(), imu.getGyroZ());
                if (safe_prob >= ML::CONFIDENCE_THRESHOLD) {
                    base_speed = ML::BOOST_SPEED;
                }    
            }
        }
        
        float correction = pid.compute(error);
        motors.move(base_speed + correction, base_speed - correction);

        // Record the info
        if (!gatherer.isFull() && (lap_count == 1)) {
            gatherer.record(imu.getAccelX(), imu.getGyroZ(), current_label);
        }
    }

    void crossProcess(bool left_black, bool right_black) {
        unsigned long time_in_cross = millis() - crossing_start_time;

        float error = target_yaw - imu.getYaw();
        float fix = error * Tuning::KP_GYRO;
        
        // May need to invert the -+
        motors.move(Tuning::SPEED_CROSSROAD - fix, Tuning::SPEED_CROSSROAD + fix);

        if (time_in_cross > Tuning::CROSSING_TIMEOUT_MS) {
            current_state = FOLLOWING;
            last_line_time = millis();
            pid.reset();
        }
    }

    void searchProcess(bool left_black, bool right_black) {
        if (left_black || right_black) {
            current_state = FOLLOWING;
            last_line_time = millis();
            pid.reset();
        }
        else {
            if (last_line_seen == -1) motors.move(-Tuning::SPEED_CURVE, Tuning::SPEED_CURVE);
            else if (last_line_seen == 1) motors.move(Tuning::SPEED_CURVE, -Tuning::SPEED_CURVE);
        }
    }

    void waitingProcess() {
        if (!trained_this_lap) {
            if (lap_count == 1) {
                ml_model.train(gatherer.sample_data, gatherer.label_data, gatherer.sample_count);
                ml_model.evaluate(gatherer.sample_data, gatherer.label_data, gatherer.sample_count, ML::CONFIDENCE_THRESHOLD);
            }
            trained_this_lap = true;
            wait_start_time = millis();
        }
        
        if(millis() - wait_start_time > Tuning::WAITING_TIME_MS) {
            if (Tuning::DO_SECOND_LAP) {
                lap_count++;
                current_state = FOLLOWING;
                trained_this_lap = false;

                pid.reset();
                execution_beginning_time = millis();
            }
        }
    }
};