#pragma once
#include "Config.h"
#include "MotorController.h"
#include "IMUManager.h"
#include "LineSensor.h"
#include "DataGatherer.h"
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
        trained_this_lap(false), wait_start_time(0),
        straight_concecutive_cycles(0) {}

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

    int straight_concecutive_cycles;

    void followProcess(bool left_black, bool right_black) {
        float current_label = 1.0f;
        float error = 0.0f;
        float safe_prob = 0.0f;

        // Two Sensors active
        if (left_black && right_black) {
            //straight_concecutive_cycles = 0;
            
            // Lap end check
            if ((millis() - execution_beginning_time) > Tuning::LAP_END_SAFEGUARD_MS) {
                motors.hardBrake();    
                current_state = WAITING;
            } else {
                current_state = CROSSING;
                target_yaw = imu.getYaw();
                crossing_start_time = millis();
            }
            return;
        }

        // Only Left Sensor active
        else if (left_black && !right_black) {
            error = -1.0f;
            last_line_seen = -1;
            last_line_time = millis();
            current_label = 0.0f;
            straight_concecutive_cycles = 0;
        }

        // Only Right Sensor active
        else if (!left_black && right_black) {
            error = 1.0f;
            last_line_seen = 1;
            last_line_time = millis();
            current_label = 0.0f;
            straight_concecutive_cycles = 0;
        }
        
        // No Sensors active
        else if (!left_black && !right_black) {
            error = 0.0f;
            current_label = 1.0f;

            if (millis() - last_line_time > Tuning::LOST_LINE_TIMEOUT_MS) {
                current_state = SEARCHING;
                straight_concecutive_cycles = 0;
                return;
            }

            if (lap_count > 1) {
                safe_prob = ml_model.predict(abs(imu.getAccelX()), abs(imu.getGyroZ()) / Config::GYRO_NORM_FACTOR);    

                //Temporal Filter
                if (safe_prob >= ML::CONFIDENCE_THRESHOLD) {
                    straight_concecutive_cycles++;
                } else {
                    straight_concecutive_cycles = 0;
                }
            }
        }
        
        
        bool allow_turbo = (lap_count > 1 && error == 0.0f && straight_concecutive_cycles >= ML::MIN_CYCLES_TURBO);
        MotorSpeeds speeds = pid.compute(error, allow_turbo, safe_prob);

        motors.move(speeds.left, speeds.right);

        if (!gatherer.isFull() && (lap_count == 1)) {
            gatherer.record(abs(imu.getAccelX()), (abs(imu.getGyroZ()) / Config::GYRO_NORM_FACTOR) , current_label);
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
            if (millis() - last_line_time > Tuning::SEARCH_SAFETY_TIMEOUT_MS) {
                motors.setStandby(true);
            }
            
            else if (last_line_seen == -1) motors.move(-Tuning::SPEED_CURVE, Tuning::SPEED_CURVE);
            else if (last_line_seen == 1) motors.move(Tuning::SPEED_CURVE, -Tuning::SPEED_CURVE);
        }
    }

    void waitingProcess() {
        if (!trained_this_lap) {
            if (lap_count == 1) {
                ml_model.train(gatherer.getDataSet(), gatherer.getSampleCount());
                ml_model.evaluate(gatherer.getDataSet(), gatherer.getSampleCount(), ML::CONFIDENCE_THRESHOLD);
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