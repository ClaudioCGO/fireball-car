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
    enum State { FOLLOWING, SEARCHING, WAITING };

    LineFollower(MotorController &m, IMUManager &i, LineSensor &s) 
    : motors(m), imu(i), sensors(s), pid(Tuning::KP_LINE, Tuning::KI_LINE, Tuning::KD_LINE) {}

    DataGatherer gatherer;
    LogisticRegression ml_model;
    SpeedController pid;
    int lap_count = 1;

    void begin() {
        motors.begin();
        sensors.begin();
        pid.reset();
        execution_beginning_time = millis();
    }

    void update() {
        SensorState line = sensors.read();

        switch (current_state) {
        case FOLLOWING: followProcess(line); break;
        case SEARCHING: searchProcess(line); break;
        case WAITING: waitingProcess(); break;
        default: break;
        }
    }

    const char* getStateName() {
        switch (current_state) {
        case FOLLOWING: return "FOLLOWING";
        case SEARCHING: return "SEARCHING";
        case WAITING:   return "WAITING";
        default:        return "UNKNOWN";
        }
    }

private:
    MotorController &motors;
    IMUManager &imu;
    LineSensor &sensors;

    State current_state = FOLLOWING;
    unsigned long execution_beginning_time = 0;
    unsigned long last_line_time = 0;
    unsigned long wait_start_time = 0;
    unsigned long last_intersection_time = 0;

    int last_line_seen = 0;
    int straight_concecutive_cycles = 0;
    bool trained_this_lap = false;


    void followProcess(const SensorState& line) {
        float current_label = 1.0f;
        float current_error = 0.0f;
        float safe_prob = 0.0f;


        // Bit Mask conversion
        uint8_t sensor_mask = 0;
        if (line.outer_left_on_line) sensor_mask |= 0x08;   // Bit 3 (MSB)
        if (line.inner_left_on_line) sensor_mask |= 0x04;   // Bit 2
        if (line.inner_right_on_line) sensor_mask |= 0x02;  // Bit 1
        if (line.outer_right_on_line) sensor_mask |= 0x01;   // Bit 0 (LSB)


        // Crossroad
        if (line.outer_left_on_line && line.outer_right_on_line) {
            
            // Lap end check
            if ((millis() - execution_beginning_time) > Tuning::LAP_END_SAFEGUARD_MS) {
                motors.hardBrake();    
                current_state = WAITING;
                return;
            } 
            
            // Go blind for a bit
            last_intersection_time = millis();
        }

        switch (sensor_mask) {
            case 0b0110:
                current_error = 0.0f;
                last_line_time = millis();
                if (lap_count > 1) straight_concecutive_cycles++;
                break;
            
            case 0b0100:
                current_error = -0.6f;
                last_line_seen = -1;
                break;
            
            case 0b0010:
            
        }



        /*


        // Two Sensors active
        if (left_black && right_black) {
            //straight_concecutive_cycles = 0;
            
            
            } else {
                current_state = CROSSING;
                target_yaw = imu.getYaw();
                crossing_start_time = millis();
            }
            return;
        }

        // Only Left Sensor active
        else if (left_black && !right_black) {
            current_error = -1.0f;
            last_line_seen = -1;
            last_line_time = millis();
            current_label = 0.0f;
            straight_concecutive_cycles = 0;
        }

        // Only Right Sensor active
        else if (!left_black && right_black) {
            current_error = 1.0f;
            last_line_seen = 1;
            last_line_time = millis();
            current_label = 0.0f;
            straight_concecutive_cycles = 0;
        }
        
        // No Sensors active
        else if (!left_black && !right_black) {
            current_error = 0.0f;
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
        
        
        bool allow_turbo = (lap_count > 1 && current_error == 0.0f && straight_concecutive_cycles >= ML::MIN_CYCLES_TURBO);
        MotorSpeeds speeds = pid.compute(current_error, allow_turbo, safe_prob);

        motors.move(speeds.left, speeds.right);

        if (!gatherer.isFull() && (lap_count == 1)) {
            gatherer.record(abs(imu.getAccelX()), (abs(imu.getGyroZ()) / Config::GYRO_NORM_FACTOR) , current_label);
        }
        */
    }



    void searchProcess(const SensorState& line) {
        /*
        if (left_black || right_black) {
            current_state = FOLLOWING;
            last_line_time = millis();
            pid.reset();
        }
        else {
            if (millis() - last_line_time > Tuning::SEARCH_SAFETY_TIMEOUT_MS) {
                motors.setStandby(true);
            }
            
            else if (last_line_seen == -1) motors.move(-Tuning::CURVE_SPEED, Tuning::CURVE_SPEED);
            else if (last_line_seen == 1) motors.move(Tuning::CURVE_SPEED, -Tuning::CURVE_SPEED);
        }
        */
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