#pragma once
#include "Config.h"
#include "MotorController.h"
#include "IMUManager.h"
#include "LineSensor.h"

class LineFollower {
public:
    enum State { FOLLOWING, CROSSING, SEARCHING, WAITING };

    LineFollower(MotorController &m, IMUManager &i, LineSensor &s) 
        : motors(m), imu(i), sensors(s), 
        current_state(FOLLOWING), target_yaw(0.0f),
         execution_beginning_time(0), last_line_seen(0), last_line_time(0) {}

    void begin() {
        motors.begin();
        sensors.begin();
        execution_beginning_time = millis();
    }

    void update() {
        if (current_state == WAITING) return;

        SensorState line = sensors.read();

        switch (current_state) {
        case FOLLOWING: followProcess(line.left_on_line, line.right_on_line); break;
        case CROSSING: crossProcess(line.left_on_line, line.right_on_line); break;
        case SEARCHING: searchProcess(line.left_on_line, line.right_on_line); break;
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


    void followProcess(bool left_black, bool right_black) {
        if (left_black && right_black) {
            unsigned long total_race_time = millis() - execution_beginning_time;
              
            if (total_race_time > Config::LAP_END_SAFEGUARD_MS) {
                current_state = WAITING;
                motors.hardBrake();
                return;
            } else {
                current_state == CROSSING;
                target_yaw = imu.getYaw();
                crossing_start_time = millis();
            }            
        }
        else if (!left_black && !right_black) {
            motors.move(Config::SPEED_STRAIGHT, Config::SPEED_STRAIGHT);
            
            if (millis() - last_line_time > Config::LOST_LINE_TIMEOUT_MS) {
                current_state = SEARCHING;
            }
        }
        else if (left_black && !right_black) {
            last_line_seen = -1;
            last_line_time = millis();
            motors.move(-Config::SPEED_CURVE, Config::SPEED_CURVE);
        }
        else if (!left_black && right_black) {
            last_line_seen = 1;
            last_line_time = millis();
            motors.move(Config::SPEED_CURVE, -Config::SPEED_CURVE);
        }
    }

    void crossProcess(bool left_black, bool right_black) {
        unsigned long time_in_cross = millis() - crossing_start_time;

        float error = target_yaw - imu.getYaw();
        float fix = error * Config::KP_GYRO;
        
        // May need to invert the -+
        motors.move(Config::SPEED_CROSSROAD - fix, Config::SPEED_CROSSROAD + fix);

        if (time_in_cross > Config::CROSSING_TIMEOUT_MS) {
            current_state = FOLLOWING;
            last_line_time = millis();
        }
    }

    void searchProcess(bool left_black, bool right_black) {
        if (left_black || right_black) {
            current_state = FOLLOWING;
            last_line_time = millis();
        }
        else {
            if (last_line_seen == -1) motors.move(-Config::SPEED_CURVE, Config::SPEED_CURVE);
            else if (last_line_seen == 1) motors.move(-Config::SPEED_CURVE, Config::SPEED_CURVE);
        }
    }
};