#pragma once
#include "Config.h"
#include "MotorController.h"
#include "IMUManager.h"
#include "LineSensor.h"

class LineFollower {
public:
    enum State { FOLLOWING, CROSSING, SEARCHING, WAITING };

    LineFollower(MotorController &m, IMUManager &i, LineSensor &s) : motors(m), imu(i), sensors(s), 
                current_state(FOLLOWING), /*target_yaw(0.0f),*/ execution_beginning_time(0), last_line_seen(0) {}

    void begin() {
        motors.begin();
        sensors.begin();

        execution_beginning_time = millis();
    }

    void update() {
        if (current_state == WAITING) {
            return;
        }

        bool left_black, right_black;
        sensors.read(left_black, right_black);

        switch (current_state) {
        case FOLLOWING: followProcess(left_black, right_black); break;
        case CROSSING: crossProcess(left_black, right_black); break;
        case SEARCHING: searchProcess(left_black, right_black); break;
        default: break;
        }
    }


private:
    MotorController &motors;
    IMUManager &imu;
    LineSensor &sensors;

    State current_state;
    //float target_yaw;
    unsigned long execution_beginning_time;
    int last_line_seen;

    void followProcess(bool left_black, bool right_black) {
        if (left_black && right_black) {
            current_state == CROSSING;
            //target_yaw = imu.getYaw();
        }
        else if (!left_black && !right_black) {
            motors.move(Config::SPEED_STRAIGHT, Config::SPEED_STRAIGHT);
            //target_yaw = imu.getYaw();
        }
        else if (left_black && !right_black) {
            last_line_seen = -1;
            motors.move(-Config::SPEED_CURVE, Config::SPEED_CURVE);
        }
        else if (!left_black && right_black) {
            last_line_seen = 1;
            motors.move(Config::SPEED_CURVE, -Config::SPEED_CURVE);
        }
    }

    void crossProcess(bool left_black, bool right_black) {
        unsigned long total_time = millis() - execution_beginning_time;
        
        if (left_black && right_black && (total_time > Config::LAP_END_SAFEGUARD_MS)) {
            current_state = WAITING;
            motors.brake();
            return;
        }

        //float error = target_yaw - imu.getYaw();
        //float fix = error * Config::KP_GYRO;
        motors.move(Config::SPEED_CROSSROAD, Config::SPEED_CROSSROAD);

        if ( !left_black && !right_black) {
            current_state = FOLLOWING;
        }
    }

    void searchProcess(bool left_black, bool right_black) {
        if (left_black || right_black) {
            current_state = FOLLOWING;
        }
        else {
            if (last_line_seen == -1) motors.move(-Config::SPEED_CURVE, Config::SPEED_CURVE);
            else if (last_line_seen == 1) motors.move(-Config::SPEED_CURVE, Config::SPEED_CURVE);
        }
    }

    String getStateName() {
        switch (current_state) {
        case FOLLOWING: return "FOLLOWING";
        case CROSSING: return "CROSSING";
        case SEARCHING: return "SEARCHING";
        case WAITING: return "WAITING";
        default: return "UNKNOWN";
        }
    }
};