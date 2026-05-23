#pragma once
#include <Arduino.h>
#include "Config.h"

struct SensorState {
    bool left_on_line;
    bool right_on_line;
};


class LineSensor {
public:
    LineSensor() {}

    void begin() {
        pinMode(Pins::LEFT_IR_SENSOR, INPUT_PULLUP);
        pinMode(Pins::RIGHT_IR_SENSOR, INPUT_PULLUP);
    }


    SensorState read() {
        SensorState state;

        state.left_on_line = (digitalRead(Pins::LEFT_IR_SENSOR) == Config::LINE_DETECT_STATE);
        state.right_on_line = (digitalRead(Pins::RIGHT_IR_SENSOR == Config::LINE_DETECT_STATE));

        return state;
    }
};