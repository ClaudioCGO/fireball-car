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
        pinMode(Pins::INNER_LEFT_IR_SENSOR, INPUT_PULLUP);
        pinMode(Pins::INNER_RIGHT_IR_SENSOR, INPUT_PULLUP);
    }


    SensorState read() {
        SensorState state;

        state.left_on_line = (digitalRead(Pins::INNER_LEFT_IR_SENSOR) == Config::LINE_DETECT_STATE);
        state.right_on_line = (digitalRead(Pins::INNER_RIGHT_IR_SENSOR) == Config::LINE_DETECT_STATE);

        return state;
    }
};