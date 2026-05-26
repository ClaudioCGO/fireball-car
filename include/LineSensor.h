#pragma once
#include <Arduino.h>
#include "Config.h"

struct SensorState {
    bool outer_left_on_line;
    bool inner_left_on_line;
    bool inner_right_on_line;
    bool outer_right_on_line;
};


class LineSensor {
public:
    LineSensor() {}

    void begin() {
        pinMode(Pins::OUTER_LEFT_IR_SENSOR, INPUT_PULLUP);
        pinMode(Pins::INNER_LEFT_IR_SENSOR, INPUT_PULLUP);
        pinMode(Pins::INNER_RIGHT_IR_SENSOR, INPUT_PULLUP);
        pinMode(Pins::OUTER_RIGHT_IR_SENSOR, INPUT_PULLUP);
    }


    SensorState read() {
        SensorState state;

        state.outer_left_on_line = (digitalRead(Pins::OUTER_LEFT_IR_SENSOR) == Config::LINE_DETECT_STATE);
        state.inner_left_on_line = (digitalRead(Pins::INNER_LEFT_IR_SENSOR) == Config::LINE_DETECT_STATE);
        state.inner_right_on_line = (digitalRead(Pins::INNER_RIGHT_IR_SENSOR) == Config::LINE_DETECT_STATE);
        state.outer_right_on_line = (digitalRead(Pins::OUTER_RIGHT_IR_SENSOR) == Config::LINE_DETECT_STATE);

        return state;
    }
};