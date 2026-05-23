#pragma once
#include <Arduino.h>
#include "Config.h"

class LineSensor {
public:
    LineSensor() {}

    void begin() {
        pinMode(Pins::LEFT_IR_SENSOR, INPUT);
        pinMode(Pins::RIGHT_IR_SENSOR, INPUT);
    }

    void read(bool &left_black, bool &right_black) {
        left_black = (digitalRead(Pins::LEFT_IR_SENSOR));
        right_black = (digitalRead(Pins::RIGHT_IR_SENSOR));
    }
};