#pragma once
#include <Arduino.h>
#include "Config.h"

class DataGatherer {
public:
    float sample_data[ML::MAX_SAMPLES][2];
    float label_data[ML::MAX_SAMPLES];

    int sample_count = 0;

    DataGatherer() : last_record_time(0) {}

    void record(float feature_1, float feature_2, float label) {
        if (sample_count < ML::MAX_SAMPLES && (millis() - last_record_time > ML::SAMPLE_INTERVAL_MS)) {
            
            sample_data[sample_count][0] = feature_1;
            sample_data[sample_count][1] = feature_2;
            label_data[sample_count] = label;

            sample_count++;
            last_record_time = millis();
        }
    }

    bool isFull() const {
        return sample_count >= ML::MAX_SAMPLES;
    }

private:
    unsigned long last_record_time;
};