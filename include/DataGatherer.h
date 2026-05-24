#pragma once
#include <Arduino.h>
#include "Config.h"

struct TrainingSample {
    float accel_x;
    float gyro_z;
    float label;
};

class DataGatherer {
public:
    DataGatherer() : sample_count(0), last_record_time(0) {}

    void record(float accel_x, float gyro_z, float label) {
        if (sample_count < ML::MAX_SAMPLES && (millis() - last_record_time > ML::SAMPLE_INTERVAL_MS)) {
            
            dataset[sample_count].accel_x = accel_x;
            dataset[sample_count].gyro_z = gyro_z;
            dataset[sample_count].label = label;

            sample_count++;
            last_record_time = millis();
        }
    }

    bool isFull() const { return sample_count >= ML::MAX_SAMPLES; }
    int getSampleCount() const { return sample_count; }
    TrainingSample* getDataSet() { return dataset; }

private:
    TrainingSample dataset[ML::MAX_SAMPLES];
    int sample_count;
    unsigned long last_record_time;
};