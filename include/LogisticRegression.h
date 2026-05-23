#pragma once
#include <Arduino.h>
#include <math.h>
#include "Config.h"

class LogisticRegression {
public:
    LogisticRegression(float lr = ML::LEARNING_RATE) : learning_rate(lr), bias(0.0f) {
        for (int i = 0; i < NUM_FEATURES; i++) weights[i] = 0.0f;
    }

    void train(float sample_data[][2], float* labels, int num_samples, int epochs) {
        Serial.println("\n[ML] Training Model...");

        for (int e = 0; e < epochs; e++) {
            for (int i = 0; i < num_samples; i++) {
                float z = bias + (weights[0] * sample_data[i][0]) +  (weights[1] * sample_data[i][1]);
                float prediction = sigmoid(z);
                float error = prediction - labels[i];
                
                bias -= learning_rate * error;
                weights[0] = learning_rate * error * sample_data[i][0];
                weights[1] = learning_rate * error * sample_data[i][1];
            }
        }
        Serial.println("[ML] Training complete!");
    }

    void evaluate(float sample_data[][2], float* labels, int num_samples, float boost_threshold) {
        int true_positives = 0;
        int true_negatives = 0;
        int false_positives = 0;
        int false_negatives = 0;

        for (int i = 0; i < num_samples; i++) {
            float prob = predict(sample_data[i][0], sample_data[i][1]);
            
            bool predicted_straight = (prob >= boost_threshold);
            bool actual_straight = (labels[i] >= 0.5f);

            if (predicted_straight && actual_straight) true_positives++;
            else if (!predicted_straight && !actual_straight) true_negatives++;
            else if (predicted_straight && !actual_straight) false_positives++;
            else if (!predicted_straight && actual_straight) false_negatives++;
        }

        float accuracy = (float)(true_positives + true_negatives) / num_samples * 100.0f;

        Serial.println("\n=== ML MODEL PERFORMANCE ===");
        Serial.printf("Total Track Samples: %d\n", num_samples);
        Serial.printf("Overall Accuracy:    %.1f%%\n", accuracy);
        Serial.println("----------------------------");
        Serial.printf("Safe Boosts (TP):    %d\n", true_positives);
        Serial.printf("Safe Slows  (TN):    %d\n", true_negatives);
        Serial.printf("Missed Ops  (FN):    %d (Too cautious)\n", false_negatives);
        Serial.printf("CRASH RISKS (FP):    %d (Boosted in curve!)\n", false_positives);
        Serial.println("============================\n");
    }

    float predict(float feature_1, float feature_2) {
        float z = bias + (weights[0] * feature_1) + (weights[1] * feature_2);
        return sigmoid(z);
    }

private:
    static const int NUM_FEATURES = 2;
    float weights[NUM_FEATURES];
    float bias;
    float learning_rate;

    float sigmoid(float z) {
        z = constrain(z, -10.0f, 10.0f);
        return 1.0f / (1.0f + exp(-z));
    }
};