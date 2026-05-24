#pragma once
#include <Arduino.h>
#include <math.h>
#include "Config.h"
#include "DataGathering.h"

class LogisticRegression {
public:
    LogisticRegression(float lr = ML::LEARNING_RATE) : learning_rate(lr), bias(0.0f) {
        weights[0] = 0.0f;
        weights[1] = 0.0f;
    }

    void train(TrainingSample* dataset, int num_samples) {
        if (num_samples == 0) return;
        
        //Shuffle with Fisher-Yates
        Serial.println("\n[ML] Shuffling dataset for better convergence...");
        shuffleDataset(dataset, num_samples);

        Serial.println("\n[ML] Training Model...");

        for (int e = 0; e < ML::TRAINING_EPOCHS; e++) {

            for (int i = 0; i < num_samples; i++) {
                float z = bias + (weights[0] * dataset[i].accel_x) +  (weights[1] * dataset[i].gyro_z);
                float prediction = sigmoid(z);

                float error = prediction - dataset[i].label;
                
                bias -= learning_rate * error;
                weights[0] -= learning_rate * error * dataset[i].accel_x;
                weights[1] -= learning_rate * error * dataset[i].gyro_z;
            }

            delay(1);
        }
        Serial.println("[ML] Training complete!");
        Serial.printf("[ML] Weights: W1=%.4f, W2=%.4f, Bias=%.4f\n", weights[0], weights[1], bias);
    }

    void evaluate(const TrainingSample* dataset, int num_samples, float boost_threshold) {
        int true_positives = 0;
        int true_negatives = 0;
        int false_positives = 0;
        int false_negatives = 0;

        for (int i = 0; i < num_samples; i++) {
            float prob = predict(dataset[i].accel_x, dataset[i].gyro_z);
            
            bool predicted_straight = (prob >= boost_threshold);
            bool actual_straight = (dataset[i].label >= 0.5f);

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

    float predict(float accel_x, float gyro_z) {
        float z = bias + (weights[0] * accel_x) + (weights[1] * gyro_z);
        return sigmoid(z);
    }

private:
    static const int NUM_FEATURES = 2;
    float weights[NUM_FEATURES];
    float bias;
    float learning_rate;

    float sigmoid(float z) {
        z = constrain(z, -20.0f, 20.0f);
        return 1.0f / (1.0f + exp(-z));
    }

    void shuffleDataset(TrainingSample* dataset, int num_samples) {
        for (int i = num_samples - 1; i > 0; i--) {
            int j = random(0, i + 1);
            TrainingSample temp = dataset[i];
            dataset[i] = dataset[j];
            dataset[j] = temp;
        }
    }
};