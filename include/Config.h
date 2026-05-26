#pragma once

// ============================================================================
// PINS CONFIGURATION (Hardware Layout)
namespace Pins {
    // H-Bridge and Motors
    constexpr uint8_t HB_STANDBY = 3;

    constexpr uint8_t LEFT_MOTOR_IN1 = 21;
    constexpr uint8_t LEFT_MOTOR_IN2 = 20;
    constexpr uint8_t LEFT_MOTOR_PWM = 10;
    
    constexpr uint8_t RIGHT_MOTOR_IN1 = 1;
    constexpr uint8_t RIGHT_MOTOR_IN2 = 2;
    constexpr uint8_t RIGHT_MOTOR_PWM = 0;

    // I2C Bus
    constexpr uint8_t I2C_SDA = 8;
    constexpr uint8_t I2C_SCL = 9;

    // IR Line Sensors
    constexpr uint8_t OUTER_LEFT_IR_SENSOR = 99;
    constexpr uint8_t INNER_LEFT_IR_SENSOR = 6;
    constexpr uint8_t INNER_RIGHT_IR_SENSOR = 7;
    constexpr uint8_t OUTER_RIGHT_IR_SENSOR = 98;
}


// ============================================================================
// PERFORMANCE TUNING (Change often during tests)
namespace Tuning {
    // --- Motor balancing (Trim) ---
    // If the car drifts slightly left on a straight, lower RIGHT_MOTOR_TRIM (e.g. 0.95)
    // If it drifts slightly right, lower LEFT_MOTOR_TRIM.
    constexpr float LEFT_MOTOR_TRIM = 1.00f;
    constexpr float RIGHT_MOTOR_TRIM = 1.00f;

    // --- Target Speeds (From 0.00 to 1.00) ---
    constexpr float BASE_SPEED = 0.50;  // Base speed 
    constexpr float CURVE_SPEED = 0.75;     // Aggressiveness of searching turns
    constexpr float CROSSROAD_SPEED = 0.55; // Driving speed when blindly crossing gaps

    // --- Gyro Intersection Controller ---
    constexpr float KP_GYRO = 0.015;        // Proportional correction weight for blind cross

    static constexpr float KP_LINE = 0.4f;
    static constexpr float KI_LINE = 0.0f;
    static constexpr float KD_LINE = 0.05f;

    // --- Race Timers & Protection (Milliseconds) ---
    constexpr unsigned long STARTING_DELAY_MS = 3000;       // Delay before the car starts
    constexpr unsigned long LAP_END_SAFEGUARD_MS = 30000;   // Ignore finish line for 30s
    constexpr unsigned long CROSSING_TIMEOUT_MS = 50;       // Time spent blind-driving over crossroad
    constexpr unsigned long LOST_LINE_TIMEOUT_MS = 500;     // White/White timeout before SEARCHING state
    constexpr unsigned long WAITING_TIME_MS = 10000;
    constexpr unsigned long SEARCH_SAFETY_TIMEOUT_MS = 10000;

    static const uint8_t DO_SECOND_LAP = 0;
}


// ============================================================================
// SYSTEM CONSTANTS (Set once, do not change unless hardware changes)
namespace Config {
    // Sensor Logic Hardware Check
    // Set to HIGH if sensor outputs HIGH on Black. Set to LOW if it outputs LOW on Black.
    static const uint8_t LINE_DETECT_STATE = HIGH;

    // I2C Configuration
    constexpr int I2C_SPEED_HZ = 400000;    // 400KHz
    constexpr int BMI160_I2C_ADDR = 0x69;


    // IMU data scales and noise filtering
    static constexpr int PWM_MAX = 255;
    static constexpr float GYRO_SCALE = 16.384f;        // For range: +-2000º/s
    static constexpr float ACCEL_SCALE = 16384.0f;      // For range: +-2g
    static constexpr float ACCEL_SMOOTH_ALPHA = 0.2f;   // Lower = smoother but slower response
    static constexpr float DEADZONE_THRESHOLD = 1.2f;   // Filters out small stationary sensor ticks

    static constexpr float GYRO_NORM_FACTOR = 1000.0f;  // Normalize Gyro for the model
    static constexpr float US_TO_SEC = 1000000.0f;

    // Speed Controller Logic
    static constexpr float PID_DT_FALLBACK = 0.001f;    // Minimal dt for first loop
}


// ============================================================================
// MACHINE LEARNING & BOOST SETTINGS
namespace ML {
    // Data Gathering
    constexpr int MAX_SAMPLES = 2500;                   // 2500 samples = ~30KB of SRAM
    constexpr unsigned long SAMPLE_INTERVAL_MS = 30;    // 30ms * 2500 = 75 seconds of recording time

    // Training Hyperparameters
    constexpr float LEARNING_RATE = 0.05f;              // Step size for Gradient Descent
    constexpr int TRAINING_EPOCHS = 200;                // How many times to loop over the data at the finish line

    // Racing Logic
    constexpr float CONFIDENCE_THRESHOLD = 0.85f;       // Requires 85% safety probability to trigger boost
    constexpr float BOOST_SPEED = 1.0f;                 // Max speed injected on straightaways
    constexpr uint8_t MIN_CYCLES_TURBO = 5;             // Minimum cycles required to run on turbo
}