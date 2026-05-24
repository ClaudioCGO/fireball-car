# Firmware Documentation: Fireball v0.5

This firmware controls an ESP32-C3 based line-following robot. It utilizes a PID control loop for navigation, a finite state machine for decision-making, and a Logistic Regression model to implement "turbo" boosts on straightaways during the second lap.

---

## Operational Workflow

The robot follows a specific state cycle from power-up to race completion.

### 1. Initialization (Setup)

* System initializes I2C bus and verifies the BMI160 IMU.
* The system executes a 3-second startup delay, allowing the user to place the robot on the track.
* The robot enters the `FOLLOWING` state and begins the first lap.

### 2. Lap 1: Data Gathering & Navigation

* The robot follows the line using PID control (`KP_LINE`, `KI_LINE`, `KD_LINE`).
* During this phase, the robot actively stores motion data (Accelerometer X, Gyroscope Z) and the corresponding label (whether it is on a straight or curve) into the `DataGatherer`.
* The `LAP_END_SAFEGUARD_MS` prevents the robot from prematurely detecting the finish line due to noise.

#### Handling Intersection Crossings (`CROSSING` State)

When the robot detects the finish line or a crossroad (both IR sensors return `HIGH`/`LINE_DETECT_STATE`), it enters the `CROSSING` state. To traverse the gap:

* The robot locks its current target heading using the IMU yaw data.
* It maintains a constant velocity (`SPEED_CROSSROAD`) for the duration of `CROSSING_TIMEOUT_MS`.
* A Proportional controller using `KP_GYRO` applies corrections based on yaw drift, ensuring the robot drives straight across the intersection without veering.

#### Recovery & Search Logic (`SEARCHING` State)

If the robot leaves the track (both sensors return `LOW`/no-line), it initiates the `SEARCHING` state after the `LOST_LINE_TIMEOUT_MS` has elapsed:

* **Retrieval Logic:** The robot references `last_line_seen` to determine if the line was last detected on the left or right side.
* **Maneuver:** It performs a pivot turn in the direction of the last detected line at `SPEED_CURVE` to attempt re-acquisition.
* **Safety:** If the line is not re-acquired within `SEARCH_SAFETY_TIMEOUT_MS`, the motors are placed in `STANDBY` mode to protect the hardware.

### 3. Transition: Lap 1 to Lap 2

* Upon detecting the finish line (both sensors high), the robot enters the `WAITING` state.
* The robot performs **Logistic Regression training** using the data collected in Lap 1.
* It calculates weights (`W1`, `W2`) and bias.
* After the training delay (`WAITING_TIME_MS`), if `Tuning::DO_SECOND_LAP` is enabled, the robot resets the PID controller and begins Lap 2.

### 4. Lap 2: Inference and Boosting

* The robot follows the line using PID control.
* For every control cycle, the system passes current IMU data through the trained ML model.
* The model outputs a probability score. If the probability exceeds `ML::CONFIDENCE_THRESHOLD` and the logic detects a straight (error == 0.0), the system injects `ML::BOOST_SPEED` into the motor command, bypassing the standard speed.

---

## Configuration Reference (`Config.h`)

Modify the following variables based on your physical testing.

### Mechanical Tuning

Adjust these when the robot does not track straight or drifts.

* `Tuning::LEFT_MOTOR_TRIM` / `Tuning::RIGHT_MOTOR_TRIM`: Calibration multipliers (0.0 to 1.0). Use to balance motor power.
* `Tuning::SPEED_STRAIGHT`: Base velocity for straight segments.
* `Tuning::SPEED_CURVE`: Base velocity for turns (lower than straight).
* `Tuning::SPEED_CROSSROAD`: Velocity when navigating over a crossing gap.

### PID Controller Tuning

Adjust these to improve line-following stability.

* `Tuning::KP_LINE`: Proportional gain. Increase if the robot reacts too slowly to the line.
* `Tuning::KD_LINE`: Derivative gain. Increase if the robot oscillates/wobbles excessively.
* `Tuning::KI_LINE`: Integral gain. Use if there is a persistent steady-state error.

### Race Logic and Protection

* `Tuning::LAP_END_SAFEGUARD_MS`: Minimum time to pass before the finish line can be detected.
* `Tuning::LOST_LINE_TIMEOUT_MS`: Time duration to wait after losing the line before entering `SEARCHING` state.
* `Tuning::DO_SECOND_LAP`: Set to `1` to enable the second lap; set to `0` to stop after the first.

### Machine Learning Parameters

These affect the training efficiency and the strictness of the turbo boost.

* `ML::CONFIDENCE_THRESHOLD`: Probability (0.0 - 1.0) required to trigger a boost. Increase for more conservative driving.
* `ML::BOOST_SPEED`: Motor speed injected during straightaways (typically 1.0).
* `ML::LEARNING_RATE`: Step size for training.
* `ML::TRAINING_EPOCHS`: Number of iterations for gradient descent.

---

## Technical Considerations

* **Loop Frequency:** The `loop()` function is capped at 500Hz (2ms) using `micros()` to ensure consistent PID calculation and IMU data sampling.
* **Sensor Logic:** `Config::LINE_DETECT_STATE` defines the signal level for the black line. If your sensors output `LOW` on black, change this constant to `LOW`.
* **IMU Deadzone:** `Config::DEADZONE_THRESHOLD` filters out stationary noise. If the robot drifts while stopped, increase this value.