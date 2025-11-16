# Line Follower Robot (ESP32 + PID + 8x IR Sensors)

This project is an autonomous **line follower robot** built around an **ESP32** microcontroller, using an **8-channel IR sensor array** and a **PID control algorithm** to follow a black line on a white surface.

The goal was to design a **fast, low-cost and reasonably stable** robot that can:

- Track a black line using multiple IR sensors  
- Correct its trajectory in real time using PID control  
- Handle curves and moderate turns without oscillating too much  
- Detect a finish line and **stop automatically**

The firmware is written in **C/C++ (Arduino framework)** for the ESP32.

---

## Main Features

- **ESP32-based control**  
  - High-performance microcontroller with enough speed for real-time sensor processing and PWM motor control.

- **8x IR Line Sensors (HY-S301)**  
  - IR sensor array mounted at the front of the robot  
  - Reads surface reflectivity to distinguish black line vs white background  

- **Custom Line Position Calculation**  
  - Uses the `QTRSensors` library to read sensor values  
  - Instead of the built-in `readLineBlack()`, the code computes a **weighted line position** using a custom position array:  
    `{-1.0, -0.714, -0.429, -0.143, 0.143, 0.429, 0.714, 1.0}`  
  - Sensors above a threshold (e.g. `threshold = 4000`) are considered to “see” the line  
  - The active sensors contribute to a weighted average, giving a **continuous position** of the line between -1 (left) and +1 (right)

- **PID Control Loop**  
  - Error is defined as the **negative** of the line position:  
    `error = -position`  
  - This makes the robot correct in the opposite direction of the deviation  
  - Uses PID terms:  
    - **Kp** (proportional): reacts to current error (e.g. `Kp = 280`)  
    - **Ki** (integral): disabled in this implementation (`Ki = 0`)  
    - **Kd** (derivative): damps oscillations by reacting to the **rate of change** of the error  
  - PID output (`correction = Kp * error + Ki * integral + Kd * derivative`) is used to adjust the PWM speed of the left and right motors, keeping the robot on the line  

- **Fail-Safe / Line Lost Handling**  
  - If no sensor detects the line (`sensorDetectCount = 0`), the robot uses the **last known sensor position** (`lastSensorDetect`) to guess the direction and try to recover the line  

- **Finish Line Detection**  
  - When more than **4 sensors** detect black simultaneously, the robot assumes it reached a **horizontal finish line**  
  - Both motors are stopped completely  

---

## Hardware Overview

- **Microcontroller**: ESP32 Development Board  
- **Line Sensors**: 8-channel IR tracking sensor module (HY-S301)  
- **Motor Driver**: DRV8833 dual motor driver  
- **Motors**: N20 DC gear motors, 12V, 1000 RPM  
- **Wheels**:  
  - 4× rubber wheels (rear, attached to the motors)  
  - 1× steel ball caster (front support)  
- **Power**:  
  - 3S1P Lithium battery (~12V)  
  - Step-down converter to **10.8–12V** for the motor driver  
  - Second step-down converter to **5V** for the ESP32 and sensor array  

Total hardware cost is roughly **~16€**, making the robot affordable for student projects and basic robotics experiments.

---

## How It Works (High-Level)

1. **Read sensors**  
   The ESP32 reads all 8 IR sensors via the `QTRSensors` library.

2. **Detect active sensors**  
   Each reading is compared to a threshold. Values above the threshold are treated as “line detected”.

3. **Compute line position**  
   - Multiply each sensor’s weight (from the position array) by whether it detects the line  
   - Sum the weighted positions and divide by the number of active sensors  
   - Result: a **normalized line offset** between -1 and +1  

4. **Compute PID correction**  
   - Calculate error from line position  
   - Update PID (proportional + derivative, integral off)  
   - Compute a correction value  

5. **Adjust motor speeds**  
   - Base speed is set the same for both motors  
   - PID correction is added/subtracted to left/right motor speed  
   - This makes the robot turn smoothly back towards the line  

6. **Check stop condition**  
   - If more than 4 sensors see black at once → stop the motors (finish line).  

---

## Results

- The robot successfully follows **simple and medium-difficulty tracks** with good stability  
- PID tuning provides **smooth corrections** without aggressive zig-zagging  
- Finish line detection works reliably when the line is clearly horizontal across all sensors  

---

## What We Learned / Future Work

From this project we gained practical experience with:

- Using **ESP32** for mixed analog/digital workflows (sensors + PWM)  
- Implementing and tuning a **PID controller** in a real system  
- Wiring and powering a complete embedded system (battery, step-downs, driver, sensors)  

Possible improvements:

- Better tuning for **very tight curves**, where the robot’s high speed makes it overshoot  
- Adding **configurable PID parameters** (e.g., via serial or Bluetooth)  
- Implementing **adaptive speed control**, slowing down before sharp turns  

---

You can clone this repository, tweak the PID gains, and adapt the code for your own robot chassis, sensor array, or motor configuration.
