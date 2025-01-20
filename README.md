# RoboCon 2025 (Team Utkarsh Codes) 


## Programming Department Guide & Tips

Welcome to the RoboCon 2025 Programming Department! This guide is designed to provide structure and direction for all contributors working on the robot’s programming. The information is divided into different sections to help you focus on key areas and complete your tasks efficiently.

## Table of Contents
1. [Basic Codes](#basic-codes)
2. [Code Modules](#code-modules)
3. [Pro Logic Codes](#pro-logic-codes)
4. [Final Tip](#final-tip)

---

## 1) Basic Codes
Here is a list of essential code examples and snippets for the basic robot operations:

- **Basic Drive**: 
    - Autonomous Drive
    - Wireless Drive
    - 360° Rotation (Optional)

- **TF Mini Plus**: Setup and calibration for distance measurements.

- **IMU (Inertial Measurement Unit)**: 
    - Bot Stabilization
    - θ° (Angle) Measurement

- **Encoders**: 
    - RPM (Revolutions Per Minute)
    - Angle Measurement
    - Positional Data

- **BLDC (Brushless DC Motor)**:
    - Simple BLDC Control
    - RPM Feedback for Flywheel

- **Stepper Motor**:
    - Basic Stepper Control
    - Encoder Feedback for Angle Measurement

- **Limit Switches & Sensors**:
    - IR Sensors
    - Mechanism Stopping for Ball Interaction

- **Wireless Control**:
    - BLE (Bluetooth Low Energy)
    - Wi-Fi
    - Zigbee (or other communication protocols)

---

## 2) Code Modules
Code modules are designed to handle specific subsystems of the robot. Below are the modules you will be working on:

### Drive
- **Wireless Control**: 
    - Simple wireless control for the robot.
    - Advanced wireless control with sensor fusion (integration of sensor data for accurate navigation).

- **Autonomous Drive**:
    - Self-positioning control using sensor data for autonomous navigation.

### Roller Mechanism
- **Roller Angle Control**: 
    - Control the angle of the roller mechanism for ball intake and other interactions.

- **Ball Intake Control**: 
    - Automated intake system for balls, with precise control and feedback.

### Elevator Mechanism
- **Chain Elevator Operation**: 
    - Control system for chain-based elevator mechanisms.

- **Controlled Movements with Limit Protections**: 
    - Safety protocols to ensure controlled and restricted movements for the elevator.

### Shooting Mechanism
- **Flywheel RPM Control**: 
    - Real-time RPM adjustment based on shooting distance.

- **Angle Variation Control**: 
    - Adjust the shooting angle based on ball trajectory and distance.

### Dribbling Mechanism
- **Dribble Control**: 
    - Implement the control for dribbling ball interactions.

---

## 3) Pro Logic Codes
These code structures are focused on implementing essential logic and control features for the robot:

- **Real-Time Control**: 
    - Implement the logic for handling time-sensitive tasks, ensuring timely responses.

- **Failsafe Timer**: 
    - Safety mechanism to ensure the robot does not continue operations if a critical failure occurs.

- **Position & Orientation Control**: 
    - Accurate control of the robot’s position and orientation using sensors and feedback mechanisms.

- **PID Controller**: 
    - Implement PID (Proportional, Integral, Derivative) controllers to fine-tune movements and behaviors.

- **PID Drive (Optional)**: 
    - Enhance robot drive performance using PID for smooth navigation.

---

## 4) Final Tip
*"You’re not just coding, you’re teaching the robot to dunk like Jordan—just without the sneakers!"*

Remember, every line of code you write brings us closer to success in RoboCon 2025. Make sure to keep the logic clear, write maintainable code, and most importantly, have fun!

---

## How to Contribute

1. **Fork the Repository**: Create a personal fork of this repository to work on your changes.
2. **Create a New Branch**: Always work on a new branch for each feature/module you are implementing.
3. **Submit a Pull Request**: Once your code is ready and tested, submit a pull request with a description of your changes.

---

Thank you for your contribution, and let's make RoboCon 2025 the best yet!
