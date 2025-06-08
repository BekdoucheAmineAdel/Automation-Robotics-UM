# uArm Swift Pro - 3DOF Robot Manipulator Control

This repository contains code and documentation for modeling and controlling a 3-degrees-of-freedom (3DOF) robotic arm using both geometric modeling and real-time control via the uArm Swift Pro API.

## 🎥 Project Demonstration

[![YouTube](https://img.shields.io/badge/Watch%20on-YouTube-red?logo=youtube&style=for-the-badge)](https://www.youtube.com/watch?v=lZgf7AkHVYs)

## 📄 Files Overview

### 1. `report.pdf`
A comprehensive lab report detailing:
- Theoretical derivation of the direct and inverse geometric models (MGD & MGI)
- Parametric identification of the robot
- Mathematical modeling and validation
- Final implementation results and observations

### 2. `uarm_swift_mgi_model.py`
A Python script implementing:
- Direct Geometric Model (MGD): Computes the end-effector position from joint angles.
- Inverse Geometric Model (MGI): Computes joint angles required to reach a given position.
- Parameter identification using least squares from experimental data.
- Model validation using known positions and angles.

This script is useful for simulation and offline computation of robot kinematics.

### 3. `uarm_swift_controller.py`
A real-time control script that:
- Connects to the uArm Swift Pro via USB using the `SwiftAPI`
- Sends joint angle commands to the robot
- Implements the MGI model for real-time inverse kinematics
- Includes a `robot_animation()` function to execute a sequence of movements with optional vacuum gripper control

This script is intended for live execution on the physical robot.

## 🚀 Getting Started

### Requirements
- Python 3.x
- `numpy`, `matplotlib`
- [`uarm.wrapper`](https://github.com/uArm-Developer/uArm-Python-SDK) Python package (for robot communication)

### Running the Simulation
To test the kinematic models without a robot:
```bash
python3 uarm_swift_mgi_model.py
