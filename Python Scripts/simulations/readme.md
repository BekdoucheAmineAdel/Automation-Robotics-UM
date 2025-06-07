# Automation & Robotics Simulations

This repository contains a collection of Python simulations and control implementations for various systems in automation and robotics. It includes nonlinear pendulum control, robotic arm kinematics, mobile robot navigation, DC motor modeling, and observer design.

---

## Project Structure

- pendulum/
  - nonlinear_pendulum_observer_simulation.py: Nonlinear inverted pendulum simulation with observer and state feedback control.

- robotic_arm/
  - two_link_arm_simulation.py: Two-link robotic arm simulation with inverse kinematics and trajectory tracking.

- mobile_robot/
  - robot_pose_control_simulation.py: Mobile robot pose control using unicycle kinematics and Lyapunov-based controllers.

- dc_motor/
  - mcc_continuous_discrete_simulation.py: DC motor (MCC) simulation in both continuous and discrete domains.
  - mcc_positioning_control_simulation.py: Positioning control of MCC using pole placement and LQR.

- observers/
  - continuous_observer_synthesis.py: Continuous-time observer synthesis for linear systems.

---

## Setup Instructions

Install the required dependencies using:

pip install -r requirements.txt

---

## Usage

Each script is standalone and can be run directly:

python path/to/script.py

For example:

python pendulum/nonlinear_pendulum_observer_simulation.py

---

## Features

- State-space modeling and simulation
- Observer design (Kalman, Luenberger)
- Pole placement and LQR control
- Inverse kinematics for robotic arms
- Path following and pose control for mobile robots
- Continuous and discrete-time system analysis

---

## Author

Bekdouche Amine  

---
