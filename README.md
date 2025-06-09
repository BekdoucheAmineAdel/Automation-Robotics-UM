# Automation-Robotics-UM

This repository contains laboratory work, projects, and practical code realted to automation and robotics developed during my studies at the University of Montpellier, with a focus on Automation and Robotics.

## Repository Structure

The main contents of this repository include:

- **Python Scripts/**
  - [`simulations`](Python%20Scripts/simulations)
    - [`pendulum`](Python%20Scripts/simulations/pendulum): Nonlinear inverted pendulum simulation with observer and state feedback control.
    - [`robotic_arm`](Python%20Scripts/simulations/robotic_arm): Two-link robotic arm simulation with inverse kinematics and trajectory tracking.
    - [`mobile_robot`](Python%20Scripts/simulations/mobile_robot): Mobile robot pose control using unicycle kinematics and Lyapunov-based controllers.
    - [`dc_motor/`](Python%20Scripts/simulations/dc_motor): DC motor (MCC) simulation and positioning control, including both continuous and discrete-time modeling.
    - [`observers`](Python%20Scripts/simulations/observers): Observer synthesis for linear systems.
    - [`requirements.txt`](Python%20Scripts/simulations/requirements.txt): All dependencies (numpy, scipy, matplotlib, control, sympy) needed to run the simulations.
    - [`readme.md`](Python%20Scripts/simulations/readme.md): Detailed documentation of simulation contents and sample results.
  - [`UARM Swift Pro - Controller`](Python%20Scripts/UARM%20Swift%20Pro%20-%20Controller): Scripts and models for controlling the UARM Swift Pro robotic arm, including forward/inverse kinematics and command scripts to interface with the real robot.

## Example Projects

- **Pendulum Control**: Nonlinear dynamics, observer-based feedback, simulation of pendulum motion and control strategies.
- **Robotic Arm**: Two-link manipulator with kinematics, trajectory planning, and visualization.
- **Mobile Robot**: Pose control using unicycle models and Lyapunov approaches; trajectory tracking and simulation.
- **DC Motor**: State-space modeling, continuous/discrete simulations, position control using pole placement and LQR.
- **Observer Design**: Continuous-time observer synthesis for state estimation in linear systems.
- **UARM Swift Pro**: Real-world and simulated control of a robotic arm, including communication with the hardware and computation of kinematics.

## Technologies & Libraries

- Python 3.x
- numpy, scipy, matplotlib, control, sympy
- [uArm Swift API](https://github.com/uarm-developer/python-uarm) for real robot control

## Getting Started

1. Install Python 3.x and the required libraries:  
   `pip install -r Python Scripts/simulations/requirements.txt`
2. Explore each project folder and run the scripts for the simulation or control task you are interested in.

## Notes

- Each folder contains source code, documentation, and sometimes sample plots or results.
- For more details, see the README file within simulation or projects subfolder.
- The content focuses on practical applications and simulation of core robotics and automation concepts learned during the academic year.
