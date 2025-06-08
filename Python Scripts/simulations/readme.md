![Python](https://img.shields.io/badge/Python-3.13.3-blue?logo=python&logoColor=white)
# Automation & Robotics Simulations

This repository contains a collection of Python simulations and control implementations for various systems in automation and robotics. It includes nonlinear pendulum control, robotic arm kinematics, mobile robot navigation, DC motor modeling, and observer design.

## Repository Structure

- [`pendulum`](pendulum): Nonlinear inverted pendulum simulation with observer and state feedback control.
- [`robotic_arm`](robotic_arm): Two-link robotic arm simulation with inverse kinematics and trajectory tracking.
- [`mobile_robot`](mobile_robot): Mobile robot pose control using unicycle kinematics and Lyapunov-based controllers.
- [`dc_motor`](dc_motor): DC motor (MCC) simulation in both continuous and discrete domains. Positioning control of MCC using pole placement and LQR.
- [`observers`](observers): Continuous-time observer synthesis for linear systems.

## Getting Started

1. **Prerequisites**
   - Python 3.x
   - Libraries listed in `requirements.txt`:
     - numpy
     - scipy
     - matplotlib
     - control
     - sympy

2. **Usage**
   - Clone the repository:
     git clone https://github.com/yourusername/automation-robotics-simulations.git
   - Navigate to the appropriate folder and run the desired script:
     python path/to/script.py

## Projects Overview

Each folder contains Python source files for a specific simulation or control project. Example projects include:
- **Pendulum Control:** Nonlinear dynamics and observer-based feedback.
- **Robotic Arm:** Kinematics and trajectory tracking for a two-link manipulator.
- **Mobile Robot:** Pose control using unicycle model and Lyapunov-based strategies.
- **DC Motor Control:** Continuous and discrete-time modeling and control of a DC motor.
- **Observer Design:** Continuous-time observer synthesis for linear systems.

## Contributing

This repository serves as a learning resource. If you have suggestions or improvements, feel free to open an issue or submit a pull request.

## License

Distributed for educational purposes.
