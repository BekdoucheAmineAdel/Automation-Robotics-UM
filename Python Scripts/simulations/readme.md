![Python](https://img.shields.io/badge/Python-3.13.3-blue?logo=python&logoColor=white) ![Requirements](https://img.shields.io/badge/Requirements-Updated-limegreen?logo=python&logoColor=white)
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
   - Libraries listed in [`requirements.txt`](requirements.txt):
     - numpy
     - scipy
     - matplotlib
     - control
     - sympy

2. **Usage**
   - Clone the repository:
     git clone https://github.com/BekdoucheAmineAdel/Automation-Robotics-UM.git
   - Navigate to the appropriate folder and run the desired script:
     python path/to/script.py

## Projects Overview

Each folder contains Python source files for a specific simulation or control project. Example projects include:


- **Pendulum Control:** Nonlinear dynamics and observer-based feedback.
   - Pendulum's position over time
![Image](https://github.com/user-attachments/assets/6639ccaf-5605-452f-a889-f66c625db0a6)
   - Position control of the pendulum
![Image](https://github.com/user-attachments/assets/8700d181-3cf4-4fb0-99da-8f414897a7a2)
- **Robotic Arm:** Kinematics and trajectory tracking for a two-link manipulator.
   - Rotation Command
![Image](https://github.com/user-attachments/assets/3c60bf50-463c-4653-9db6-f83304c14296)
   - Position Control
![Image](https://github.com/user-attachments/assets/7cef0caf-31b8-42b3-8dfe-70a7f583d3c2)
- **Mobile Robot:** Pose control using unicycle model and Lyapunov-based strategies.
   - Follow Horizontal Path
![Image](https://github.com/user-attachments/assets/1553f1bb-2b2c-4894-85b8-dbfb22b779aa)
   - Follow Angled Path
![Image](https://github.com/user-attachments/assets/6a93f5e7-094e-4a9e-b483-fdc8ca7b62a9)
   - Position Control
![Image](https://github.com/user-attachments/assets/da073a14-6f81-4889-930d-6eb579478717)
   - Lyapunov Result
![Image](https://github.com/user-attachments/assets/25790d9a-8f80-4c8b-a71a-2fcd93fc605a)
- **DC Motor Control:** Continuous and discrete-time modeling and control of a DC motor.
- **Observer Design:** Continuous-time observer synthesis for linear systems.

## Contributing

This repository serves as a learning resource. If you have suggestions or improvements, feel free to open an issue or submit a pull request.

## License

Distributed for educational purposes.
