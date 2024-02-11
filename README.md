# ws23-feedback-pouring
Pouring the liquid/cereals using the Kinova arm with weight estimation feedback


## Introduction

This project demonstrates the implementation of feedback-controlled pouring using a Kinova robotic arm and the Kortex API. 
The goal is to accurately pour the specified amount by estimating the force from ChainExternalWrenchEstimator and using IMU acceleration to calculate the weight. 

This repository is developed by the second semester SS23 batch of Autonomous Systems at Hochschule Bonn-Rhein-Sieg. The code in this repository is Kortex API-based and is developed using a single Kinova arm.

## Dependencies

This repository depends on the below components. 

`Kinova Robotic Arm`
`Kinova Kortex API`
`[Orocos KDL (Kinematics and Dynamics Library)](https://github.com/orocos/orocos_kinematics_dynamics/tree/master)`

## Technologies
- Robot Platform: [KINOVA](https://www.kinovarobotics.com/resources)
- OS: Ubuntu 20.04 / 22.04 LTS
- Middleware: Kinova Kortex API
- Libraries: Kinematics and Dynamics Libraries (KDL)

## Setup and Usage of Customized Package
Please refer to the detailed steps provided to set the environment and use it for the project on the [Wiki page](https://github.com/HBRS-SDP/ws23-feedback-pouring/wiki)

## Architecture Overview
![Architecture Workflow](docs/SDP%20-%20Architecture%20Workflow-2.jpg)

## Getting started after the environment setup

- Weigh the bottle using any weighting system manually, and record the weight. 
- Provide the bottle to the gripper using the help of Joystick(open/close gripper operation).
- Run the executable file, and give the user command for the amount of liquid/cereal to be poured.
- Wait for the execution to complete the pouring task.
- Remove the bottle and weigh again using the weighing system manually.
- To validate the desired pouring completion, compare the manual weight measurement with the API weight measurement. 



## Acknowledgments

The guides who supported and mentored us throughout this journey were: 
* Minh Nguyen, Djordje Vukcevic, Santosh Thoduka, Vamsi Kalagaturu , Kishan Sawant 
* the MAS staff and professors who have provided their advice and support
