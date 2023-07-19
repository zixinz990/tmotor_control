# T-Motor Controller Scripts for Position Control - Spring Impact Study

![Teensy and T-Motor](teensy_t_motor.jpg)

## Table of Contents

1. [Introduction](#introduction)
2. [Project Overview](#project-overview)
3. [Hardware Requirements](#hardware-requirements)
4. [Software Requirements](#software-requirements)
5. [Installation](#installation)
6. [Usage](#usage)
7. [Contributing](#contributing)
8. [License](#license)

## 1. Introduction

Welcome to the T-Motor Controller Scripts for Position Control ReadMe! This repository contains the necessary scripts and resources to control a T-Motor using a Teensy board for precise position control. The primary goal of this project is to study the impact of springs on a robot leg's jumping performance versus the power consumed during jumping.

## 2. Project Overview

Robotics enthusiasts, researchers, and engineers often use T-Motors to drive robot joints with high precision and power. However, the inclusion of springs in leg mechanisms can introduce complexities in controlling leg movement and affect energy consumption during locomotion. This study aims to investigate how the incorporation of springs in a robot leg affects its jumping performance, energy efficiency, and overall stability.

In this project, we use a Teensy board, which provides real-time control capabilities, to accurately position the T-Motor and simulate various spring configurations for the robot leg. By collecting data on jumping height, power consumption, and other relevant metrics, we can gain valuable insights into the trade-offs involved in designing spring-loaded robot legs.

## 3. Hardware Requirements

To replicate this project, you will need the following hardware components:

- T-Motor: [Insert specific T-Motor model here]
- Teensy Board (e.g., Teensy 4.0 or newer)
- Robot Leg Mechanism (with spring attachment points)
- Power Supply for T-Motor and Teensy
- Motor Driver or ESC compatible with T-Motor
- Sensors for data collection (e.g., IMU, force sensor, or encoder)
- Jumper wires and cables

## 4. Software Requirements

The following software tools are necessary to run the controller scripts:

- Arduino IDE (for Teensy programming)
- Teensyduino Library (to integrate Teensy with Arduino IDE)
- T-Motor Control Library (specific library for your T-Motor model)

## 5. Installation

To set up the project environment, follow these steps:

1. Install Arduino IDE: Download and install the latest version of the Arduino IDE from the official Arduino website (https://www.arduino.cc/en/software).

2. Install Teensyduino Library: Follow the instructions on the PJRC website (https://www.pjrc.com/teensy/teensyduino.html) to install the Teensyduino add-on for the Arduino IDE.

3. Obtain the T-Motor Control Library: Get the specific library required to control your T-Motor model from the T-Motor official website or GitHub repository.

4. Clone this repository: Clone this project repository to your local machine using the following command:
  - https://github.com/zixinz990/tmotor_control.git 
5. Upload the Scripts: Open the Arduino IDE, load the main controller script, and upload it to your Teensy board.

## 6. Usage

Follow these steps to conduct the spring impact study:

1. Assemble the Robot Leg: Build the robot leg mechanism and attach the T-Motor with the chosen spring configuration.

2. Power up the System: Connect the Teensy board, T-Motor, motor driver, and sensors to the power supply.

3. Calibrate Sensors: Ensure that all sensors are calibrated properly before starting the experiments.

4. Run Experiments: Execute the controller script on the Teensy board to control the T-Motor's position. Record relevant data such as jumping height, power consumption, and any other metrics of interest.

5. Change Spring Configurations: Repeat the experiments for different spring configurations to collect sufficient data for analysis.

6. Analyze Data: Use appropriate data analysis tools (e.g., MATLAB, Python) to process and visualize the collected data.

7. Interpret Results: Analyze the data to draw conclusions about the impact of springs on robot leg jumping and power consumption.

## 7. Contributing

We welcome contributions to this project! If you find any issues or want to enhance the controller scripts, feel free to create a pull request. Please adhere to the standard code style and documentation guidelines.

## 8. License

The T-Motor Controller Scripts for Position Control - Spring Impact Study project is licensed under the [MIT License](LICENSE.md). You are free to use, modify, and distribute the code as per the terms of this license.

---

Thank you for your interest in our project! If you have any questions or need further assistance, please don't hesitate to contact us. Happy experimenting!
