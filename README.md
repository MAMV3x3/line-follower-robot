# Linefollower Robot with PID Control using Arduino Nano

This repository contains the code and schematics for a linefollower robot, which uses PID control to maintain a straight path. The robot is built around an Arduino Nano microcontroller, which interfaces with a driver TB6612FNG and QTR-8A sensors for motor control and line detection, respectively. The robot's motion is powered by Pololu 10:1 3000 RPM motors, which provide precise and fast movement.

The repository includes the Arduino code for the robot, which is written in C++ and uses the PID library for control. The code is well-documented and easy to modify for different use cases. Additionally, the gerber files the robot's PCB are provided, along with a detailed parts list.

This project is ideal for anyone interested in robotics, embedded systems, or control theory. By building and programming this robot, you will gain hands-on experience with PID control, motor control, and sensor interfacing. The robot is also a fun and engaging way to learn about linefollowing and autonomous navigation.

## Hardware used

- PID control for precise and stable linefollowing
- Arduino Nano microcontroller for flexibility and expandability
- TB6612FNG motor driver for high-power motor control
- QTR-8A sensors for accurate line detection
- Pololu 10:1 3000 RPM motors for fast and agile movement 

## Contents

- Arduino code for linefollower robot in 'line-follower-robot/line_follower_PID.ino'
- PCB gerber files in 'line-follower-robot/PCB gerber files'

## How to Use

1. Clone this repository to your local machine.
2. Open 'line_follower_PID.ino' file in the Arduino IDE.
3. Connect your Arduino board to your computer and upload the code to the board.
4. Power on the robot and place it on a line to start following it.

## License

This project is licensed under the [MIT License](https://opensource.org/license/mit/). See the 'LICENSE' file for details.
