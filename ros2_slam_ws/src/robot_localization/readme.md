# Robot Localization Package

## Role in Project
**Core sensor fusion engine**. Implements Extended Kalman Filter to combine wheel encoder and IMU data into unified robot pose estimate. Configured via our `config/ekf.yaml`.

## Installation Note
RPi5 requires source build with custom shell script. Consider swap allocation and memory limits during compilation.

## Repository
https://github.com/cra-ros-pkg/robot_localization