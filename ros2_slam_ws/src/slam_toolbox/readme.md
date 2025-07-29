# SLAM Toolbox Package

## Role in Project
**Primary mapping component**. Processes single-point distance sensor data to build occupancy grid maps. Heavily customized via `config/slam.yaml` to work with sparse sensor data instead of 360° laser scanners.

## Installation Note
RPi5 requires manual source compilation. Use shell script with swap management - SLAM toolbox is memory-intensive during build.

## Repository
https://github.com/SteveMacenski/slam_toolbox