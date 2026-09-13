# Autonomous Navigation Robot

Autonomous mobile robot developed for ENGR 122 at Stevens Institute
of Technology. The robot uses MQTT-based position tracking, ultrasonic
obstacle detection, and closed-loop heading correction to autonomously
navigate through a sequence of target coordinates.

## Features

- Autonomous waypoint navigation
- Real-time position updates over MQTT
- Three-sensor ultrasonic obstacle avoidance
- Dynamic heading correction
- Distance-based speed control
- Oscillation detection and recovery
- Stuck detection and recovery
- OLED telemetry display

## System Overview

The robot receives its X/Y position and orientation through MQTT.
It calculates the direction and distance to the current waypoint and
adjusts the drive motors accordingly.

Navigation priority:

1. Obstacle avoidance
2. Waypoint arrival detection
3. Stuck recovery
4. Heading alignment
5. Forward movement

## Hardware

- Microcontroller / Wi-Fi development board
- 2 continuous-rotation servo motors
- 3 ultrasonic distance sensors
- SSD1306 OLED display
- Mobile robot chassis

## Navigation Algorithm

For each waypoint, the robot calculates:

distance = sqrt((x_target - x_robot)^2 + (y_target - y_robot)^2)

The desired heading is calculated using atan2(), and the robot compares
it with its measured orientation.

The acceptable heading error becomes smaller as the robot approaches
the target.

## Obstacle Avoidance

Three ultrasonic sensors monitor the front, left, and right sides of
the robot.

The robot prioritizes avoiding nearby obstacles before resuming
waypoint navigation.

## Recovery Logic

The program includes two recovery systems:

- Oscillation detection prevents repeated left-right corrections.
- Stuck detection commands a short forward movement if the robot's
  position stops changing.

## Course

ENGR 122 – Stevens Institute of Technology
Final Project

## Authors

Parker Robles
