# SAR Multi-Robot Waste Detection System

### Autonomous Waste Detection, Mapping, Navigation and Multi-Agent Task Allocation

---

## 1. Overview

This project implements a ROS 2 based multi-robot Search and Rescue / waste-detection framework for autonomous exploration of an unknown indoor environment.

The system is designed around **two autonomous mobile robots** operating inside a simulated warehouse environment. Each robot is equipped with:

- 2D LiDAR
- RGB camera
- IMU
- Wheel odometry
- Differential-drive motion

The robots explore the environment, detect waste/target objects represented by ArUco markers, report detected targets to a shared database, and receive navigation assignments from a centralized task allocator.

The overall system is designed to follow the pipeline:

```text
                    ┌───────────────────────┐
                    │     Gazebo World      │
                    │   Warehouse / Maze    │
                    └───────────┬───────────┘
                                │
                    ┌───────────▼───────────┐
                    │       Sensors         │
                    │ Camera / LiDAR / IMU  │
                    │       / Odometry      │
                    └───────────┬───────────┘
                                │
                ┌───────────────┴────────────────┐
                │                                │
        ┌───────▼────────┐             ┌────────▼────────┐
        │   Perception   │             │   Localization  │
        │                │             │                  │
        │ ArUco Detector │             │ SLAM / TF / Odom │
        └───────┬────────┘             └────────┬─────────┘
                │                               │
                │ Detected Target               │ Robot Pose
                │                               │
                └───────────────┬───────────────┘
                                │
                       ┌────────▼─────────┐
                       │  Target Database │
                       │                  │
                       │ Deduplication    │
                       │ Target Storage   │
                       └────────┬─────────┘
                                │
                         Available Targets
                                │
                       ┌────────▼─────────┐
                       │ Task Allocation  │
                       │                  │
                       │ Nearest Robot    │
                       └────────┬─────────┘
                                │
                    ┌───────────┴───────────┐
                    │                       │
             ┌──────▼──────┐         ┌──────▼──────┐
             │   Robot 1   │         │   Robot 2   │
             │ Navigation  │         │ Navigation  │
             └──────┬──────┘         └──────┬──────┘
                    │                       │
                    └───────────┬───────────┘
                                │
                         ┌──────▼──────┐
                         │    Nav2     │
                         │ Planning +  │
                         │ Control     │
                         └──────┬──────┘
                                │
                         Robot Motion
