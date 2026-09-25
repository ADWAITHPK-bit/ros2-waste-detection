# SAR — Search and Rescue Autonomous Robot System

ROS 2 Humble-based autonomous search-and-rescue robotics system developed for simulation and future deployment on a multi-robot platform.

The system is designed around:

- Gazebo simulation
- TurtleBot3 robots
- SLAM / localization
- Nav2 autonomous navigation
- Dynamic obstacle perception
- Camera-based object / marker detection
- Target database
- Multi-robot task allocation
- Waypoint-based navigation

---

## 1. System Architecture

The intended system pipeline is:

```text
                    Sensors
              ┌───────┴────────┐
              │                │
            LiDAR            Camera
              │                │
              ▼                ▼
        SLAM / Mapping     Perception
              │                │
              ▼                ▼
        Localization      Detections
              │                │
              └───────┬────────┘
                      ▼
               Navigation Logic
                      │
                      ▼
                Global Planner
                      │
                      ▼
                Local Controller
                      │
                      ▼
                 Robot Motion
