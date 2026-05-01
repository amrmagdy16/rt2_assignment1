# Robot Navigation - Research Track II Assignment 1

This repository contains a ROS 2 based navigation stack implemented using **Actions**, **Components**, and **TF2**[cite: 1, 2, 3]. The system allows a user to input a target pose ($x$, $y$, $\theta$), and a robot moves toward that goal in a simulated environment while providing real-time feedback and the ability to cancel the task[cite: 1, 3].

## Architecture Overview
The project follows a **Hybrid Modular Architecture** to ensure non-blocking user interaction and high-performance robot control[cite: 2]:

*   **`nav_interfaces`**: A dedicated package for custom messages (`TargetGoal.msg`) and actions (`RobotNav.action`)[cite: 2].
*   **`nav_ui` (Python Node)**: A standalone node that captures user input from the keyboard. It handles inputs in a separate thread to prevent blocking the ROS 2 executor[cite: 2].
*   **`nav_assignment` (C++ Components)**:
    *   **Action Client**: Subscribes to the UI topic and manages the lifecycle of navigation goals[cite: 2, 3].
    *   **Action Server**: The core controller. it uses `tf2` to track the robot's pose relative to the `odom` frame and publishes velocity commands to `/cmd_vel` using a proportional control loop[cite: 1, 3].



## Prerequisites
*   **ROS 2 Jazzy** (or compatible version)[cite: 1].
*   **Simulator**: The `bme_gazebo_sensors` package (branch `rt2`).

## Installation
1.  **Clone the repository and simulator**:
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/amrmagdy16/rt2_assignment1.git
    git clone -b rt2 https://github.com/CarmineD8/bme_gazebo_sensors.git
    ```
2.  **Build the workspace**:
    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```

## How to Run
You will need three terminals, each sourced with `source install/setup.bash`.

1.  **Terminal 1 (Simulation)**:
    ```bash
    ros2 launch bme_gazebo_sensors spawn_robot_ex.launch.py
    ```
2.  **Terminal 2 (Navigation Stack)**:
    ```bash
    ros2 launch nav_assignment nav_system.launch.py
    
```
3.  **Terminal 3 (User Interface)**:
    ```bash
    ros2 run nav_ui ui_node
    ```

## Action Interface Details
The `RobotNav.action` is defined as follows[cite: 3]:
*   **Goal**: `target_x`, `target_y`, `target_theta`
*   **Result**: `success` (boolean)
*   **Feedback**: `distance_remaining` (float64)

