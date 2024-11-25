# Turtlesim Exploration Project

A ROS2-based project for exploring Turtlesim, including basic path planning, navigation, and autonomous movement. This simple project randomly spawns turtles as obstacles as a main turtle tries to avoid them to reach its goal. 

## Features
- Autonomous exploration of the Turtlesim environment.
- Basic path planning and obstacle avoidance.
- Real-time visualization of turtle movements.
- Integration with ROS2 (Robot Operating System 2).

## Prerequisites
Before starting, ensure you have the following installed:
- ROS2 (tested with [specific ROS2 distribution, e.g., Humble/Noetic]).
- Python 3.10+ or another compatible version.
- Git and a compatible code editor/IDE.

## Installation
1. Clone this repository:
    ```bash
    git clone https://github.com/DevCielo/ros_turtle_exploration.git
    cd ros_turtle_exploration
    ```

2. Install dependencies:
    ```bash
    sudo apt update
    rosdep install --from-paths src --ignore-src -r -y
    ```

3. Build the project:
    ```bash
    colcon build
    ```

4. Source your workspace:
    ```bash
    source install/setup.bash
    ```

## Usage
1. Start the Turtlesim node:
    ```bash
    ros2 run turtlesim turtlesim_node
    ```

2. Run the exploration script:
    ```bash
    ros2 run turtlesim_exploration_project turtle_spawner_node
    ros2 run turtlesim_exploration_project exploration_node
    ```

3. Observe the turtle exploring the environment autonomously.
