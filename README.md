# ROS 2 Iron Docker and Gazebo Demo

This repository contains a Docker-based development environment for ROS 2 Iron, including a simple differential drive robot simulation in Gazebo.

## Prerequisites

- Windows 10/11
- [Docker Desktop](https://www.docker.com/products/docker-desktop/)
- [VcXsrv Windows X Server](https://sourceforge.net/projects/vcxsrv/)

## Directory Structure
```
ros2_iron_ws/
├── docker-compose.yml
└── src/
    └── my_robot/
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/
        │   └── spawn.launch.py
        └── urdf/
            └── robot.urdf.xacro
```

## Setup Instructions

1. **Clone the repository:**
   ```bash
   git clone https://github.com/vedicpatel07/ros_docker_gazebo_demo.git
   cd ros_docker_gazebo_demo
   ```

2. **Start XServer (VcXsrv)**
   - Launch XLaunch
   - Choose "Multiple windows"
   - Display number: 0
   - Start no client
   - Extra settings:
     - Check "Disable access control"
     - Check "Native opengl"
     - Check "Indirect OpenGL"

3. **Start the Docker container:**
   ```bash
   docker compose up -d
   docker exec -it ros2_iron_dev bash
   ```

4. **Inside the container:**
   ```bash
   source /opt/ros/iron/setup.bash
   cd /ros2_ws
   colcon build
   source install/setup.bash
   ```

5. **Launch the robot simulation:**
   ```bash
   ros2 launch my_robot spawn.launch.py
   ```

## Testing Robot Movement

1. **In a new terminal, enter the container:**
   ```bash
   docker exec -it ros2_iron_dev bash
   ```

2. **Source ROS 2:**
   ```bash
   source /opt/ros/iron/setup.bash
   source /ros2_ws/install/setup.bash
   ```

3. **Send movement commands:**
   ```bash
   # Move forward and rotate
   ros2 topic pub --rate 1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"
   ```

4. **Stop the robot:**
   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
   ```

## Shutting Down

1. **Stop the simulation:** Press Ctrl+C in the Gazebo terminal
2. **Exit the container:** Type `exit`
3. **Stop the container:**
   ```bash
   docker compose down
   ```

## Troubleshooting

1. **If Gazebo doesn't display:**
   - Make sure XServer is running with the correct settings
   - Try restarting the Docker container
   - Check if the DISPLAY environment variable is set correctly

2. **If robot movement doesn't work:**
   - Check if topics are available: `ros2 topic list | grep cmd_vel`
   - Verify joint states: `ros2 topic echo /joint_states`
   - Make sure all ROS 2 dependencies are installed

## Notes
- The robot model is a simple differential drive robot with a cylindrical body and two wheels
- The simulation uses Gazebo Classic
- The robot can be controlled using the `/cmd_vel` topic

## Acknowledgments
- ROS 2 Iron Documentation
- Gazebo Classic Documentation
- OSRF Docker Images
