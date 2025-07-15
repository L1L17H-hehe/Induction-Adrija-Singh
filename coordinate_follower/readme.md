# Coordinate Follower

This is a ROS2 package for commanding the turtlebot to chase target coordinates provided to in a txt file. The simulation is rendered on gazebo.

## Dependencies
- ROS2 Humble Hawksbill
- Turtlebot3*
- rclpy
- geometry_msgs
- nav_msgs
- python3
- Gazebo v11 (classic) used
- math

## Steps to Build

export TURTLEBOT3_MODEL=burger
colcon build --packages-select coordinate_follower --symlink-install
source install/setup.bash

## Steps to run

#### Terminal 1: launch turtlebot3
- export TURTLEBOT3_MODEL=burger
- ros2 launch turtlebot3_gazebo empty_world.launch.py

#### Terminal 2: run the server script
- ros2 run coordinate_follower server

#### Terminal 3: run the client script
- ros2 run coordinate_follower client

>[!Note]
>You can modify the coordinates.txt file
>You can echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc to make the change permanent
