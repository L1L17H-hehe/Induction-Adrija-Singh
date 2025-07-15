# Turtle Controller

This is a ROS2 package to move the turtlebot3 in a circle. Simulation is rendered on Gazebo.

## Dependencies
- ROS2 Humble Hawksbill
- Turtlebot3*
- rclpy
- geometry_msgs
- python3
- Gazebo v11 (classic)

## Steps to Build

export TURTLEBOT3_MODEL=burger
colcon build --packages-select turtle_controller --symlink-install
source install/setup.bash

## Steps to Run

#### Terminal 1: launch gazebo world
- export TURTLEBOT3_MODEL=burger
- ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py <br>

#### Terminal 2: run the velocity node
- ros2 run turtle_controller velocity_node.py

#### Terminal 3: show what is being echoed to cmd_vel topic
- ros2 topic echo /cmd_vel

>[!Note]
> You can ```echo "export TURTLEBOT3_MODEL=burger">> ~/.bashrc``` to make the change permanent
