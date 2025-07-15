# Obstacle Avoider

This is a ROS2 package for commanding the turtlebot3 to detect and avoid obstacles using LaserScan. The simulation is rendered on Gazebo.

## Dependencies
- ROS2 Humble Hawksbill
- TurtleBot3*
- rclpy
- geometry_msgs
- sensor_msgs
- python3
- Gazebo v11 (classic)

## Steps to Build

export TURTLEBOT3_MODEL=burger
colcon build --packages-select obstacle_avoider --symlink-install
source install/setup.bash

## Steps to Run

#### Terminal 1: launch gazebo world
- export TURTLEBOT3_MODEL=burger
- ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 

#### Terminal 2: run the package
- ros2 run obstacle_avoider obstacle_swerver

#### Terminal 3: display what is being echoed to the cmd_vel topic
- ros2 topic echo /cmd_vel

>[!Note]
> You can ```echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc ``` to make the change permanent

