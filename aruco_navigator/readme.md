# Aruco Navigator

This is a ROS2 package to make the turtlebot make 90 degree turns and follow instructions encoded on arUco tags. The simulation is rendered on Gazebo. 

## Dependencies

ROS2 Humble Hawksbill
Turtlebot3*
rclpy
geometry_msgs
sensor_msgs
cv2
cv_bridge
numpy
python3
Gazebo v11 (classic)

## Steps to Build
colcon build --packages-select aruco_navigator --symlink-install
source install/setup.bash

## Steps to run

Terminal 1 : launch gazebo world with the arUco tags

export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:(to the arUco tags directory)
ros2 launch turtlebot3_gazebo empty_world.launch.py

Terminal 2: run the node

ros2 run aruco_navigator aruco_nav

>[!Note]
> You can add the line "export TURTLEBOT3_MODEL=waffle" by :: echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc to make the change permanent
> Similarly you can echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:\<path to the directory with arUco tags>" >> ~/.bashrc

