cmake_minimum_required(VERSION 3.8)
project(turtle_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif() 

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

#install python scripts
install(PROGRAMS
    scripts/velocity_node.py
    DESTINATION lib/turtle_controller
    RENAME velocity_node
)

ament_package()

