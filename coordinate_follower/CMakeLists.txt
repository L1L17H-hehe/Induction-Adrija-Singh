cmake_minimum_required(VERSION 3.8)
project(coordinate_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) 
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

#generate action interfaces
rosidl_generate_interfaces(coordinate_follower
  "action/FollowCoordinate.action"
  DEPENDENCIES geometry_msgs nav_msgs action_msgs
)

# install python scripts as executables
install(PROGRAMS
  scripts/server.py
  DESTINATION lib/coordinate_follower
  RENAME server
)

install(PROGRAMS
  scripts/client.py
  DESTINATION lib/coordinate_follower
  RENAME client
)

# install action files 
install(DIRECTORY action
  DESTINATION share/coordinate_follower
)

install(FILES
  coordinates.txt
  DESTINATION share/coordinate_follower
  )


ament_package()
