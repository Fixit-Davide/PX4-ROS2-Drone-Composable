cmake_minimum_required(VERSION 3.5)
project(drone_pose)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic 
  -O3 -mtune=native -march=native
  #-g -ggdb
  )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(
  include
)

add_library(drone_pose SHARED src/DronePose.cpp)
ament_target_dependencies(drone_pose rclcpp rcutils rclcpp_components nav_msgs visualization_msgs geometry_msgs message_filters px4_msgs tf2_ros tf2)

rclcpp_components_register_node(
  drone_pose
  PLUGIN "px4_autonav::DronePose"
  EXECUTABLE DronePoseExecutable)

install(TARGETS drone_pose
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files.
install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()