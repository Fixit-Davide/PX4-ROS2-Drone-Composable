cmake_minimum_required(VERSION 3.16)
project(drone_pose)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g3 -ggdb")

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_BUILD_TYPE Debug)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

include_directories(include)

add_library(DronePose_Node SHARED src/DronePose.cpp)
target_include_directories(
  DronePose_Node
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
          "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(
  DronePose_Node
  SYSTEM
  rclcpp
  rclcpp_components
  rcutils
  message_filters
  nav_msgs
  geometry_msgs
  visualization_msgs
  px4_msgs)

add_library(DronePose SHARED src/Component.cpp)
target_include_directories(
  DronePose PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                      "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(
  DronePose
  SYSTEM
  rclcpp
  rclcpp_components
  rcutils
  message_filters
  nav_msgs
  geometry_msgs
  visualization_msgs
  px4_msgs)

rclcpp_components_register_node(
  DronePose PLUGIN "px4_autonav::DronePose" EXECUTABLE
  DronePoseExecutable)

install(
  TARGETS DronePose DronePose_Node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  rclcpp_components
  rcutils
  message_filters
  nav_msgs
  geometry_msgs
  visualization_msgs
  px4_msgs)
ament_package()
