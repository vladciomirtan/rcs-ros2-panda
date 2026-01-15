cmake_minimum_required(VERSION 3.8)
project(panda_pick_place)

# Required for MoveGroupInterface and ROS2 logic
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

# Define the executable
add_executable(pick_place_node src/pick_place_node.cpp)

# Use ament_target_dependencies for all ROS2/MoveIt2 libs to avoid keyword conflicts
ament_target_dependencies(pick_place_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

# Standard ROS2 installation paths
install(TARGETS pick_place_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()