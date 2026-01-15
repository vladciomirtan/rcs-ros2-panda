cmake_minimum_required(VERSION 3.8)
project(panda_pick_place)

# Required for MoveGroupInterface and ROS2 logic
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

# Define the executable
add_executable(pick_place_node src/pick_place_node.cpp)

# Link MoveIt2 and ROS2 dependencies
target_link_libraries(pick_place_node PRIVATE
  moveit_ros_planning_interface::moveit_ros_planning_interface
)

ament_target_dependencies(pick_place_node
  rclcpp
  geometry_msgs
)

# Standard ROS2 installation paths
install(TARGETS pick_place_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()