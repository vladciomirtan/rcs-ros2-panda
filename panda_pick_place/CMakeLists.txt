cmake_minimum_required(VERSION 3.8)
project(panda_pick_place)

# Required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# --- 1. Simple Pick and Place Node ---
add_executable(pick_place_node src/pick_place_node.cpp)
ament_target_dependencies(pick_place_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# --- 2. Stacking Cubes Node ---
add_executable(stacking_node src/stacking_cubes_node.cpp)
ament_target_dependencies(stacking_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# --- Installation ---

# Install both executables
install(TARGETS 
    pick_place_node 
    stacking_node
  DESTINATION lib/${PROJECT_NAME})

# Install the entire launch directory (contains both .launch.py files)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()