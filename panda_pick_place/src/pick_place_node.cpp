/*
 * Filename: pick_place_node.cpp
 * Project: Robotic Systems Control - Panda Pick and Place
 * Description: A ROS2 node using MoveIt2 to perform a simple pick and lift task with the Franka Emika Panda robot.
 * Author: Vlad Ciomirtan
 * Contact: ciomixd@gmail.com
 * Version: 0.1.5
 * License: Apache License-2.0
 * Date: January 2026
 */

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("pick_place_node", 
  rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  moveit::planning_interface::MoveGroupInterface arm_group(node, "panda_arm");
  moveit::planning_interface::MoveGroupInterface hand_group(node, "hand");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // --- RESET SIMULATION ---
  arm_group.detachObject("cube");
  hand_group.setNamedTarget("open");
  hand_group.move();

  std::vector<std::string> object_ids = {"cube"};
  planning_scene_interface.removeCollisionObjects(object_ids);

  arm_group.setNamedTarget("ready");
  arm_group.move();

  // --- STEP 1: Add the Cube ---
  moveit_msgs::msg::CollisionObject cube;
  cube.header.frame_id = arm_group.getPlanningFrame();
  cube.id = "cube";
  shape_msgs::msg::SolidPrimitive prim;
  prim.type = prim.BOX;
  prim.dimensions = {0.05, 0.05, 0.05};

  geometry_msgs::msg::Pose box_p;
  box_p.orientation.w = 1.0;
  box_p.position.x = 0.4; 
  box_p.position.y = 0.0; 
  box_p.position.z = 0.025; 
  
  cube.primitives.push_back(prim);
  cube.primitive_poses.push_back(box_p);
  cube.operation = cube.ADD;

  planning_scene_interface.applyCollisionObject(cube);
  rclcpp::sleep_for(std::chrono::seconds(2)); 

  // --- STEP 2: Pre-Grasp Approach with TF2 ---
  geometry_msgs::msg::Pose target;
  
  // Define orientation in Radians
  // Roll = 3.14 (180 deg), Pitch = 0, Yaw = 0.785 (45 deg)
  tf2::Quaternion q;
  q.setRPY(3.14159, 0.0, 0.785398); 

  target.orientation = tf2::toMsg(q); // Automatically fills x, y, z, w
  target.position.x = 0.4; 
  target.position.y = 0.0; 
  target.position.z = 0.30;
  
  arm_group.setPoseTarget(target);
  arm_group.move();

  // --- STEP 3: Vertical Descent ---
  // Slow down for precision
  arm_group.setMaxVelocityScalingFactor(0.1);
  
  float fine_z = 0.008975;
  //float fine_z = 0.008900;
  target.position.z = 0.16 - fine_z; 
  arm_group.setPoseTarget(target);
  arm_group.move();
  
  // --- STEP 4: Grasp ---
  hand_group.setNamedTarget("close");
  hand_group.move();

  // --- STEP 5: Attach & Lift ---
  // Important: include all gripper links in touch_links
  std::vector<std::string> touch_links = {"panda_leftfinger", "panda_rightfinger", "panda_hand", "panda_link8"};
  arm_group.attachObject("cube", "panda_hand", touch_links);
  arm_group.setMaxVelocityScalingFactor(0.25);
  arm_group.setMaxAccelerationScalingFactor(0.25);

  target.position.z = 0.5;
  arm_group.setPoseTarget(target);
  arm_group.move();

  rclcpp::shutdown();
  return 0;
}