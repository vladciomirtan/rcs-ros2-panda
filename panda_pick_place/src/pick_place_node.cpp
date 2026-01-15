/*
 * Filename: pick_place_node.cpp
 * Project: Robotic Systems Control - Panda Pick and Place
 * Description: A ROS2 node using MoveIt2 to perform a simple pick and lift 
 * task with the Franka Emika Panda robot.
 * Author: [Vlad Ciomirtan]
 * Contact: ciomixd@gmail.com
 * Version: 0.0.5
 * License: Apache License-2.0
 * Date: January 2026
 */

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

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
  // 1. Release the object in software
  arm_group.detachObject("cube");

  // 2. Open the fingers
  hand_group.setNamedTarget("open");
  hand_group.move();

  // 3. Remove from the world if you want to 'delete' it
  std::vector<std::string> object_ids;
  object_ids.push_back("cube");
  planning_scene_interface.removeCollisionObjects(object_ids);

  // 4. Now reset the arm
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
  box_p.position.x = 0.4; box_p.position.y = 0.0; box_p.position.z = 0.025;
  cube.primitives.push_back(prim);
  cube.primitive_poses.push_back(box_p);
  cube.operation = cube.ADD;

  // Apply and wait a moment for the scene to update
  planning_scene_interface.applyCollisionObject(cube);
  rclcpp::sleep_for(std::chrono::seconds(2)); 

  // --- STEP 2: Execute Task ---
  // 1. Open Hand
  hand_group.setNamedTarget("open");
  hand_group.move();

  // 2. Pre-Grasp Pose
  geometry_msgs::msg::Pose target;
  target.orientation.x = 1.0; // Gripper pointing down
  target.position.x = 0.4; target.position.y = 0.0; target.position.z = 0.25;
  
  arm_group.setPoseTarget(target);
  arm_group.move();

  // 3. Grasp (Close)
  hand_group.setNamedTarget("close");
  hand_group.move();

  // 4. Attach & Lift
  std::vector<std::string> touch_links;
  touch_links.push_back("panda_leftfinger");
  touch_links.push_back("panda_rightfinger");

  arm_group.attachObject("cube", "panda_hand", touch_links);
  target.position.z = 0.5;
  arm_group.setPoseTarget(target);
  arm_group.move();

  rclcpp::shutdown();
  return 0;
}