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

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "pick_place_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Use a thread to allow MoveIt to process callbacks
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // 1. Setup Planning Interfaces
  moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // 2. Define the Cube (Collision Object)
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();
  collision_object.id = "cube";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {0.05, 0.05, 0.05}; // 5cm cube

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.4; // 40cm in front of robot
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.025; // Sitting on the ground plane

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // 3. Add the cube to the world
  planning_scene_interface.applyCollisionObject(collision_object);

  // 4. Plan to a position above the cube (The "Lift" or Pre-Grasp)
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 0.0; // Pointing down
  target_pose.orientation.x = 1.0;
  target_pose.position.x = 0.4;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.3; // 30cm high

  move_group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  if (bool(move_group.plan(my_plan))) {
    move_group.execute(my_plan);
  }

  rclcpp::shutdown();
  return 0;
}