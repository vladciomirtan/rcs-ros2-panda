/*
 * Filename: stacking_cubes_node.cpp
 * Project: Robotic Systems Control - Panda Pick and Place
 * Description: A ROS2 node using MoveIt2 to perform a series of pick and place tasks to create a stack of cubes
 * Contact: ciomixd@gmail.com
 * Version: 0.5.5
 * License: Apache License-2.0
 * Date: January 2026
 */

// Required header files:
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define TOP_RETREAT 0.5

// Pick and place function signature:
void pickAndPlace(
    const std::string& object_id,
    double pick_x, double pick_y, double pick_z,
    double place_x, double place_y, double place_z,
    moveit::planning_interface::MoveGroupInterface& arm,
    moveit::planning_interface::MoveGroupInterface& hand);

// Main function
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>("stacking_node", 
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    moveit::planning_interface::MoveGroupInterface arm_group(node, "panda_arm");
    moveit::planning_interface::MoveGroupInterface hand_group(node, "hand");
    moveit::planning_interface::PlanningSceneInterface planning_scene;

    arm_group.setPlanningTime(30.0);
    arm_group.setNumPlanningAttempts(10); // Try multiple paths
    arm_group.setGoalPositionTolerance(0.002); 
    arm_group.setGoalOrientationTolerance(0.01); 
    arm_group.setMaxVelocityScalingFactor(0.2);

    // --- 0. Reset sequence ---
    // Detach any object and delete them, then revert to "Ready" position
    // RCLCPP_INFO(node->get_logger(), "Initializing Cleanup...");
    arm_group.detachObject(); 
    hand_group.setNamedTarget("open");
    hand_group.move();

    std::vector<std::string> object_ids = {"cube_0", "cube_1", "cube_2"};
    planning_scene.removeCollisionObjects(object_ids);

    arm_group.setNamedTarget("ready");
    arm_group.move();

    // --- 1. Add cubes to the envirnoment ---
    const int num_cubes = 4; // Set to 3 for the stacking task
    for (int i = 0; i < num_cubes; i++)
    {
        moveit_msgs::msg::CollisionObject cube;
        cube.header.frame_id = arm_group.getPlanningFrame();
        cube.id = "cube_" + std::to_string(i);
        
        shape_msgs::msg::SolidPrimitive prim;
        prim.type = prim.BOX;
        prim.dimensions = {0.05, 0.05, 0.05};

        geometry_msgs::msg::Pose box_p;
        box_p.orientation.w = 1.0;
        box_p.position.x = 0.4; 
        box_p.position.y = i * 0.15; 
        box_p.position.z = 0.025; 
        
        cube.primitives.push_back(prim);
        cube.primitive_poses.push_back(box_p);
        cube.operation = cube.ADD;

        planning_scene.applyCollisionObject(cube);
        
        // 0.5s is usually enough once you use applyCollisionObject
        rclcpp::sleep_for(std::chrono::milliseconds(500)); 
    }
    // --- THE TASK ---
    // Syntax: pickAndPlace(ID, pick_x, pick_y, pick_z, place_x, place_y, place_z, arm, hand)
    
    for(int i = 0; i < num_cubes; i++)
    {
        std::string cube_name = "cube_" + std::to_string(i);
        double stack_z = 0.025 + (i * 0.051);
        pickAndPlace(cube_name, 0.4, i * 0.15, 0.025, 0.6, 0.0, stack_z, arm_group, hand_group);
    }
    
    rclcpp::shutdown();
    return 0;
}

void pickAndPlace(
    const std::string& object_id,
    double pick_x, double pick_y, double pick_z,
    double place_x, double place_y, double place_z,
    moveit::planning_interface::MoveGroupInterface& arm,
    moveit::planning_interface::MoveGroupInterface& hand) 
{
    // Define the logger using the arm's node
    // auto logger = arm.get_node()->get_logger();
    // RCLCPP_INFO(logger, "Processing object: %s", object_id.c_str());

    geometry_msgs::msg::Pose target;
    
    // Orientation: Fingers pointing straight down, aligned with the axes
    tf2::Quaternion q;
    q.setRPY(3.14159, 0.0, 0.785398); 
    target.orientation = tf2::toMsg(q);

    // --- PICK PHASE ---

    // 1. Hover above the pick site
    target.position.x = pick_x; 
    target.position.y = pick_y; 
    target.position.z = pick_z + 0.15; // Hover 15cm above
    arm.setPoseTarget(target);
    arm.move();

    // 2. Open Hand
    hand.setNamedTarget("open");
    hand.move();

    // 3. Lower to grasp position
    // Note: 0.13 is a safe offset for the Panda hand to center the cube in the fingers
    target.position.z = pick_z + 0.13; 
    arm.setPoseTarget(target);
    arm.move();

    // 4. Close Hand
    hand.setNamedTarget("close");
    hand.move();

    // 5. Attach object to the robot model
    // This allows MoveIt to treat the cube as part of the robot for collision checking
    std::vector<std::string> touch_links = {"panda_leftfinger", "panda_rightfinger", "panda_hand", "panda_link8"};
    arm.attachObject(object_id, "panda_hand", touch_links);

    // 6. Lift up
    target.position.z = 0.4; 
    arm.setPoseTarget(target);
    arm.move();

    // --- PLACE PHASE ---

    // 7. Move to a position above the stack site
    target.position.x = place_x; 
    target.position.y = place_y; 
    target.position.z = 0.4;
    arm.setPoseTarget(target);
    arm.move();

    // 8. Lower onto the stack
    // We use a slightly higher offset (+0.14) to avoid "slamming" the cubes together
    target.position.z = place_z + 0.135; 
    arm.setMaxVelocityScalingFactor(0.05); // Slow down for precision
    arm.setPoseTarget(target);
    arm.move();

    // 9. Release Sequence
    // Small pause to let the physics engine settle
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    hand.setNamedTarget("open");
    hand.move();

    arm.detachObject(object_id);
    
    // 10. Retreat upward and slightly backward to avoid hitting the stack
    arm.setMaxVelocityScalingFactor(0.2);
    target.position.z = 0.5;
    target.position.x -= 0.05; // Back away 5cm
    arm.setPoseTarget(target);
    arm.move();

    // RCLCPP_INFO(logger, "Successfully placed %s", object_id.c_str());
}