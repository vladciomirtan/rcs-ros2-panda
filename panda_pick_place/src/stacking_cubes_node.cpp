/*
 * Filename: stacking_cubes_node.cpp
 * Project: Robotic Systems Control - Panda Pick and Place
 * Description: A ROS2 node using MoveIt2 to perform a series of pick and place tasks to create a stack of cubes
 * Contact: ciomixd@gmail.com
 * Version: 0.5.5
 * License: Apache License-2.0
 * Date: January 2026
 */

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define TOP_RETREAT 0.5

void pickAndPlace(
    const std::string& object_id,
    double pick_y,
    double place_x, double place_y, double place_z,
    moveit::planning_interface::MoveGroupInterface& arm,
    moveit::planning_interface::MoveGroupInterface& hand);


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

    arm_group.setPlanningTime(20.0);
    arm_group.setGoalPositionTolerance(0.001); // 1mm
    arm_group.setGoalOrientationTolerance(0.01); // ~0.5 degrees
    arm_group.setMaxVelocityScalingFactor(0.2);

    // --- RESET SEQUENCE ---
    RCLCPP_INFO(node->get_logger(), "Initializing Cleanup...");

    // 1. Detach any objects currently held by the robot
    arm_group.detachObject(); 

    // 2. Open the gripper to release any physical "pinch"
    hand_group.setNamedTarget("open");
    hand_group.move();

    // 3. Remove all collision objects (cubes) from the world
    // We use a list of all possible IDs we might have used
    std::vector<std::string> object_ids = {"cube_0", "cube_1", "cube_2"};
    planning_scene.removeCollisionObjects(object_ids);

    // 4. Move the arm back to the 'ready' pose
    arm_group.setNamedTarget("ready");
    arm_group.move();

    RCLCPP_INFO(node->get_logger(), "Cleanup Complete. Starting Task...");

    // --- SPAWN MULTIPLE CUBES ---
    for(int i=0; i<2; i++) {
        moveit_msgs::msg::CollisionObject c;
        c.header.frame_id = arm_group.getPlanningFrame();
        c.id = "cube_" + std::to_string(i);
        shape_msgs::msg::SolidPrimitive p;
        p.type = p.BOX; p.dimensions = {0.05, 0.05, 0.05};
        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = 0.4; 
        pose.position.y = (i == 0) ? 0.0 : 0.15; 
        pose.position.z = 0.025;
        c.primitives.push_back(p);
        c.primitive_poses.push_back(pose);
        c.operation = c.ADD;
        
        // Use applyCollisionObject for synchronous addition
        planning_scene.applyCollisionObject(c);
    }
    RCLCPP_INFO(node->get_logger(), "Cubes spawned and verified.");
    rclcpp::sleep_for(std::chrono::seconds(1));

    // --- THE TASK ---
    // Place cube_0 at (0.5, 0.2) on the floor (z=0.025)
    pickAndPlace("cube_0", 0.0, 0.5, 0.2, 0.025, arm_group, hand_group);

    // Place cube_1 at (0.5, 0.2) on top of cube_0 (z=0.075)
    pickAndPlace("cube_1", 0.15, 0.5, 0.2, 0.075, arm_group, hand_group);

    rclcpp::shutdown();
    return 0;
}

void pickAndPlace(
    const std::string& object_id,
    double pick_y,
    double place_x, double place_y, double place_z,
    moveit::planning_interface::MoveGroupInterface& arm,
    moveit::planning_interface::MoveGroupInterface& hand) 
{
    geometry_msgs::msg::Pose target;
    tf2::Quaternion q;
    q.setRPY(3.14159, 0.0, 0.785398); // Fingers down, aligned yaw
    target.orientation = tf2::toMsg(q);

    // --- PICK ---
    // 1. Hover over cube
    target.position.x = 0.4; target.position.y = pick_y; target.position.z = 0.25;
    arm.setPoseTarget(target);
    arm.move();

    hand.setNamedTarget("open");
    hand.move();

    // 2. Descend to grab
    double fine_tuning_z = 0.008975;
    target.position.z = 0.17 - fine_tuning_z;  
    arm.setPoseTarget(target);
    arm.move();

    hand.setNamedTarget("close");
    hand.move();

    // 3. Attach
    std::vector<std::string> touch_links = {"panda_leftfinger", "panda_rightfinger", "panda_hand"};
    arm.attachObject(object_id, "panda_hand", touch_links);

    // 4. Lift
    target.position.z = 0.35;
    arm.setPoseTarget(target);
    arm.move();

    // --- PLACE ---
    // 5. Move to Stack Location
    target.position.x = place_x; target.position.y = place_y; target.position.z = 0.35;
    arm.setPoseTarget(target);
    arm.move();

    // 6. Descend onto stack
    target.position.z = place_z + 0.10; // Palm height relative to cube center
    arm.setPoseTarget(target);
    arm.move();

    // 7. Release
    hand.setNamedTarget("open");
    hand.move();
    arm.detachObject(object_id);

    // 8. Retreat upward
    target.position.z = TOP_RETREAT;
    arm.setPoseTarget(target);
    arm.move();
}