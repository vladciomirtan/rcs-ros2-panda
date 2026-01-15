# TODO: Add file description boilerplate

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Load the robot configuration (URDF, SRDF, etc.)
    # This matches the 'panda' robot from your installed config package
    moveit_config = MoveItConfigsBuilder("panda", package_name="moveit_resources_panda_moveit_config").to_moveit_configs()

    # 2. Define the Node
    # This passes all the necessary robot parameters to your C++ executable
    run_move_group_node = Node(
        package="panda_pick_place",
        executable="pick_place_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription([
        run_move_group_node,
    ])