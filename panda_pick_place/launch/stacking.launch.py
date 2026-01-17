import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Load the robot configuration
    moveit_config = MoveItConfigsBuilder("panda", package_name="moveit_resources_panda_moveit_config").to_moveit_configs()

    # 2. Define the Stacking Node
    run_stacking_node = Node(
        package="panda_pick_place",
        executable="stacking_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription([
        run_stacking_node,
    ])