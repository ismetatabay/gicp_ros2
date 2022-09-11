from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from glob import glob

import os

def generate_launch_description():
    rviz_cfg_path = os.path.join(get_package_share_directory('gicp_ros2'),
                                 'config', 'gicp.rviz')
    gicp = Node(
        package="gicp_ros2",
        executable="gicp",
        name="gicp_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"pcd1_filename": "src/gicp_ros2/data/capture0001.pcd"},
            {"pcd2_filename": "src/gicp_ros2/data/capture0002.pcd"}
        ]
    )
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg_path],
    )
    return LaunchDescription([
        gicp,
        rviz2
    ])
