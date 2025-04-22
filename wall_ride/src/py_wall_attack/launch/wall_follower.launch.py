import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # locate the bringup package so we can include its Gazebo+RViz launcher
    bringup_pkg = get_package_share_directory('ros_gz_bringup')

    return LaunchDescription([
        # 1. Start the Gazebo sim + bridge + RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_pkg, 'launch', 'diff_drive.launch.py')
            )
        ),

        # 2. Start your wall_follower node
        Node(
            package='py_wall_attack',        # your package name
            executable='wall_follower',      # console_script name you set in setup.py
            name='wall_follower',
            output='screen',
            parameters=[
                {'d_des': 0.6},
                {'band': 0.05},
                {'front_thresh': 0.4},
                {'v_forward': 0.25},
                {'w_turn': 0.7},
            ]
        ),
    ])
