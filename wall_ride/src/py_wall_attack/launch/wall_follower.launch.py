import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_pkg = get_package_share_directory("ros_gz_bringup")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_pkg, "launch", "diff_drive.launch.py")
            )
        ),
        Node(
            package="py_wall_attack",
            executable="wall_follower",
            name="wall_follower",
            output="screen",
            parameters=[{
                "d_des":        0.60,
                "band":         0.05,
                "front_thresh": 0.45,
                "v_forward":    0.25,
                "w_turn":       0.80,
                "avg_window":   3,
            }],
        ),
    ])
