from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace='aRN_Turtle', package='turtlesim',
            executable='turtlesim_node', output='screen'),
    ])

