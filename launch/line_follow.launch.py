import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('line_follow'), 'config', 'params.yaml'
    )

    return LaunchDescription(
        [
            LifecycleNode(
                package='line_follow',
                executable='core_loop',
                name='core_loop',
                namespace='line_follow',
                output='screen',
                parameters=[config],
            ),
            LifecycleNode(
                package='line_follow',
                executable='navigation',
                name='navigation',
                namespace='line_follow',
                output='screen',
                parameters=[config],
            ),
            LifecycleNode(
                package='line_follow',
                executable='pid_loop',
                name='pid_loop',
                namespace='line_follow',
                output='screen',
                parameters=[config],
            ),
        ]
    )
