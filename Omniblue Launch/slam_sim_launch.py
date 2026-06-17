from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    params_file = os.path.expanduser('~/Desktop/Omniblue Launch/slam_toolbox_sim.yaml')
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file, {'use_sim_time': True}],
            remappings=[('/scan', '/omniblue/scan')],
        )
    ])
