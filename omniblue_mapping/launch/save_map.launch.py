import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # 声明参数
    map_name = LaunchConfiguration('map_name', default='warehouse_map')
    map_save_path = LaunchConfiguration(
        'map_save_path',
        default=os.path.join(
            get_package_share_directory('omniblue_mapping'),
            'maps'
        )
    )
    
    # 保存地图节点
    map_saver = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', 
             [map_save_path, '/', map_name]],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value='warehouse_map',
            description='Name of the map to save'
        ),
        DeclareLaunchArgument(
            'map_save_path',
            default_value=os.path.join(
                get_package_share_directory('omniblue_mapping'),
                'maps'
            ),
            description='Path to save the map'
        ),
        map_saver
    ])
