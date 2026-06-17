import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_name = LaunchConfiguration('map_name', default='warehouse_map')
    
    # 获取包目录
    pkg_dir = get_package_share_directory('omniblue_mapping')
    map_path = os.path.join(pkg_dir, 'maps', map_name)
    
    # 地图服务器节点
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_path + '.yaml'}
        ]
    )
    
    # AMCL定位节点
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'odom_frame_id': 'odom'},
            {'base_frame_id': 'base_footprint'},
            {'global_frame_id': 'map'}
        ],
        remappings=[
            ('/scan', '/omniblue/omniblue/laser/scan')
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'map_name',
            default_value='warehouse_map',
            description='Name of the map to use for localization'
        ),
        map_server_node,
        amcl_node
    ])
