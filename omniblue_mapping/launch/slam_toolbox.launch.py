import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 获取包目录
    pkg_dir = get_package_share_directory('omniblue_mapping')
    
    # 获取SLAM Toolbox配置文件路径
    slam_params_file = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'config',
        'mapper_params_online_async.yaml'
    )
    
    # 配置SLAM Toolbox节点
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {
                'use_sim_time': use_sim_time,
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'map_frame': 'map'
            }
        ],
        remappings=[
            ('/scan', '/omniblue/scan')  # 修改为您的激光扫描话题
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        slam_toolbox_node
    ])
