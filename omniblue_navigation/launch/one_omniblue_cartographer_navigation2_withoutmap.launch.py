import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    omniblue_cartographer_prefix = get_package_share_directory('omniblue_navigation')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  omniblue_cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='one_omniblue_lds_2d_withoutmap.lua')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    param_file_name = 'one_omniblue_cartographer_move_base.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('omniblue_navigation'),
            'config',
            param_file_name))
    rviz_config_dir = os.path.join(get_package_share_directory('omniblue_navigation'),
                                   'rviz', 'one_omniblue_cartographer_navigation2.rviz')
    
    # 为 Nav2 添加必要的重映射
    remappings = [
        SetRemap('/cmd_vel', '/omniblue/cmd_vel'),
        SetRemap('/odom', '/omniblue/odom'),
        SetRemap('/scan', '/omniblue/scan')
    ]
    
    return LaunchDescription([
        # 首先添加重映射动作
        *remappings,
        
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        # Cartographer SLAM 节点 (不加载现有地图)
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[("scan", "/omniblue/scan"),
                        ("odom", "/omniblue/odom")],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]),
                       
        # 占用栅格发布器
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),
        
        # 启动Nav2，传递命名空间参数
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
                'autostart': 'true'
            }.items(),
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
