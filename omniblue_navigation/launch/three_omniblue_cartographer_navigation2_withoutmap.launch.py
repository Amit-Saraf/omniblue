import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取共享目录路径
    omniblue_cartographer_prefix = get_package_share_directory('omniblue_navigation')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', 
                              default=os.path.join(omniblue_cartographer_prefix, 'config'))
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # RViz配置
    rviz_config_dir = os.path.join(omniblue_cartographer_prefix,
                                'rviz', 'multi_omniblue_cartographer_navigation2.rviz')
    
    # 为三个机器人定义配置
    robot_configurations = [
        {
            'namespace': 'omniblue1',
            'config_basename': 'omniblue1_lds_2d_withoutmap.lua',
            'params_file': 'omniblue1_cartographer_move_base.yaml',
            'offset_x': 4.5,  # 全局地图中的位置偏移
            'offset_y': -4.0,
            'offset_roll': 0.0,  # 添加roll角
            'offset_pitch': 0.0,  # 添加pitch角
            'offset_yaw': 0.0
        },
        {
            'namespace': 'omniblue2',
            'config_basename': 'omniblue2_lds_2d_withoutmap.lua',
            'params_file': 'omniblue2_cartographer_move_base.yaml',
            'offset_x': 5.7,  # 全局地图中的位置偏移
            'offset_y': -3.2,
            'offset_roll': 0.0,
            'offset_pitch': 0.0,
            'offset_yaw': 0.0
        },
        {
            'namespace': 'omniblue3',
            'config_basename': 'omniblue3_lds_2d_withoutmap.lua',
            'params_file': 'omniblue3_cartographer_move_base.yaml',
            'offset_x': 5.6,  # 全局地图中的位置偏移
            'offset_y': -5.0,
            'offset_roll': 1.57,
            'offset_pitch': 0.0,
            'offset_yaw': 0.0  # 约90度，与Gazebo中设置相匹配
        }
    ]
    
    # 定义启动参数
    launch_args = [
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config files directory'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        )
    ]
    
    # 初始化节点列表
    robot_groups = []
    static_tf_pubs = []
    
    # 创建全局地图到各个机器人地图的静态变换
    for robot in robot_configurations:
        namespace = robot['namespace']
        offset_x = str(robot['offset_x'])
        offset_y = str(robot['offset_y'])
        offset_roll = str(robot['offset_roll'])
        offset_pitch = str(robot['offset_pitch'])
        offset_yaw = str(robot['offset_yaw'])
        
        # 创建从全局地图到机器人地图的静态TF变换
        static_tf_pubs.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'global_to_{namespace}_map',
            arguments=[
                offset_x, offset_y, '0', 
                offset_roll, offset_pitch, offset_yaw, 
                'global_map', f'{namespace}/map'
            ]
        ))
    
    # 为每个机器人创建一个命名空间组
    for robot in robot_configurations:
        namespace = robot['namespace']
        config_basename = robot['config_basename']
        params_file = robot['params_file']
        
        # 完整参数文件路径
        full_params_path = os.path.join(
            omniblue_cartographer_prefix, 'config', params_file)
        
        # 创建一个组，确保所有节点都在给定的命名空间下
        group_actions = GroupAction([
            # 设置命名空间
            PushRosNamespace(namespace),
            
            # Cartographer SLAM节点
            Node(
                package='cartographer_ros',
                executable='cartographer_node',
                name='cartographer_node',  # 不需要额外的命名空间前缀
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                remappings=[
                    ('scan', f'scan'),  # 在命名空间内，这些话题会自动加前缀
                    ('odom', f'odom')
                ],
                arguments=[
                    '-configuration_directory', cartographer_config_dir,
                    '-configuration_basename', config_basename
                ]
            ),
            
            # 占用栅格发布器
            Node(
                package='cartographer_ros',
                executable='cartographer_occupancy_grid_node',
                name='occupancy_grid_node',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
            ),
            
            # Nav2 导航节点 - 使用独立的nav2启动文件
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_launch.py']),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': full_params_path,
                    'autostart': 'true',
                    # 不再传递namespace参数，因为已经在组内设置了
                }.items(),
            ),
        ])
        
        robot_groups.append(group_actions)
    
    # 添加一个虚拟的机器人状态发布器，用于发布global_map坐标系
    global_map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='global_map_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'global_map']
    )
    
    # 添加RViz - 独立于任何特定机器人的命名空间
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 组装最终的启动描述
    return LaunchDescription([
        *launch_args,
        global_map_node,  # 首先创建全局地图坐标系
        *static_tf_pubs,  # 然后创建到各个机器人地图的链接
        *robot_groups,    # 接着启动所有机器人的功能节点
        rviz_node         # 最后启动RViz
    ])
