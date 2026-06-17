import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node, SetRemap
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    omniblue_cartographer_prefix = get_package_share_directory('omniblue_navigation')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  omniblue_cartographer_prefix, 'config'))
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    
    # 共享目录
    share_dir = get_package_share_directory('omniblue_navigation')
    
    # RViz配置
    rviz_config_dir = os.path.join(share_dir, 'rviz', 'two_omniblue_cartographer_navigation2.rviz')
    
    # 全局坐标系节点
    global_map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='global_map_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )
    
    # 机器人1坐标系变换 - 连接全局地图和机器人1地图
    robot1_map_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_omniblue1_map',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'omniblue1_map']
    )
    
    # 机器人2坐标系变换 - 连接全局地图和机器人2地图
    robot2_map_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_omniblue2_map',
        arguments=['2.0', '0', '0', '0', '0', '0', 'map', 'omniblue2_map']
    )
    
    # ==== 机器人1 ====
    # 机器人1 Cartographer SLAM节点
    robot1_cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='omniblue1_cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ("scan", "/omniblue1/scan"),
            ("odom", "/omniblue1/odom"),
            ("map", "/omniblue1/map"),
            ("submap_list", "/omniblue1/submap_list"),
            ("submap_query", "/omniblue1/submap_query"),
            ("trajectory_query", "/omniblue1/trajectory_query")
        ],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', 'omniblue1_lds_2d_withoutmap.lua'])
                   
    # 机器人1 占用栅格发布器
    robot1_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='omniblue1_occupancy_grid_node',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ("map", "/omniblue1/map"),
            ("submap_list", "/omniblue1/submap_list")
        ],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )
    
    # 机器人1的重映射设置
    robot1_remappings = [
        # 基本话题重映射
        ('cmd_vel', '/omniblue1/cmd_vel'),
        ('map', '/omniblue1/map'),
        ('map_server/map', '/omniblue1/map'),
        ('scan', '/omniblue1/scan'),
        ('odom', '/omniblue1/odom'),
        # 导航系统状态话题重映射
        ('plan', '/omniblue1/plan'),
        ('plan_smoothed', '/omniblue1/plan_smoothed'),
        ('received_global_plan', '/omniblue1/received_global_plan'),
        ('local_plan', '/omniblue1/local_plan'),
        ('transformed_global_plan', '/omniblue1/transformed_global_plan'),
        ('behavior_tree_log', '/omniblue1/behavior_tree_log'),
        ('goal_pose', '/omniblue1/goal_pose'),
        ('waypoints', '/omniblue1/waypoints'),
        # Costmap相关话题重映射
        ('global_costmap/costmap', '/omniblue1/global_costmap/costmap'),
        ('global_costmap/costmap_raw', '/omniblue1/global_costmap/costmap_raw'),
        ('global_costmap/costmap_updates', '/omniblue1/global_costmap/costmap_updates'),
        ('global_costmap/published_footprint', '/omniblue1/global_costmap/published_footprint'),
        ('global_costmap/footprint', '/omniblue1/global_costmap/footprint'),
        ('global_costmap/clearing_endpoints', '/omniblue1/global_costmap/clearing_endpoints'),
        ('local_costmap/costmap', '/omniblue1/local_costmap/costmap'),
        ('local_costmap/costmap_raw', '/omniblue1/local_costmap/costmap_raw'),
        ('local_costmap/costmap_updates', '/omniblue1/local_costmap/costmap_updates'),
        ('local_costmap/published_footprint', '/omniblue1/local_costmap/published_footprint'),
        ('local_costmap/footprint', '/omniblue1/local_costmap/footprint'),
        ('local_costmap/clearing_endpoints', '/omniblue1/local_costmap/clearing_endpoints'),
        # 其他各种服务和状态话题重映射
        ('bond', '/omniblue1/bond'),
        ('cost_cloud', '/omniblue1/cost_cloud'),
        ('evaluation', '/omniblue1/evaluation'),
        ('initialpose', '/omniblue1/initialpose'),
        ('particle_cloud', '/omniblue1/particle_cloud'),
        ('parameter_events', '/omniblue1/parameter_events'),
        ('clicked_point', '/omniblue1/clicked_point'),
        # 转换事件重映射
        ('planner_server/transition_event', '/omniblue1/planner_server/transition_event'),
        ('controller_server/transition_event', '/omniblue1/controller_server/transition_event'),
        ('behavior_server/transition_event', '/omniblue1/behavior_server/transition_event'),
        ('bt_navigator/transition_event', '/omniblue1/bt_navigator/transition_event'),
        ('waypoint_follower/transition_event', '/omniblue1/waypoint_follower/transition_event'),
        ('velocity_smoother/transition_event', '/omniblue1/velocity_smoother/transition_event'),
        ('global_costmap/global_costmap/transition_event', '/omniblue1/global_costmap/global_costmap/transition_event'),
        ('local_costmap/local_costmap/transition_event', '/omniblue1/local_costmap/local_costmap/transition_event'),
        ('smoother_server/transition_event', '/omniblue1/smoother_server/transition_event'),
    ]
    
    # 将每个重映射转换为SetRemap动作
    robot1_remap_actions = [SetRemap(from_topic, to_topic) for from_topic, to_topic in robot1_remappings]
    
    # ==== 机器人2 ====
    # 机器人2 Cartographer SLAM节点
    robot2_cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='omniblue2_cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ("scan", "/omniblue2/scan"),
            ("odom", "/omniblue2/odom"),
            ("map", "/omniblue2/map"),
            ("submap_list", "/omniblue2/submap_list"),
            ("submap_query", "/omniblue2/submap_query"),
            ("trajectory_query", "/omniblue2/trajectory_query")
        ],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', 'omniblue2_lds_2d_withoutmap.lua'])
                   
    # 机器人2 占用栅格发布器
    robot2_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='omniblue2_occupancy_grid_node',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ("map", "/omniblue2/map"),
            ("submap_list", "/omniblue2/submap_list")
        ],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )
    
    # 机器人2的重映射设置
    robot2_remappings = [
        # 基本话题重映射
        ('cmd_vel', '/omniblue2/cmd_vel'),
        ('map', '/omniblue2/map'),
        ('map_server/map', '/omniblue2/map'),
        ('scan', '/omniblue2/scan'),
        ('odom', '/omniblue2/odom'),
        # 导航系统状态话题重映射
        ('plan', '/omniblue2/plan'),
        ('plan_smoothed', '/omniblue2/plan_smoothed'),
        ('received_global_plan', '/omniblue2/received_global_plan'),
        ('local_plan', '/omniblue2/local_plan'),
        ('transformed_global_plan', '/omniblue2/transformed_global_plan'),
        ('behavior_tree_log', '/omniblue2/behavior_tree_log'),
        ('goal_pose', '/omniblue2/goal_pose'),
        ('waypoints', '/omniblue2/waypoints'),
        # Costmap相关话题重映射
        ('global_costmap/costmap', '/omniblue2/global_costmap/costmap'),
        ('global_costmap/costmap_raw', '/omniblue2/global_costmap/costmap_raw'),
        ('global_costmap/costmap_updates', '/omniblue2/global_costmap/costmap_updates'),
        ('global_costmap/published_footprint', '/omniblue2/global_costmap/published_footprint'),
        ('global_costmap/footprint', '/omniblue2/global_costmap/footprint'),
        ('global_costmap/clearing_endpoints', '/omniblue2/global_costmap/clearing_endpoints'),
        ('local_costmap/costmap', '/omniblue2/local_costmap/costmap'),
        ('local_costmap/costmap_raw', '/omniblue2/local_costmap/costmap_raw'),
        ('local_costmap/costmap_updates', '/omniblue2/local_costmap/costmap_updates'),
        ('local_costmap/published_footprint', '/omniblue2/local_costmap/published_footprint'),
        ('local_costmap/footprint', '/omniblue2/local_costmap/footprint'),
        ('local_costmap/clearing_endpoints', '/omniblue2/local_costmap/clearing_endpoints'),
        # 其他各种服务和状态话题重映射
        ('bond', '/omniblue2/bond'),
        ('cost_cloud', '/omniblue2/cost_cloud'),
        ('evaluation', '/omniblue2/evaluation'),
        ('initialpose', '/omniblue2/initialpose'),
        ('particle_cloud', '/omniblue2/particle_cloud'),
        ('parameter_events', '/omniblue2/parameter_events'),
        ('clicked_point', '/omniblue2/clicked_point'),
        # 转换事件重映射
        ('planner_server/transition_event', '/omniblue2/planner_server/transition_event'),
        ('controller_server/transition_event', '/omniblue2/controller_server/transition_event'),
        ('behavior_server/transition_event', '/omniblue2/behavior_server/transition_event'),
        ('bt_navigator/transition_event', '/omniblue2/bt_navigator/transition_event'),
        ('waypoint_follower/transition_event', '/omniblue2/waypoint_follower/transition_event'),
        ('velocity_smoother/transition_event', '/omniblue2/velocity_smoother/transition_event'),
        ('global_costmap/global_costmap/transition_event', '/omniblue2/global_costmap/global_costmap/transition_event'),
        ('local_costmap/local_costmap/transition_event', '/omniblue2/local_costmap/local_costmap/transition_event'),
        ('smoother_server/transition_event', '/omniblue2/smoother_server/transition_event'),
    ]
    
    # 将每个重映射转换为SetRemap动作
    robot2_remap_actions = [SetRemap(from_topic, to_topic) for from_topic, to_topic in robot2_remappings]
    
    # 机器人1 Nav2启动组
    robot1_nav2_group = GroupAction([
        *robot1_remap_actions,
        
        # 为机器人1启动Nav2，使用唯一的节点名称
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': os.path.join(share_dir, 'config', 'omniblue1_cartographer_move_base.yaml'),
                'autostart': 'true',
                'map_server_name': 'omniblue1_map_server',
                'planner_server_name': 'omniblue1_planner_server',
                'controller_server_name': 'omniblue1_controller_server',
                'bt_navigator_name': 'omniblue1_bt_navigator',
                'waypoint_follower_name': 'omniblue1_waypoint_follower',
                'velocity_smoother_name': 'omniblue1_velocity_smoother',
                'lifecycle_manager_name': 'omniblue1_lifecycle_manager_navigation',
                'behavior_server_name': 'omniblue1_behavior_server',
                'smoother_server_name': 'omniblue1_smoother_server',
            }.items(),
        )
    ])
    
    # 机器人2 Nav2启动组
    robot2_nav2_group = GroupAction([
        *robot2_remap_actions,
        
        # 为机器人2启动Nav2，使用唯一的节点名称
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': os.path.join(share_dir, 'config', 'omniblue2_cartographer_move_base.yaml'),
                'autostart': 'true',
                'map_server_name': 'omniblue2_map_server',
                'planner_server_name': 'omniblue2_planner_server',
                'controller_server_name': 'omniblue2_controller_server',
                'bt_navigator_name': 'omniblue2_bt_navigator',
                'waypoint_follower_name': 'omniblue2_waypoint_follower',
                'velocity_smoother_name': 'omniblue2_velocity_smoother',
                'lifecycle_manager_name': 'omniblue2_lifecycle_manager_navigation',
                'behavior_server_name': 'omniblue2_behavior_server',
                'smoother_server_name': 'omniblue2_smoother_server',
            }.items(),
        )
    ])
    
    # RViz启动
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        # 基本参数声明
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        # 全局和机器人坐标系
        global_map_node,
        robot1_map_tf_node,
        robot2_map_tf_node,
            
        # 机器人1的Cartographer节点
        robot1_cartographer_node,
        robot1_occupancy_grid_node,
        
        # 机器人1的Nav2启动组（包含所有重映射）
        robot1_nav2_group,
        
        # 机器人2的Cartographer节点
        robot2_cartographer_node,
        robot2_occupancy_grid_node,
        
        # 机器人2的Nav2启动组（包含所有重映射）
        robot2_nav2_group,
        
        # RViz启动
        rviz_node,
    ])
