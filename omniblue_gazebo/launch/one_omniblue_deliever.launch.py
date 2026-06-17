import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution
import xacro

def generate_launch_description():
    # 获取包路径
    omniblue_gazebo_path = get_package_share_directory('omniblue_gazebo')
    
    # 指定自定义仓库世界文件的完整路径
    warehouse_world_path = os.path.join(
        omniblue_gazebo_path,
        'world',
        'deliever_environment.world'
    )
    
    
    # 直接启动Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', warehouse_world_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen',
    )
    
    # 加载机器人描述
    omniblue_description_path = os.path.join(
        get_package_share_directory('omniblue_description'))
    
    # RViz配置
    rviz_config_dir = os.path.join(
        omniblue_gazebo_path,
        'rviz',
        'actor_lidar.rviz'
    )
    
    # 单个机器人 - 使用固定值而非LaunchConfiguration
    robot_name = 'omniblue'
    robot_namespace = 'omniblue'
    
    robot_xacro_file = os.path.join(omniblue_description_path, 'urdf', 'omniblue.urdf.xacro')
    
    # 使用参数映射来处理XACRO - 传入字符串
    robot_mappings = {
        'robot_name': robot_name,
        'robot_namespace': robot_namespace
    }
    
    # 处理机器人的URDF
    robot_description_config = xacro.process_file(
        robot_xacro_file, 
        mappings=robot_mappings
    ).toxml()
    
    # 将机器人描述添加到参数中
    robot_description = {'robot_description': robot_description_config}
    
    # 机器人的状态发布器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_namespace,
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': True},
            {'publish_frequency': 30.0},
            {'ignore_timestamp': True},  # 忽略时间戳差异
            {'frame_prefix': f"{robot_namespace}/"}  # 添加正确的命名空间前缀
        ]
    )

    # 修改机器人关节状态发布器
    robot_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=robot_namespace,
        output='screen',
        parameters=[{'use_sim_time': True}]
    )    
       
    # 在Gazebo中生成机器人
    spawn_robot = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-topic', f'/{robot_namespace}/robot_description',
            '-entity', robot_name,
            '-robot_namespace', robot_namespace,
            '-x', '-14',
            '-y', '16.0',  # 仓库环境中的位置
            '-z', '0.1',
            '-Y', '0.0',
        ],
        output='screen'
    )
    
    # RViz可视化
    display_rviz = Node(
        package='rviz2', 
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )
    
    # 返回启动描述
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        robot_joint_state_publisher,
        spawn_robot,
        display_rviz
    ])
