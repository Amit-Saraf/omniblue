import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # 加载Gazebo空世界
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('omniblue_gazebo'), 'launch'), 
            '/empty_world.launch.py'])
    )
    
    # 加载机器人描述
    omniblue_description_path = os.path.join(
        get_package_share_directory('omniblue_description'))
    
    # RViz配置
    rviz_config_dir = os.path.join(
        omniblue_description_path,
        'config',
        'omniblue_demo.rviz'
    )
    
    # 单个机器人
    robot_name = 'omniblue'
    robot_namespace = 'omniblue'
    
    robot_xacro_file = os.path.join(omniblue_description_path, 'urdf', 'omniblue.urdf.xacro')
    
    # 使用参数映射来处理XACRO
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
        parameters=[robot_description]
    )
    
    # 机器人的关节状态发布器
    robot_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=robot_namespace,
        output='screen',
    )
    
    # 添加 odom 到 base_footprint 的转换发布器
    odom_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=robot_namespace,
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )
    
    # 在Gazebo中生成机器人
    spawn_robot = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-topic', f'/{robot_namespace}/robot_description',
            '-entity', robot_name,
            '-robot_namespace', robot_namespace,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.',
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
        odom_to_base_footprint,  # 添加了odom到base_footprint的转换
        spawn_robot,
        #display_rviz
    ])
