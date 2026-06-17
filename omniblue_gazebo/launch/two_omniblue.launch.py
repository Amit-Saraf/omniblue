import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml

# 辅助函数
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
    
    # ==================== 机器人1 ====================
    # 加载机器人1的XACRO文件（使用参数）
    robot1_name = 'omniblue_1'
    robot1_namespace = 'omniblue1'
    
    robot1_xacro_file = os.path.join(omniblue_description_path, 'urdf', 'omniblue.urdf.xacro')
    
    # 使用参数映射来处理XACRO
    robot1_mappings = {
        'robot_name': robot1_name,
        'robot_namespace': robot1_namespace
    }
    
    # 处理机器人1的URDF
    robot1_description_config = xacro.process_file(
        robot1_xacro_file, 
        mappings=robot1_mappings
    ).toxml()
    
    # 将机器人1的描述添加到参数中
    robot1_description = {'robot_description': robot1_description_config}
    
    # 机器人1的状态发布器
    robot1_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot1_namespace,
        output='screen',
        parameters=[robot1_description]
    )
    
    # 机器人1的关节状态发布器
    robot1_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=robot1_namespace,
        output='screen',
    )
    
    # 在Gazebo中生成机器人1
    spawn_robot1 = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-topic', f'/{robot1_namespace}/robot_description',
            '-entity', robot1_name,
            '-robot_namespace', robot1_namespace,
            '-x', '-1.0',
            '-y', '0.0',
            '-z', '0.1',
            '-Y', '0.0',
        ],
        output='screen'
    )
    
    # ==================== 机器人2 ====================
    # 加载机器人2的XACRO文件（使用参数）
    robot2_name = 'omniblue_2'
    robot2_namespace = 'omniblue2'
    
    # 使用参数映射来处理XACRO
    robot2_mappings = {
        'robot_name': robot2_name,
        'robot_namespace': robot2_namespace
    }
    
    # 处理机器人2的URDF
    robot2_description_config = xacro.process_file(
        robot1_xacro_file,  # 使用同一个文件，但参数不同
        mappings=robot2_mappings
    ).toxml()
    
    # 将机器人2的描述添加到参数中
    robot2_description = {'robot_description': robot2_description_config}
    
    # 机器人2的状态发布器
    robot2_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot2_namespace,
        output='screen',
        parameters=[robot2_description]
    )
    
    # 机器人2的关节状态发布器
    robot2_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=robot2_namespace,
        output='screen',
    )
    
    # 在Gazebo中生成机器人2
    spawn_robot2 = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-topic', f'/{robot2_namespace}/robot_description',
            '-entity', robot2_name,
            '-robot_namespace', robot2_namespace,
            '-x', '1.0',
            '-y', '0.0',
            '-z', '0.1',
            '-Y', '3.1415',
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
        robot1_state_publisher,
        robot1_joint_state_publisher,
        spawn_robot1,
        robot2_state_publisher,
        robot2_joint_state_publisher,
        spawn_robot2,
        display_rviz
    ])
