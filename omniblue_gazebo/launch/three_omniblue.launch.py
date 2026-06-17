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
    
    robots_info = [
        {
            'name': 'omniblue1',
            'namespace': 'omniblue1',
            'x': '-2.0',
            'y': '0.0',
            'yaw': '0.0'
        },
        {
            'name': 'omniblue2',
            'namespace': 'omniblue2',
            'x': '0.0',
            'y': '2.0',
            'yaw': '1.5708'  # 90度
        },
        {
            'name': 'omniblue3',
            'namespace': 'omniblue3',
            'x': '2.0',
            'y': '0.0',
            'yaw': '3.1415'  # 180度
        }
    ]
    
    robot_xacro_file = os.path.join(omniblue_description_path, 'urdf', 'omniblue.urdf.xacro')
    
    # 存储所有节点的列表
    launch_nodes = [gazebo]
    
    # 为每个机器人创建节点
    for robot in robots_info:
        # 使用参数映射来处理XACRO
        robot_mappings = {
            'robot_name': robot['name'],
            'robot_namespace': robot['namespace']
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
            namespace=robot['namespace'],
            output='screen',
            parameters=[robot_description]
        )
        
        # 机器人的关节状态发布器
        robot_joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace=robot['namespace'],
            output='screen',
        )
        
        # 在Gazebo中生成机器人
        spawn_robot = Node(
            package='gazebo_ros', 
            executable='spawn_entity.py',
            arguments=[
                '-topic', f'/{robot["namespace"]}/robot_description',
                '-entity', robot['name'],
                '-robot_namespace', robot['namespace'],
                '-x', robot['x'],
                '-y', robot['y'],
                '-z', '0.1',
                '-Y', robot['yaw'],
            ],
            output='screen'
        )
        
        # 添加到启动节点列表
        launch_nodes.extend([
            robot_state_publisher,
            robot_joint_state_publisher,
            spawn_robot
        ])
    
    # RViz可视化
    display_rviz = Node(
        package='rviz2', 
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    controller1 = Node(
        package='gazebo_mecanum_plugins',
        executable='formation_smc_controller',
        namespace='omniblue1',
        output='screen'
    )

    controller2 = Node(
        package='gazebo_mecanum_plugins',
        executable='formation_smc_controller',
        namespace='omniblue2',
        output='screen'
    )

    controller3 = Node(
        package='gazebo_mecanum_plugins',
        executable='formation_smc_controller',
        namespace='omniblue3',
        output='screen'
    )

    launch_nodes.extend([
        controller1,
        controller2,
        controller3
    ])
        
    launch_nodes.append(display_rviz)
    
    # 返回启动描述
    return LaunchDescription(launch_nodes)
