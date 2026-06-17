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
    omniblue_description_path = os.path.join(
        get_package_share_directory('omniblue_description'))
    
    # 指定自定义仓库世界文件的完整路径
    warehouse_world_path = os.path.join(
        omniblue_gazebo_path,
        'world',
        'warehouse_small.world'
    )
    
    # 设置模型路径
    models_path = os.path.join(omniblue_gazebo_path, 'world')
    env = {'GAZEBO_MODEL_PATH': f"{models_path}:{os.environ.get('GAZEBO_MODEL_PATH', '')}"}
    
    # 直接启动Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', warehouse_world_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        additional_env=env
    )
    
    # RViz配置
    rviz_config_dir = os.path.join(
        omniblue_gazebo_path,
        'rviz',
        'basic_data.rviz'
    )
    
    # RViz可视化
    display_rviz = Node(
        package='rviz2', 
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )
    
    # 定义机器人位置列表 [id, x, y, z, yaw]
    robot_positions = [
        [1, 4.5, -4.0, 0.1, 0.0],
        [2, 5.7, -3.2, 0.1, 0.0],
        [3, 5.6, -5.0, 0.1, 1.57]
    ]
    
    # 创建所有节点列表，从gazebo开始
    nodes_list = [gazebo]
    
    # 循环创建多个机器人
    for robot_id, x, y, z, yaw in robot_positions:
        # 设置机器人名称和命名空间
        robot_name = f'omniblue{robot_id}'
        robot_namespace = f'omniblue{robot_id}'
        
        # XACRO文件路径
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
            parameters=[{'use_sim_time': True},
            {'frame_prefix': f"{robot_namespace}/"}  # 添加正确的命名空间前缀
            ]
        )    
           
        # 在Gazebo中生成机器人
        spawn_robot = Node(
            package='gazebo_ros', 
            executable='spawn_entity.py',
            arguments=[
                '-topic', f'/{robot_namespace}/robot_description',
                '-entity', robot_name,
                '-robot_namespace', robot_namespace,
                '-x', str(x),
                '-y', str(y),
                '-z', str(z),
                '-Y', str(yaw),
            ],
            output='screen'
        )
        
        # 将本机器人的节点添加到节点列表中
        nodes_list.extend([robot_state_publisher, robot_joint_state_publisher, spawn_robot])
    
    # 返回启动描述
    return LaunchDescription(nodes_list)
