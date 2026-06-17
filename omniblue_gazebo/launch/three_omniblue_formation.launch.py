import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml
from launch.actions import TimerAction


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


# ---------------- obstacles ----------------
# Static obstacles in the corridor between start (centroid ~(0, 0.7),
# robots within ~2.5 m) and goal (5, 3) (slots within ~1.3 m).
# All positions verified >1.3 m from every start/goal robot position.
OBSTACLES = [
    # course for goal (8, 3), 3 m formation. All obstacles >2.0 m from
    # every start/goal formation position and >2.6 m apart. Staggered
    # along the centroid corridor so the formation must weave through.
    {'name': 'obstacle_1', 'type': 'cylinder', 'x': 2.00, 'y': 3.75, 'radius': 0.25, 'height': 0.6},
    {'name': 'obstacle_2', 'type': 'box',      'x': 3.75, 'y': 1.75, 'sx': 0.5, 'sy': 0.5, 'sz': 0.6},
    {'name': 'obstacle_3', 'type': 'cylinder', 'x': 5.75, 'y': 0.00, 'radius': 0.25, 'height': 0.6},
    {'name': 'obstacle_4', 'type': 'box',      'x': 5.75, 'y': 4.00, 'sx': 0.5, 'sy': 0.5, 'sz': 0.6},
]


def obstacle_sdf(obs):
    """Generate a static obstacle SDF string."""
    if obs['type'] == 'cylinder':
        geom = f"""
            <cylinder>
              <radius>{obs['radius']}</radius>
              <length>{obs['height']}</length>
            </cylinder>"""
        z = obs['height'] / 2.0
    else:
        geom = f"""
            <box>
              <size>{obs['sx']} {obs['sy']} {obs['sz']}</size>
            </box>"""
        z = obs['sz'] / 2.0

    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{obs['name']}">
    <static>true</static>
    <pose>0 0 {z} 0 0 0</pose>
    <link name="link">
      <collision name="collision">
        <geometry>{geom}
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>{geom}
        </geometry>
        <material>
          <ambient>0.8 0.2 0.2 1</ambient>
          <diffuse>0.8 0.2 0.2 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def make_obstacle_nodes():
    """Write SDF files to /tmp and return spawn_entity nodes."""
    nodes = []
    for obs in OBSTACLES:
        sdf_path = f"/tmp/{obs['name']}.sdf"
        with open(sdf_path, 'w') as f:
            f.write(obstacle_sdf(obs))

        nodes.append(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', sdf_path,
                '-entity', obs['name'],
                '-x', str(obs['x']),
                '-y', str(obs['y']),
                '-z', '0.0',
            ],
            output='screen'
        ))
    return nodes
# --------------------------------------------


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

    # 生成障碍物 (spawn obstacles after world is up)
    launch_nodes.append(
        TimerAction(
            period=5.0,
            actions=make_obstacle_nodes()
        )
    )
    
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

    formation_manager = Node(
        package='gazebo_mecanum_plugins',
        executable='formation_manager',
        output='screen'
    )

    launch_nodes.extend([
        formation_manager,

        TimerAction(
            period=10.0,
            actions=[controller1]
        ),

        TimerAction(
            period=12.0,
            actions=[controller2]
        ),

        TimerAction(
            period=14.0,
            actions=[controller3]
        )
        
        # controller1,
        # controller2,
        # controller3
    ])
        
    launch_nodes.append(display_rviz)
    
    # 返回启动描述
    return LaunchDescription(launch_nodes)