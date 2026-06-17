import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('omniblue_gazebo'),
                'launch',
                'empty_world.launch.py'
            )
        )
    )

    description_path = os.path.join(
        get_package_share_directory('omniblue_description')
    )

    xacro_file = os.path.join(
        description_path,
        'urdf',
        'omniblue.urdf.xacro'
    )

    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            'robot_name': 'omniblue',
            'robot_namespace': 'omniblue'
        }
    ).toxml()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='omniblue',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/omniblue/robot_description',
            '-entity', 'omniblue',
            '-robot_namespace', 'omniblue',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    spawn_obstacle = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'cylinder_obstacle',
            '-database', 'construction_barrel',
            '-x', '4.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn_robot,
        spawn_obstacle,
    ])