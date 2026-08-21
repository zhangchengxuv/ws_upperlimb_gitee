import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    upperlimb_share = get_package_share_directory('upperlimb_robot')
    gazebo_share = get_package_share_directory('gazebo_ros')

    urdf_file = os.path.join(upperlimb_share, 'urdf', 'robot_arms.urdf')
    with open(urdf_file, 'r', encoding='utf-8') as urdf:
        robot_description = urdf.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')
        )
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_upperlimb_robot',
        output='screen',
        arguments=[
            '-entity',
            'upperlimb_robot',
            '-file',
            urdf_file,
        ],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])
