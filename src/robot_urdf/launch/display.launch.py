from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_path = get_package_share_directory('robot_urdf')
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        # 角度 GUI
        Node(
            package='robot_urdf',
            executable='joint_deg_gui',
            name='joint_deg_gui'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        )

    ])