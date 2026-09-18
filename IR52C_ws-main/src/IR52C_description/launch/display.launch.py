import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'IR52C_description'
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'arm.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_raw}]
        ),
        # New Telemetry Node instead of Sliders
        Node(
            package=pkg_name,
            executable='telemetry_node.py',
            name='ir52c_telemetry',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        )
    ])