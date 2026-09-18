import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'IR52C_description'
    
    # Path to your xacro file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'arm.urdf.xacro')
    
    # Process Xacro to generate URDF string
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        # Robot State Publisher: Broadcasts the TF tree
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_raw}]
        ),
        # Joint State Publisher GUI: The Sliders
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])