from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    diff_drive_node = Node(
        package='amr_handler',
        executable='diff_drive_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    odometry_node = Node(
        package='amr_handler',
        executable='odometry_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    return LaunchDescription([use_sim_time_arg, diff_drive_node, odometry_node])
