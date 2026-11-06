from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    default = os.path.join(get_package_share_directory('dock_detector'), 'config', 'seggy.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('namespace', default_value=''),
        Node(package='dock_detector', executable='dock_detector', name='dock_detector',
             namespace=LaunchConfiguration('namespace'), output='screen',
             parameters=[LaunchConfiguration('params_file'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}],
             remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')])])
