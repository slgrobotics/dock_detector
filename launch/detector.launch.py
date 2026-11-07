from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import yaml


def launch_detector(context):
    # Passing the parameter dictionary makes the same catalogue work in any
    # robot namespace, without depending on YAML's fully-qualified node matching.
    path = LaunchConfiguration('params_file').perform(context)
    with open(path) as stream:
        parameters = yaml.safe_load(stream)['dock_detector']['ros__parameters']
    parameters['use_sim_time'] = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    return [Node(package='dock_detector', executable='dock_detector', name='dock_detector',
                 namespace=LaunchConfiguration('namespace'), output='screen', parameters=[parameters],
                 remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')])]


def generate_launch_description():
    default = os.path.join(get_package_share_directory('dock_detector'), 'config', 'seggy_t.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('namespace', default_value=''),
        OpaqueFunction(function=launch_detector)])
