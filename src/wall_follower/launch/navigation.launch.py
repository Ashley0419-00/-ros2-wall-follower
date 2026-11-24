"""Stage 2: nav2 on the saved map, then visit the markers in the marker file's order.

ros2 launch wall_follower navigation.launch.py map:=$HOME/map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package = get_package_share_directory('wall_follower')
    nav2 = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    sim = LaunchConfiguration('use_sim_time')
    clock = {'use_sim_time': ParameterValue(sim, value_type=bool)}

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/map.yaml'),
                              description='Map saved by map_saver_cli after the wall-follower run'),
        DeclareLaunchArgument('marker_file',
                              default_value=os.path.expanduser('~/wall_follower_markers.csv')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            package, 'launch', 'nav2_params.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, 'bringup_launch.py')),
            launch_arguments={'map': LaunchConfiguration('map'),
                              'params_file': LaunchConfiguration('params_file'),
                              'use_sim_time': sim,
                              'autostart': 'true'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, 'rviz_launch.py')),
            launch_arguments={'use_sim_time': sim}.items(),
            condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='wall_follower', executable='marker_navigator.py',
             name='marker_navigator', output='screen',
             parameters=[clock, {'marker_file': LaunchConfiguration('marker_file')}]),
    ])
