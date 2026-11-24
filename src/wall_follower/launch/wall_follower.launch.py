from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim = LaunchConfiguration('use_sim_time')
    clock = {'use_sim_time': ParameterValue(sim, value_type=bool)}
    camera_check = Node(package='wall_follower', executable='wait_for_camera.py',
                        condition=UnlessCondition(sim), output='screen')

    def controller(*, speed_limits=None, **kwargs):
        return Node(package='wall_follower', executable='wall_follower',
                    name='wall_follower', parameters=[clock, speed_limits or {}], **kwargs)

    def camera_checked(event, context):
        if event.returncode == 0 and not context.is_shutdown:
            return [controller(speed_limits={'max_linear_speed': 0.25, 'turn_speed_scale': 0.60})]
        return []

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('camera_frame', default_value='camera_rgb_optical_frame'),
        # Stage 2 marker file: start pose + markers, rewritten every second during the run.
        DeclareLaunchArgument('marker_file', default_value='~/wall_follower_markers.csv'),
        RegisterEventHandler(OnProcessExit(target_action=camera_check, on_exit=camera_checked)),
        Node(package='wall_follower', executable='see_marker.py', name='see_marker',
             parameters=[clock, {'camera_frame': LaunchConfiguration('camera_frame')}]),
        Node(package='wall_follower', executable='point_transformer.py',
             name='point_transformer',
             parameters=[clock, {'landmarks_file': LaunchConfiguration('marker_file')}]),
        controller(condition=IfCondition(sim)),
        camera_check,
    ])
