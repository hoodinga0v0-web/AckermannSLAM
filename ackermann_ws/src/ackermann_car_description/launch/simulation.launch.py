"""Start Gazebo, bridge, command guard, robot, and controllers in order."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import Action, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import (
    Command,
    FindExecutable,
    IfElseSubstitution,
    LaunchConfiguration,
)
from launch.utilities import (
    normalize_to_list_of_substitutions,
    perform_substitutions,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


class LogError(Action):
    """Log at error level; Jazzy only ships LogInfo as a built-in action."""

    def __init__(self, *, msg):
        super().__init__()
        self._msg = normalize_to_list_of_substitutions(msg)

    def execute(self, context):
        get_logger('launch').error(perform_substitutions(context, self._msg))


def actions_after_exit(returncode, next_action, label):
    """Return the next action only on success, otherwise stop the launch."""
    if returncode == 0:
        return [] if next_action is None else [next_action]
    return [
        LogError(msg=f'{label} failed: rc={returncode}'),
        EmitEvent(event=Shutdown(reason=f'{label} failed')),
    ]


def next_only_on_success(next_action, label):
    """Build an OnProcessExit callback with explicit return-code handling."""
    def _on_exit(event, _context):
        return actions_after_exit(event.returncode, next_action, label)

    return _on_exit


def generate_launch_description():
    package_share = get_package_share_directory('ackermann_car_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(package_share, 'urdf', 'ackermann_car.urdf.xacro')
    controllers_file = os.path.join(package_share, 'config', 'controllers.yaml')
    guard_file = os.path.join(package_share, 'config', 'command_guard.yaml')
    bridge_file = os.path.join(package_share, 'config', 'bridge.yaml')
    default_world = os.path.join(package_share, 'worlds', 'slam_world.sdf')
    rviz_config = os.path.join(package_share, 'config', 'ackermann_car.rviz')
    mesh_prefix = 'package://ackermann_car_description/meshes'

    world = LaunchConfiguration('world')
    verbosity = LaunchConfiguration('verbosity')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')
    rviz_fixed_frame = LaunchConfiguration('rviz_fixed_frame')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            xacro_file,
            ' controllers_file:=',
            controllers_file,
            ' mesh_prefix:=',
            mesh_prefix,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [
                IfElseSubstitution(
                    headless,
                    if_value='-s ',
                    else_value='',
                ),
                '-r -v ',
                verbosity,
                ' ',
                world,
            ],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_file}],
    )

    command_guard = Node(
        package='ackermann_command_guard',
        executable='command_guard',
        name='ackermann_command_guard',
        output='screen',
        parameters=[
            guard_file,
            {
                'use_sim_time': True,
                'input_topic': '/cmd_vel_raw',
                'output_topic': '/cmd_vel',
            },
        ],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_ackermann_car',
        output='screen',
        arguments=[
            '-world',
            'slam_world',
            '-topic',
            'robot_description',
            '-name',
            'ackermann_car',
            '-allow_renaming',
            'false',
        ],
    )

    spawner_common = [
        '--controller-manager',
        '/controller_manager',
        '--controller-manager-timeout',
        '60',
        '--switch-timeout',
        '60',
    ]
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=['joint_state_broadcaster', *spawner_common],
    )
    ackermann_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='ackermann_steering_controller_spawner',
        output='screen',
        arguments=[
            'ackermann_steering_controller',
            '--param-file',
            controllers_file,
            '--controller-ros-args',
            '--remap /ackermann_steering_controller/reference:=/cmd_vel',
            '--controller-ros-args',
            '--remap /ackermann_steering_controller/odometry:=/odom',
            '--controller-ros-args',
            '--remap /ackermann_steering_controller/tf_odometry:=/tf',
            *spawner_common,
        ],
    )

    spawn_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn,
            on_exit=next_only_on_success(
                joint_state_broadcaster_spawner,
                'robot spawn',
            ),
        )
    )
    joint_state_broadcaster_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=next_only_on_success(
                ackermann_controller_spawner,
                'joint_state_broadcaster spawner',
            ),
        )
    )
    ackermann_controller_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=ackermann_controller_spawner,
            on_exit=next_only_on_success(
                None,
                'ackermann_steering_controller spawner',
            ),
        )
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config, '-f', rviz_fixed_frame],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Absolute path to the Gazebo world.',
        ),
        DeclareLaunchArgument(
            'verbosity',
            default_value='3',
            description='Gazebo verbosity from 0 through 4.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run only the Gazebo server by adding -s.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Start RViz with the project configuration.',
        ),
        DeclareLaunchArgument(
            'rviz_fixed_frame',
            default_value='odom',
            description='RViz fixed frame (map for SLAM, odom for simulation).',
        ),
        robot_state_publisher,
        gazebo,
        bridge,
        command_guard,
        spawn_exit,
        joint_state_broadcaster_exit,
        ackermann_controller_exit,
        spawn,
        rviz_node,
    ])
