"""Start one simulation instance and SLAM Toolbox pose-graph localization."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    package_share = get_package_share_directory('ackermann_car_description')
    simulation_launch = os.path.join(
        package_share,
        'launch',
        'simulation.launch.py',
    )
    localization_params = os.path.join(
        package_share,
        'config',
        'slam_localization.yaml',
    )

    map_file_name = LaunchConfiguration('map_file_name')
    rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world')
    verbosity = LaunchConfiguration('verbosity')
    headless = LaunchConfiguration('headless')
    autostart = LaunchConfiguration('autostart')

    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            localization_params,
            {
                'map_file_name': ParameterValue(
                    map_file_name,
                    value_type=str,
                ),
                'use_sim_time': True,
                'use_lifecycle_manager': False,
            },
        ],
    )

    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart),
    )
    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='SLAM Toolbox localization is activating.'),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(autostart),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file_name',
            description='Pose-graph base path, without .posegraph or .data.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz from the included simulation launch.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(package_share, 'worlds', 'slam_world.sdf'),
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
            'autostart',
            default_value='true',
            description='Configure and activate localization automatically.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation_launch),
            launch_arguments={
                'rviz': rviz,
                'rviz_fixed_frame': 'map',
                'world': world,
                'verbosity': verbosity,
                'headless': headless,
            }.items(),
        ),
        slam_toolbox,
        configure_slam,
        activate_slam,
    ])
