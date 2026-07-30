"""Start one simulation instance and online asynchronous SLAM Toolbox."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory('ackermann_car_description')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    simulation_launch = os.path.join(
        package_share,
        'launch',
        'simulation.launch.py',
    )
    slam_toolbox_launch = os.path.join(
        slam_toolbox_share,
        'launch',
        'online_async_launch.py',
    )
    slam_params = os.path.join(package_share, 'config', 'slam.yaml')

    rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world')
    verbosity = LaunchConfiguration('verbosity')
    headless = LaunchConfiguration('headless')

    return LaunchDescription([
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch),
            launch_arguments={
                'slam_params_file': slam_params,
                'use_sim_time': 'true',
                'autostart': 'true',
                'use_lifecycle_manager': 'false',
            }.items(),
        ),
    ])
