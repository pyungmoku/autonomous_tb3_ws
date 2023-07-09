#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    maze_path = os.path.join(get_package_share_directory('autonomous_tb3'), 'world', 'maze', 'model.sdf')
    config_dir = os.path.join(get_package_share_directory('autonomous_tb3'),'config')
    map_file = os.path.join(config_dir,'maze.yaml')
    params_file = os.path.join(config_dir,'tb3_nav_params.yaml')
    rviz_config= os.path.join(config_dir,'tb3_nav.rviz')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # x_pose = LaunchConfiguration('x_pose', default='-3.0')
    # y_pose = LaunchConfiguration('y_pose', default='-7.0')

    return LaunchDescription([
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        #     ),
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'empty_world.launch.py')),
        ),


        Node(
            package='autonomous_tb3',
            executable='sdf_spawner',#'spawn_entity',
            name='maze_spawner',
            output='screen',
            arguments=[maze_path, "maze", '0.0', '0.0'],
        ),
    ])