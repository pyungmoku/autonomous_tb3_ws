#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    # launch_nav2_bringup_dir = '/home/pmk/autonomous_tb3_ws/src/navigation2/nav2_bringup/bringup/launch'
    maze_path = os.path.join(get_package_share_directory('autonomous_tb3'), 'world', 'maze', 'model.sdf')
    config_dir = os.path.join(get_package_share_directory('autonomous_tb3'),'config')
    map_file = os.path.join(config_dir,'maze.yaml')
    params_file = os.path.join(config_dir,'tb3_nav_params.yaml')
    rviz_config= os.path.join(config_dir,'tb3_nav.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    x_pose = LaunchConfiguration('x_pose', default='-3.0')
    y_pose = LaunchConfiguration('y_pose', default='-7.0')
    
    # Names and poses of the robots
    # robot = {'name': 'robot1', 'x_pose': -3.0, 'y_pose': -7.0, 'z_pose': 0.01}

    # x_pose = TextSubstitution(text=str(robot['x_pose']))

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),

        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )


    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('autonomous_tb3'), 'launch', 'spawn_turtlebot3.launch.py')
            ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
)

    # spawn_turtlebot_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([launch_nav2_bringup_dir, '/spawn_tb3_launch.py']),
    #         launch_arguments={
    #             'robot_name': robot['name'],
    #             'turtlebot_type': TextSubstitution(text='waffle'),
    #             'x_pose': TextSubstitution(text=str(robot['x_pose'])),
    #             'y_pose': TextSubstitution(text=str(robot['y_pose'])),
    #             'z_pose': TextSubstitution(text=str(robot['z_pose'])),
    #         }.items()
    # )

    # spawn_turtlebot_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([launch_nav2_bringup_dir, '/multi_tb3_simulation_launch.py']),
            
    # )

    maze_spawner=Node(
        package='autonomous_tb3',
        executable='sdf_spawner',#'spawn_entity',
        name='maze_spawner',
        output='screen',
        arguments=[maze_path, "maze", '0.0', '0.0'],
    )

    # maze_mapping = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('autonomous_tb3'), 'launch', 'mapping.launch.py')
    #     )
    # )

    # maze_mapping = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
    #     )
    # )

    maze_nav=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'),'/launch','/bringup_launch.py']),
        launch_arguments={
        'map':map_file,
        'params_file': params_file}.items(),
    )

    rviz=Node(
        package='rviz2',
        output='screen',
        executable='rviz2',
        name='rviz2_node',
        arguments=['-d',rviz_config],
        # parameters=[{'use_sim_time': use_sim_time}],
        # remappings=[
        #     ('/tf', 'tf'),
        #     ('/tf_static', 'tf_static')
        # ],

    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(maze_spawner)
    # ld.add_action(maze_mapping)
    ld.add_action(maze_nav)
    ld.add_action(rviz)

    return ld
