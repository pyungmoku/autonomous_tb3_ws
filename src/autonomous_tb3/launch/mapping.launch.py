from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('autonomous_tb3'),'config')
    # config_dir = '/home/pmk/autonomous_tb3_ws/src/autonomous_tb3/config'
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            output='screen',
            executable='cartographer_node',
            name='cr_node',
            arguments=['-configuration_directory', config_dir,'-configuration_basename','turtlebot3_lds_2d.lua']
        ),
        Node(
            package='cartographer_ros',
            output='screen',
            executable='occupancy_grid_node',
            name='cr_oc_node'
        ),
    ])