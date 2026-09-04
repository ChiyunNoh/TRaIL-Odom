# -*- coding: utf-8 -*-
"""
    ros2 launch trail trail_garlileo.launch.py
"""
import os

from launch          import LaunchDescription
from launch.actions   import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    trail_share = get_package_share_directory('trail')

    default_cfg_path   = os.path.join(trail_share, 'dataset', 'GaRLILEO', 'config.yaml')
    default_bag_path   = '/path/to/your/dataset/Seq'

    default_start_time = '0'


    cfg_arg   = DeclareLaunchArgument('config_path', default_value=default_cfg_path)
    bag_arg   = DeclareLaunchArgument('rosbag_path', default_value=default_bag_path)
    start_arg = DeclareLaunchArgument('start_time',  default_value=default_start_time)

    cfg_path   = LaunchConfiguration('config_path')
    bag_path   = LaunchConfiguration('rosbag_path')
    start_time = LaunchConfiguration('start_time')


    trail_node = Node(
        package='trail',
        executable='trail_node',
        name='trail_node',
        output='screen',
        parameters=[{
            'config_path': cfg_path
        }]
    )

    # spot_msgs_node = Node(
    #     package='spot_msgs',
    #     executable='spot_msgs_node',
    #     name='spot_msgs_node2',
    #     output='screen'
    # )


    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path,
             '--clock',      
             '--start-offset', start_time,
             '-r', '1.0',
             '--log-level', 'error'],
        output='screen'
    )

    rviz_cfg = PathJoinSubstitution(
        [trail_share, 'config', 'trail_rviz.rviz'])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_cfg]
    )


    return LaunchDescription([
        cfg_arg, bag_arg, start_arg,        # launch arguments
        bag_play,
        trail_node,
        rviz,
    ])
