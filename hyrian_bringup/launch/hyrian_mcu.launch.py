#!/usr/bin/env python3

# Author: John-woohyeon

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
  hyrian_mcu_parameter = LaunchConfiguration(
    'hyrian_mcu_parameter',
    default=os.path.join(
      get_package_share_directory('hyrian_bringup'),
      'param/hyrian_mcu.yaml'
    )
  )

  return LaunchDescription([
    DeclareLaunchArgument(
      'hyrian_mcu_parameter',
      default_value=hyrian_mcu_parameter
    ),

    Node(
      package='hyrian_bringup',
      executable='hyrian_mcu_node',
      name='hyrian_mcu_node',
      output='screen',
      emulate_tty=True,
      parameters=[hyrian_mcu_parameter],
      namespace='',
    )
  ])