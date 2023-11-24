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


def generate_launch_description():
  hyrian_mcu_parameter = LaunchConfiguration(
    'hyrian_mcu_parameter',
    default=os.path.join(
      get_package_share_directory('hyrian_bringup'),
      'param/hyrian_mcu.yaml'
    )
  )

  hyrian_lidar_parameter = LaunchConfiguration(
    'hyrian_lidar_parameter',
    default=os.path.join(
      get_package_share_directory('hyrian_bringup'),
      'param/hyrian_lidar.yaml'
    )
  )

  use_sim_time = LaunchConfiguration('use_sim_time', default='true') ## false -> true
  hyrian_description_dir = LaunchConfiguration(
    'hyrian_description_dir',
    default=os.path.join(
      get_package_share_directory('hyrian_description'),
      'launch'
    )
  )

  return LaunchDescription([
    DeclareLaunchArgument(
      'hyrian_mcu_parameter',
      default_value=hyrian_mcu_parameter
    ),

    DeclareLaunchArgument(
      'hyrian_lidar_parameter',
      default_value=hyrian_lidar_parameter
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/hyrian_mcu.launch.py']),
      launch_arguments={'hyrian_mcu_parameter': hyrian_mcu_parameter}.items()
    ),
    
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/hyrian_lidar.launch.py']),
      launch_arguments={'hyrian_lidar_parameter': hyrian_lidar_parameter}.items()
    ),
    
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([hyrian_description_dir, '/hyrian_state_publisher.launch.py']),
      launch_arguments={'use_sim_time': use_sim_time}.items(),
    ),
  ])