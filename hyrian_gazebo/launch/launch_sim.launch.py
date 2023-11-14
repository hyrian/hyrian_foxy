import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration



def generate_launch_description():

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('hyrian_gazebo'),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    gazebo_params_path = os.path.join(
                  get_package_share_directory('hyrian_gazebo'),'config','gazebo_params.yaml')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'verbose': 'true', 'world': os.path.join(get_package_share_directory('hyrian_gazebo'), 'worlds', 'turtlebot3.world'),'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path}.items()
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'hyrian',
                                   '-x', '-2',
                                   '-y', '-0.5',
                                   '-z', '0.0575',],
                        output='screen')
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_drive_controller"],
    )

    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(get_package_share_directory('hyrian_bringup'), 'config', 'ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
    )

    rviz2_node = ExecuteProcess(
            cmd=["ros2", "run", "rviz2", "rviz2"], output="screen"
        )    
    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        robot_localization_node,
        # rviz2_node

    ])
