from launch_ros.actions import Node
from launch import LaunchDescription

from launch.action import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.event_handlers import (
    OnExecutionComplete, OnProcessExit, OnProcessIO,
    OnProcessStart, OnShutdown)

from launch.actions import (
    RegisterEventHandler, LogInfo, ExecuteProcess)

def generate_launch_description():
    
    lidar_launch_path = PathJoinSubstitution(
        [FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_s1_launch_tf2.py']
    )
    
    node1 = Node(
        package="robonav",
        executable="pub_joy_logitech",
        # output='screen'  # Display output in the terminal
    )

    node2 = Node(
        package="robonav",
        executable="robot_core",
        output='screen'  # Display output in the terminal
    )

    node3 = Node(
        package="robonav",
        executable="gpt_new",
        # output='screen'  # Display output in the terminal
    )

    return LaunchDescription([
        # node1,
        # RegisterEventHandler(
            # OnProcessStart(
            #     target_action=node1,
            #     on_start=[
            #         LogInfo(msg='Node1 joy stick started'),
            #         ExecuteProcess(
            #             cmd=['sleep', '3'],
            #             # output='screen',
            #             # emulate_tty=False,
            #             name='delay_process'
            #         ),
            #         node2,
            #     ]
            # )
        # ),
        node2,
        RegisterEventHandler(
            OnProcessStart(
                target_action=node2,
                on_start=[
                    LogInfo(msg='Node2 robot core started'),
                    node3,
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=node3,
                on_start=[
                    LogInfo(msg='Node3 gpt new started'),
                ]
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path),
        ),
        
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    LogInfo(msg='Launch file was asked to shutdown'),
                ]
            )
        )
    ])
