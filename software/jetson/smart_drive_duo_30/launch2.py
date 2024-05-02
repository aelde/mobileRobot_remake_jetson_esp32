from launch_ros.actions import Node
from launch import LaunchDescription

from launch.event_handlers import (OnExecutionComplete, OnProcessExit, OnProcessIO,
 				 OnProcessStart, OnShutdown)

from launch.actions import (RegisterEventHandler, LogInfo)

def generate_launch_description():
    node1 = Node(
        package="robonav",
        executable="pub_joy_logitech")
        
    node2 = Node(
        package="robonav",
        executable="robot_core")
    
    node3 = Node(
        package="robonav",
        executable="gpt_new")
    
    return LaunchDescription([
		node1,
		RegisterEventHandler(
			OnProcessStart(
				target_action=node1,
				on_start=[
					LogInfo(msg='Node1 joy stick started'),
					node2,
				]
			)
		),
		RegisterEventHandler(
			OnProcessStart(
				target_action=node2,
				on_start=[
					LogInfo(msg='Node2 robot core started'),
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
        )
		RegisterEventHandler(
			OnShutdown(
				on_shutdown= [
					LogInfo(msg= 'Launch file was asked to shutdown'),
				]
			)
		)

	])