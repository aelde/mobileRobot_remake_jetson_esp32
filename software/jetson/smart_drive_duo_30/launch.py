from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robonav',
            executable='robot_core',
            name='robot_core',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='robonav',
            executable='pub_joy_logitech',
            name='pub_joy_logitech',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='robonav',
            executable='gpt_new',
            name='gpt_new',
            output='screen',
            emulate_tty=True,
        ),
    ])