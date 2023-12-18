from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pappap',
            executable='controller.py',
            name='controller_node',
        ),
        Node(
            package='pappap',
            executable='L_W_sen.py',
            name='L_W_Sensor_node',
        ),
        Node(
            package='pappap',
            executable='R_W_sen.py',
            name='R_W_Sensor_node',  
        ),
        Node(
            package='pappap',
            executable='L_W_driver.py',
            name='L_W_driver_node',
        ),
        Node(
          package='pappap',
          executable='R_W_driver.py',
          name='R_W_driver_node',  
        ),
        # ExecuteProcess(
        #     cmd=['ros2', 'topic', 'list'],
        #     output='screen'
        # )
    ]) 