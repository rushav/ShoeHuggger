import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_path = os.path.join(
        os.path.expanduser('~'),
        'turtlebot4_ws', 'src', 'alienbot_description', 'urdf', 'robot_v4.urdf'
    )

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
    ])