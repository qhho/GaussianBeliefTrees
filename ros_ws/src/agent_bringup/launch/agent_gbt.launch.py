from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    agent_gbt = Node(
        package='agent_gbt',
        executable='agent_gbt',
        output='screen',
        shell=True,
    )

    return LaunchDescription([
        agent_gbt,
    ])
