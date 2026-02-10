from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster"
        ]
    )

    simple_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "simple_velocity_controller"
        ]
    )

    return LaunchDescription([
        joint_state_broadcaster,
        simple_velocity_controller
    ])
