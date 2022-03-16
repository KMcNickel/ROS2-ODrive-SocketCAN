from launch import LaunchDescription
from launch_ros.actions import Node

device_name = "agv0"
can_interface = "can0"

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="socketcan",
            executable="receiver",
            namespace=device_name + "/" + can_interface,
            name="receiver",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"interface_name": can_interface}
            ],
            remappings=[
                ("socketcan/receiver/data", "receiver/data")
            ]
        ),
        Node(
            package="socketcan",
            executable="sender",
            namespace=device_name + "/" + can_interface,
            name="sender",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"interface_name": can_interface}
            ],
            remappings=[
                ("socketcan/sender/data", "sender/data")
            ]
        )
    ])