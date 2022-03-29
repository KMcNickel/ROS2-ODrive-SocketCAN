from launch import LaunchDescription
from launch_ros.actions import Node

device_name = "agv0"
axis_number = 0
odrive_topic_prefix = "/" + device_name + "/axis" + str(axis_number) + "/odrive"

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="steer_axis_angle",
            executable="steer_axis_angle",
            namespace=device_name + "/axis" + str(axis_number),
            name="angle",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"axis_number": axis_number},
                {"axis_gear_ratio": 2.5},
            ],
            remappings=[
                ("angle/input/odrive/status", odrive_topic_prefix + "/output/status"),
                ("angle/output/current", "output/current"),
                ("angle/input/zero", "input/zero"),
            ]
        )
    ])