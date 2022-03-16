from launch import LaunchDescription
from launch_ros.actions import Node

device_name = "agv0"
canbus_interface_name = "can0"
axis_number = 0

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="odrive",
            executable="odrive",
            namespace=device_name + "/axis" + str(axis_number),
            name="odrive",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"axis_number": axis_number}
            ],
            remappings=[
                ("odrivecan/receiver/data", "/" + device_name + "/" + canbus_interface_name + "/receiver/data"),
                ("odrivecan/sender/data", "/" + device_name + "/" + canbus_interface_name + "/sender/data"),
            ]
        )
    ])