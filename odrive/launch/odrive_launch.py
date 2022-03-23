from launch import LaunchDescription
from launch_ros.actions import Node

device_name = "agv0"
canbus_interface_name = "can0"
canbus_interface_namespace = "/" + device_name + "/" + canbus_interface_name
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
                {"axis_number": axis_number},
                {"calibration_type": 1},
                {"publish_debug_messages": True}
            ],
            remappings=[
                ("odrive/input/can", canbus_interface_namespace + "/output/data"),
                ("odrive/output/can", canbus_interface_namespace + "/input/data"),
                ("odrive/output/status", "output/status"),
                ("odrive/input/position", "input/position"),
                ("odrive/input/velocity", "input/velocity"),
                ("odrive/input/start", "input/start"),
                ("odrive/input/shutdown", "input/shutdown"),
                ("odrive/input/clearErrors", "input/clearErrors"),
                ("odrive/input/reboot", "input/reboot"),
            ]
        )
    ])