from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    deviceName = LaunchConfiguration('device_name')
    canbusInterfaceName = LaunchConfiguration('canbus_interface_name')

    deviceNameLaunchArg = DeclareLaunchArgument(
        'device_name',
        default_value = 'agv0'
    )

    canbusInterfaceNameLaunchArg = DeclareLaunchArgument(
        'canbus_interface_name',
        default_value = 'can0'
    )

    canReceiverNode = Node(
        package="socketcan",
        executable="receiver",
        namespace = [deviceName, "/", canbusInterfaceName],
        name="receiver",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"interface_name": canbusInterfaceName}
        ],
        remappings=[
            ("socketcan/receiver/output/data", "output/data")
        ]
    )

    canSenderNode = Node(
        package="socketcan",
        executable="sender",
        namespace = [deviceName, "/", canbusInterfaceName],
        name="sender",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"interface_name": canbusInterfaceName}
        ],
        remappings=[
            ("socketcan/sender/input/data", "input/data")
        ]
    )

    return LaunchDescription([
        deviceNameLaunchArg,
        canbusInterfaceNameLaunchArg,
        canReceiverNode,
        canSenderNode
    ])