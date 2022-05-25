from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    deviceName = LaunchConfiguration('robot_name')
    canbusInterfaceName = LaunchConfiguration('canbus_interface_name')
    axisNumber = LaunchConfiguration('axis_number')

    deviceNameLaunchArg = DeclareLaunchArgument(
        'device_name',
        default_value = 'agv0'
    )

    canbusInterfaceNameLaunchArg = DeclareLaunchArgument(
        'canbus_interface_name',
        default_value = 'can0'
    )

    axisNumberLaunchArg = DeclareLaunchArgument(
        'axis_number',
        default_value = 0
    )

    odriveNode = Node(
        package="odrive",
        executable="odrive",
        namespace=deviceName + "/axis" + str(axisNumber),
        name="odrive",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"axis_number": axisNumber},
            {"calibration_type": 1},
            {"publish_debug_messages": True}
        ],
        remappings=[
            ("odrive/input/can", "/" + deviceName + "/" + canbusInterfaceName + "/output/data"),
            ("odrive/output/can", "/" + deviceName + "/" + canbusInterfaceName + "/input/data"),
            ("odrive/output/status", "output/status"),
            ("odrive/input/velocity", "input/velocity"),
            ("odrive/input/start", "input/start"),
            ("odrive/input/shutdown", "input/shutdown"),
            ("odrive/input/clearErrors", "input/clearErrors"),
            ("odrive/input/reboot", "input/reboot"),
        ]
    )

    return LaunchDescription([
        deviceNameLaunchArg,
        canbusInterfaceNameLaunchArg,
        axisNumberLaunchArg,
        odriveNode
    ])