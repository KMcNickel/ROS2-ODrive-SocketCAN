from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    deviceName = LaunchConfiguration('device_name')
    canbusInterfaceName = LaunchConfiguration('canbus_interface_name')
    axisNumber = LaunchConfiguration('axis_number')
    calibrationType = LaunchConfiguration('calibration_type')

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
        default_value = '0'
    )

    axisNumberLaunchArg = DeclareLaunchArgument(
        'calibration_type',
        default_value = '0',
        description = "0 = None, 1 = Encoder Index Search, 2 = Full"
    )

    odriveNode = Node(
        package="odrive",
        executable="odrive",
        namespace = [deviceName, "/axis", axisNumber],
        name="odrive",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"axis_number": axisNumber},
            {"calibration_type": calibrationType},
            {"publish_debug_messages": True}
        ],
        remappings=[
            ("odrive/input/can", ["/", deviceName, "/", canbusInterfaceName, "/output/data"]),
            ("odrive/output/can", ["/", deviceName, "/", canbusInterfaceName, "/input/data"]),
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