import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def check_numjoints(context, *args, **kwargs):
    numjoints = context.launch_configurations.get('numjoints', '')
    if numjoints == '':
        raise RuntimeError("ERROR: numjoints must be specified as a launch argument.")
    return

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('numjoints', default_value='', description='numjoints specifies the number of rs485 devices which should be included for pressure control.'))
    ld.add_action( DeclareLaunchArgument('ns', default_value='', description='Namespace for pneudrive node, topics, and parameters'))
    ld.add_action(OpaqueFunction(function=check_numjoints))

    ld.add_action(
        Node(
            package='pneudrive',
            executable='pressure_controller_node',
            output='screen',
            namespace=LaunchConfiguration('ns'),
            parameters=[{'numjoints': LaunchConfiguration('numjoints')}]
        )
    )
    return ld
