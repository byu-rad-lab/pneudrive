from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('numjoints', default_value='1', description='numJoints specifies the number of rs485 devices which should be included for pressure control.'))
    ld.add_action( DeclareLaunchArgument('ns', default_value='', description='Namespace for pneudrive node, topics, and parameters'))

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
