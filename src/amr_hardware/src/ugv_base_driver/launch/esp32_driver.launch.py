from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'uart_port',
            default_value='/dev/ttyUSB0',
            description='UART port for ESP32 communication'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate for UART communication'
        ),
        DeclareLaunchArgument(
            'publish_odom',
            default_value='false',
            description='Publish odometry from ESP32 (if available)'
        ),
        DeclareLaunchArgument(
            'publish_joint_states',
            default_value='false',
            description='Publish joint states from ESP32 (if available)'
        ),
        
        Node(
            package='ugv_base_driver',
            executable='motor_driver_node',
            name='motor_driver_node',
            parameters=[{
                'uart_port': LaunchConfiguration('uart_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'publish_odom': LaunchConfiguration('publish_odom'),
                'publish_joint_states': LaunchConfiguration('publish_joint_states'),
            }],
            output='screen',
        ),
    ])
