from ament_index_python.packages import get_package_share_path
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments   
    pub_odom_tf_arg = DeclareLaunchArgument('pub_odom_tf', default_value='true',
                                            description='Whether to publish the tf from the original odom to the base_footprint')                                         
     
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false',
                                         description='Whether to launch RViz2')  
                                         
    rviz_config_arg = DeclareLaunchArgument('rviz_config', default_value='bringup',
                                         description='Choose which rviz configuration to use')  
                                                  
    imu_filter_config = os.path.join(              
        get_package_share_directory('ugv_bringup'),
        'param',
        'imu_filter_param.yaml'
    )
    # Include the robot state launch from the ugv_description package
    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ugv_description'), 'launch', 'display.launch.py')
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items()
    ) 
    # Declare launch arguments for serial port configuration
    # Per Waveshare documentation: GPIO UART communication at 115200 baud
    # Jetson Orin uses /dev/ttyTHS1, others use /dev/ttyAMA0
    # Reference: https://github.com/waveshareteam/ugv_base_ros
    # Note: os is already imported at the top of the file
    default_serial_port = '/dev/ttyTHS1' if any("ugv_jetson" in root for root, dirs, files in os.walk("/")) else '/dev/ttyAMA0'
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value=default_serial_port,
        description=f'Serial port for ESP32 GPIO UART communication (default: {default_serial_port})'
    )
    serial_baud_arg = DeclareLaunchArgument(
        'serial_baud', default_value='115200',
        description='Baud rate for ESP32 GPIO UART communication (default: 115200)'
    )

    # Define the nodes to be launched                                     
    bringup_node = Node(
        package='ugv_bringup',
        executable='ugv_bringup',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baud': LaunchConfiguration('serial_baud'),
        }]
    )
    # Define the base node with parameters
    imu_complementary_filter_node = Node(
            package='imu_complementary_filter',
            executable='complementary_filter_node',
            name='complementary_filter_gain_node',
            output='screen',
            parameters=[
                {'do_bias_estimation': True},
                {'do_adaptive_gain': True},
                {'use_mag': False},
                {'gain_acc': 0.01},
                {'gain_mag': 0.01},
            ]
    )
    # Define the base node with parameters
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_filter_config]
    )
    # Include laser lidar launch file
    laser_bringup_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ldlidar'), 'launch'),
         '/ldlidar.launch.py'])
    )
    # Define the base node with parameters    
    driver_node = Node(
        package='ugv_bringup',
        executable='ugv_driver',
    )
    # Define the base node with parameters
    base_node = Node(
        package='ugv_base_node',
        executable='base_node',
        parameters=[{'pub_odom_tf': LaunchConfiguration('pub_odom_tf')}]
    )
    

    return LaunchDescription([
        pub_odom_tf_arg,
        use_rviz_arg,
        rviz_config_arg,
        serial_port_arg,
        serial_baud_arg,
        robot_state_launch,
        bringup_node,
        imu_complementary_filter_node,
        #imu_filter_node,
        laser_bringup_launch,
        driver_node,
        base_node
    ])

