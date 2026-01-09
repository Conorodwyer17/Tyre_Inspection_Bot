from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('segmentation_3d')
    
    # Config file path
    config_file = os.path.join(pkg_share, 'config', 'config.yaml')
    
    # Declare launch arguments
    camera_rgb_topic_arg = DeclareLaunchArgument(
        'camera_rgb_topic',
        default_value='/oak/rgb/image_rect',
        description='Camera RGB image topic'
    )
    
    # Ultralytics segmentation node with performance optimizations
    ultralytics_node = Node(
        package='segmentation_3d',
        executable='ultralytics_node',
        name='ultralytics_segmentation',
        output='screen',  # Changed from 'log' to 'screen' to see errors
        parameters=[{
            # Model paths - using lighter yolov8n-seg for navigation (faster)
            'navigation_model': 'yolov8n-seg.pt',  # Lighter model: ~5-10x faster than yolov8m-seg
            'inspection_model': 'best.pt',
            # Performance optimization parameters
            # CRITICAL: Using 640 for navigation mode for better vehicle detection accuracy
            # Trade-off: Slightly slower (~2x) but much better detection rate
            'img_size': 640,  # Increased from 320 for better detection accuracy
            'device': 'cpu',  # Change to 'cuda' if GPU available
            'conf_threshold': 0.15,  # Lowered from 0.25 for better vehicle detection (especially distant/partial vehicles)
            'frame_skip': 1,  # Process every frame (changed from 3) - critical for not missing vehicles
            'half': False,  # FP16 inference (requires CUDA)
        }],
        # Removed problematic prefix that was hiding errors
    )
    
    # Segmentation processor node
    segmentation_processor_node = Node(
        package='segmentation_3d',
        executable='segmentation_processor_node',
        name='segmentation_processor_node',
        output='screen',
        parameters=[config_file]
    )
    
    return LaunchDescription([
        camera_rgb_topic_arg,
        ultralytics_node,
        segmentation_processor_node,
    ])
