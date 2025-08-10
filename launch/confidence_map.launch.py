import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the shared config directory of the package
    confidence_mapping_dir = get_package_share_directory('confidence_mapping')
    config_dir = os.path.join(confidence_mapping_dir, 'config')

    # Declare launch configuration variables
    topics_config = LaunchConfiguration('topics_config')
    map_config = LaunchConfiguration('map_config')
    sensor_config = LaunchConfiguration('sensor_config')
    postprocessor_pipeline_config = LaunchConfiguration('postprocessor_pipeline_config')
    postprocessor_filter_config = LaunchConfiguration('postprocessor_filter_config')

    # Declare launch arguments
    declare_topics_config_cmd = DeclareLaunchArgument(
        'topics_config',
        default_value=os.path.join(config_dir, 'robots', 'topics.yaml'),
        description='Path to the robot topics config file.')

    declare_map_config_cmd = DeclareLaunchArgument(
        'map_config',
        default_value=os.path.join(config_dir, 'confidence_maps', 'map_config.yaml'),
        description='Path to the confidence map config file.')

    declare_sensor_config_cmd = DeclareLaunchArgument(
        'sensor_config',
        default_value=os.path.join(config_dir, 'sensor_processors', 'realsense_d435.yaml'),
        description='Path to the sensor processor config file.')

    declare_postprocessor_pipeline_config_cmd = DeclareLaunchArgument(
        'postprocessor_pipeline_config',
        default_value=os.path.join(config_dir, 'postprocessing', 'postprocessor_pipeline.yaml'),
        description='Path to the postprocessor pipeline config file.')

    declare_postprocessor_filter_config_cmd = DeclareLaunchArgument(
        'postprocessor_filter_config',
        default_value=os.path.join(config_dir, 'postprocessing', 'filters.yaml'),
        description='Path to the postprocessor filter config file.')

    # Define the main node
    confidence_mapping_node = Node(
        package='confidence_mapping',
        executable='confidence_mapping',
        name='confidence_mapping',
        output='screen',
        parameters=[
            topics_config,
            map_config,
            sensor_config,
            postprocessor_pipeline_config,
            postprocessor_filter_config
        ]
        # prefix=['gdbserver localhost:3000'],  # uncomment to enable debugging
    )
    confidence_map_filter_node = Node(
        package='confidence_mapping',
        executable='confidence_map_filter',
        name='confidence_map_filter',
        output='screen',
        parameters=[
            postprocessor_filter_config
        ]
        # prefix=['gdbserver localhost:3000'],  # uncomment to enable debugging
    )

    # Build the launch description
    ld = LaunchDescription()

    # Add all launch arguments
    ld.add_action(declare_topics_config_cmd)
    ld.add_action(declare_map_config_cmd)
    ld.add_action(declare_sensor_config_cmd)
    ld.add_action(declare_postprocessor_pipeline_config_cmd)
    ld.add_action(declare_postprocessor_filter_config_cmd)

    # Add the main node
    ld.add_action(confidence_mapping_node)
    ld.add_action(confidence_map_filter_node)

    return ld
