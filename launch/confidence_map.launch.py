import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    share_dir = get_package_share_directory('confidence_mapping')
    config_dir = os.path.join(share_dir, 'config')
    xacro_path = os.path.join(config_dir, 'robot.urdf.xacro')
    list_params = []
    for filee in ["robots/topics.yaml","confidence_maps/map_config.yaml","sensor_processors/realsense_d435.yaml","postprocessing/postprocessor_pipeline.yaml"]:
        list_params.append(os.path.join(config_dir, filee))
        
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package='confidence_mapping',
                executable='confidence_mapping',
                name='confidence_mapping',
                # prefix=['gdbserver localhost:3000'], # uncoment if running GDB debugger
                output='screen',
                parameters=list_params,
            )
        ]
    )
