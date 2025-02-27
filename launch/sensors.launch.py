import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(get_package_share_directory('mavros'), 'launch', 'px4.launch'))
    )

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('vicharaka-rover'), 'launch', 'depth_camera.launch.py'))
    )

    return LaunchDescription([
        mavros_launch,
        depth_camera_launch
    ])
