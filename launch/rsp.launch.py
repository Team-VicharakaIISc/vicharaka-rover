import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    #variables 
    package__path=os.path.join(get_package_share_directory("vicharaka_rover"))

    xacro_file_path=os.path.join(package__path,"description","rover.urdf.xacro")

    rover_description_config= xacro.process_file(xacro_file_path)

    return LaunchDescription([
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use sim time if true'),
    ])