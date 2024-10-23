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

    # Variables
    package_path = os.path.join(get_package_share_directory("vicharaka_rover"))
    xacro_file_path = os.path.join(package_path, "description", "rover.urdf.xacro")

    # Process the xacro file to generate the robot description
    rover_description_config = xacro.process_file(xacro_file_path)

    # Parameters for the node
    params = {'robot_description': rover_description_config.toxml(), 'use_sim_time': use_sim_time}

    # Define the robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    # Return the LaunchDescription
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        node_robot_state_publisher
    ])
