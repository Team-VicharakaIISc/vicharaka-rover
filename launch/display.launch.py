
from launch.substitutions import Command, LaunchConfiguration
import launch_ros,launch
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'vicharaka-rover'
    pkg_share = launch_ros.substitutions.FindPackageShare(package='vicharaka-rover').find('vicharaka-rover')
    default_model_path = os.path.join(pkg_share, 'src/description/vicharaka-rover.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    parameters=[{
          'frame_id':'base_footprint',
          'odom_frame_id':'odom',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'wait_imu_to_init':True,
          'approx_sync':False}]
    

    remappings1=[
          ('/odom', '/visual_odom'),
          ('/imu', 'sensor/imu/data'),
          ('rgb/image', '/camera/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/camera/color/camera_info'),
          ('depth/image', '/camera/camera/depth/image_rect_raw')]    
    
    remappings2=[
          ('/imu', '/imu/data'),
          ('rgb/image', '/camera/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/camera/color/camera_info'),
          ('depth/image', '/camera/camera/depth/image_rect_raw')]   
    remappings3=[
          ('/odom', '/odmetry/filtered'),
          ('/imu', 'sensor/imu/data'),
          ('rgb/image', '/camera/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/camera/color/camera_info'),
          ('depth/image', '/camera/camera/depth/image_rect_raw')]   

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(pkg_name),'launch','joystick.launch.py'
                )])
    )
    
    twist_mux_params = os.path.join(get_package_share_directory(pkg_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/cmd_vel')]
        )

    spawn_entity = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=['-entity', 'vicharaka-rover', '-topic', 'robot_description'],
    output='screen'
    )
    
    odometry_publisher = Node(
    package='odometry_publisher',
    executable='odometry_publisher',
    )
    
    robot_localization_node = Node(
        package='robot_localization',
        executable='ukf_node',
        name='ukf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml')]
    )

    imu_fusion_node = Node(
        package='imu_fusion_pkg',
        executable='imu_fusion',
        name='imu_node'
    )

    depth_camera_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(pkg_name),'launch','depth_camera.launch.py'
                )])
    )

    sensor_filter_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(pkg_name),'launch','sensor_filter.launch.py'
                )])
    )

    navigation_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(pkg_name),'launch','navigation.launch.py'
                )])
    )

    rtabmap_odom = Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings1,
            arguments=['--ros-args', '--log-level', 'error'])

    rtabmap_slam = Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings1,
            arguments=['--ros-args', '--log-level', 'error'])

    rtabmap_viz = Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings1,
            arguments=['--ros-args', '--log-level', 'error'])


    return launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        # launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        robot_state_publisher_node,
        # joystick,
        # twist_mux,
        # spawn_entity,
        # rviz_node,
        joint_state_publisher_node,
        imu_fusion_node,
        depth_camera_launch,
        sensor_filter_launch,
        rtabmap_odom,
        # robot_localization_node
        rtabmap_slam,
        # rtabmap_viz
        # odometry_publisher
    ])