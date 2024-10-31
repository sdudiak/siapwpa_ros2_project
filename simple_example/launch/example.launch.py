import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    simple_example_path = get_package_share_directory('simple_example')
    config_path = os.path.join(simple_example_path, 'config','gz_bridge.yaml')


    # Declare world file argument
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(simple_example_path, 'worlds', 'sonoma.sdf'),
        description='Path to the Gazebo world file'
    )

    # Include Gazebo simulation launch with world file argument
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': 'src/simple_example/worlds/sonoma.sdf'}.items(),
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        # arguments=['-d', os.path.join(pkg_ros_gz_sim_demos, 'rviz', 'camera.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/front_camera@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/back_camera@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        output='screen'
    )

    return LaunchDescription([
        world_file_arg,
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        gz_sim,
        bridge,
        # rviz
    ])