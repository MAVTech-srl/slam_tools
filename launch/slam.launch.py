import os
import datetime

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    livox_wrapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('livox_ros_driver2'), 'launch_ROS2'),
            '/msg_MID360_launch.py'])
        )
    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('fast_lio'), 'launch'),
            '/mapping.launch.py']),
        launch_arguments={'config_file': 'mid360.yaml'}.items(),
        )
    
    # PCD
    pcd_dir_path = os.path.dirname(os.path.realpath(__file__))
    pcd_file_path = os.path.join(pcd_dir_path, "../../../rosbag/") + "pointcloud_mid360_" + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")+ ".pcd" 
    save = Node(
            package='slam_tools',
            executable='pointcloud2pcd',
            name='save_pcd',
            arguments={'sigint_timeout': '30'}.items(),
            parameters=[{'pcd_output_path': pcd_file_path}]
        )
    rosbag = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a'],
            # cmd=['ros2', 'bag', 'record', '/cloud_registered', '/mavros/global_position/local', '/path', '/Odometry'],
            output='screen',
            cwd="rosbag"
        )

    return LaunchDescription([
        livox_wrapper,
        fast_lio,
        save,
        rosbag
    ])
