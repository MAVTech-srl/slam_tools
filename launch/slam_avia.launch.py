import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription 
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_ros.actions

def generate_launch_description():
    livox_wrapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('livox_ros2_driver'), 'launch'),
            # '/livox_lidar_launch.py']) # Use ROS Pointcloud2 because custom livox message always gives invalid meessage error...
            '/livox_lidar_msg_launch.py'])
        )
    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('fast_lio'), 'launch'),
            '/mapping.launch.py']),
        launch_arguments={'config_file': 'avia.yaml'}.items()
        )
    # save = Node(
    #         package='slam_tools',
    #         executable='pointcloud2pcd',
    #         name='save_pcl',
    #         arguments={'sigint_timeout': '30'}.items()
    #     )
    save = Node(
            package='slam_tools',
            executable='pointcloud2las',
            name='save_las',
            arguments={'sigint_timeout': '30'}.items()
        )
    
    # mavros = Node(
    #         package='mavros',
    #         executable='mavros_node',
    #         name='mavros',
    #         ros_arguments={'params-file': 'px4_config.yaml'}.items()
    #     ),

    # rosbag = Node(
    #         package='rosbag2',
    #         executable='record',
    #         name='rosbag2_record',
    #         output='screen',
    #         arguments= {'-o': 'rosbag/bag_"$(date + "%Y_%m_%d-%H_%M_%S")"',
    #                     '/cloud_registered', '/mavros/local_position/odom', '/path', '/Odometry'}
    #     )

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
