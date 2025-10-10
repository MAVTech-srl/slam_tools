import os
import datetime

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription 
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_ros.actions
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Launch arguments
    convert_livox_cloud = LaunchConfiguration('convert_livox_cloud')
    declare_convert_cmd = DeclareLaunchArgument(
        'convert_livox_cloud', default_value='false',
        description='Convert livox custom point cloud to ROS PointCloud2 message if true. The converted cloud is published in /livox/points.'
    )
    save_pcd_cloud = LaunchConfiguration('save_pcd_cloud')
    declare_save_pcd_cmd = DeclareLaunchArgument(
        'save_pcd_cloud', default_value='true',
        description='Save local point cloud in .pcd format if true.'
    )
    save_UTM_pcd_cloud = LaunchConfiguration('save_UTM_pcd_cloud')
    declare_save_UTM_pcd_cmd = DeclareLaunchArgument(
        'save_UTM_pcd_cloud', default_value='true',
        description='Save point cloud with UTM (zone 32) coordinates in .pcd format if true.'
    )


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

    # Convert livox point cloud custom msg (/livox/lidar) to PointCloud2 (/livox/points)
    convert = Node(
            package='slam_tools',
            executable='livox2pointcloud',
            name='convert_pointcloud',
            arguments={'sigint_timeout': '30'}.items(),
            condition=IfCondition(convert_livox_cloud)
    )

    # UTM PCD
    pcd_dir_path = os.path.dirname(os.path.realpath(__file__))
    pcd_file_path = os.path.join(pcd_dir_path, "../../../rosbag/") + "pointcloud_UTM_avia_" + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")+ ".pcd" 
    save_UTM = Node(
            package='slam_tools',
            executable='pointcloud2UTMpcd',
            name='save_pcd',
            arguments={'sigint_timeout': '30'}.items(),
            parameters=[{'pcd_output_path': pcd_file_path}],
            condition=IfCondition(save_UTM_pcd_cloud)
    )

    # local PCD
    local_pcd_dir_path = os.path.dirname(os.path.realpath(__file__))
    local_pcd_file_path = os.path.join(local_pcd_dir_path, "../../../rosbag/") + "pointcloud_avia_" + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")+ ".pcd" 
    save_local = Node(
            package='slam_tools',
            executable='pointcloud2pcd',
            name='save_pcd',
            arguments={'sigint_timeout': '30'}.items(),
            parameters=[{'pcd_output_path': local_pcd_file_path}],
            condition=IfCondition(save_pcd_cloud)
        )

    rosbag = ExecuteProcess(
            # cmd=['ros2', 'bag', 'record', '-a'],
            cmd=['ros2', 'bag', 'record', '/cloud_registered', '/mavros/local_position/odom', '/path', '/Odometry', '/livox/points', '/mavros/global_position/raw/fix', '/mavros/odometry/in'],
            output='screen',
            cwd="rosbag"
        )
    

    return LaunchDescription([
        declare_convert_cmd,
        declare_save_pcd_cmd,
        declare_save_UTM_pcd_cmd,
        livox_wrapper,
        fast_lio,
        convert,
        save_UTM,
        save_local,
        rosbag
    ])
