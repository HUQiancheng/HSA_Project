from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

config_dir = os.path.join(
    get_package_share_directory('sensor_publisher'),
    'config',
    'ekf.yaml'
)
config_dir_gmapping = os.path.join(
    get_package_share_directory('sensor_publisher'),
    'config',
    'slam.yaml'
)

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='sensor_publisher',
            executable='imu_node',
            name='imu_node',
            output='screen',
        ),
        
        Node(
            package='imu_complementary_filter',
            executable='complementary_filter_node',
            name='complementary_filter_gain_node',
            parameters=[
                {"do_bias_estimation": True},
                {"do_adaptive_gain": True},
                {"use_mag": False},
                {"gain_acc": 0.01},
                {"gain_mag": 0.01}
            ],
            remappings=[
                ("imu/data_raw", "imu/data_raw"),  
                ("imu/data", "imu")      
            ]
        ),

        Node(
            package='sensor_publisher',
            executable='encoder_node1',
            name='encoder_node1',
            output='screen',
        ),

        # IMU tf: base_link -> imu_link    
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_static_tf',
            arguments=['0.03', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link'],
            parameters=[{'use_sim_time': False}]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='mag_static_tf',
            arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'base_link', 'mag_link'],
            parameters=[{'use_sim_time': False}]
        ),

        # robot_localization ekf
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_dir],
            remappings=[
                ('odometry/filtered', '/odometry/odom_filtered')
            ]
        ),

        Node(
            package='sensor_publisher',
            executable='path_publisher',
            name='path_publisher',
            output='screen',
        ),
        Node(
            package='sensor_publisher',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
        ),
        
        # LiDAR tf: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_tf',
            arguments=['-0.03', '0', '0', '0', '0', '0', 'base_link', 'laser'],
            parameters=[{'use_sim_time': False}]
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'map_frame': 'map',
                'scan_topic': '/scan',
                
                'slam_params_file': config_dir_gmapping,
                'throttle_scans': 1,
                'transform_publish_period': 0.02,
                'map_update_interval': 5.0,
                'resolution': 0.05,
                'transform_timeout': 0.2,
                'transform_tolerance': 0.1, 
                
                'use_approximate_sync': False,  
                'minimum_time_interval': 0.1,

                'use_scan_matching': True,
                'minimum_travel_distance': 0.1,
                'minimum_travel_heading': 0.1,
                'scan_buffer_size': 10,
                'do_loop_closing': True,
               
                'odom_topic': '/odometry/odom_filtered'
            }],
          
        ),
    ])