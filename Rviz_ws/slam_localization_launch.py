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
                ("imu/data_raw", "imu/data_raw"),  # 输入
                ("imu/data", "imu")       # 输出到/imu
            ]
        ),

        Node(
            package='sensor_publisher',
            executable='encoder_node1',
            name='encoder_node1',
            output='screen',
        ),

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

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_dir],
            remappings=[
                ('odometry/filtered', '/odometry/odom_filtered'),
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')  
            ]
        ),
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_map_node',
        #     output='screen',
        #     parameters=[config_dir],
        #     remappings=[ ('odometry/filtered', '/odometry/filtered_map') ]
        # ),
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
                'slam_params_file': config_dir_gmapping,
                # 关键参数 ↓↓↓
                'scan_queue_size': 200,          # 再把队列拉大
                'throttle_scans': 3,              # 只处理每 3 帧（按需再调大 5、10）
                'minimum_time_between_update': 0.1, # 最快 10Hz 更新一次地图（再慢点可 0.2）
                'transform_timeout': 0.5,
                'transform_tolerance': 1.0,
                'delay': 0.2,   
                'use_approximate_sync': True,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'odom_topic': '/odometry/odom_filtered'
            }],
          
        ),
        # Node(
        #     package='slam_toolbox',
        #     executable='map_saver_cli',
        #     name='map_saver',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': False,
        #         'filename': os.path.join(get_package_share_directory('sensor_publisher'), 'maps', 'map')
        #     }],
        #     arguments=['-f', os.path.join(get_package_share_directory('sensor_publisher'), 'maps', 'map')]
        # ),
    ])
