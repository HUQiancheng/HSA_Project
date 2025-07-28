from launch import LaunchDescription
from launch_ros.actions import Node

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
                {"use_mag": True},
                {"gain_acc": 0.01},
                {"gain_mag": 0.01}
            ],
            remappings=[
                ("imu/data", "imu")  
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_static_tf',
            arguments=['0.095', '-0.05', '0.0', '0', '0', '0', 'base_link', 'imu_link']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='mag_static_tf',
            arguments=['0.0', '0', '0.0', '0', '0', '0', 'base_link', 'mag_link']
        ),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', '/home/ubuntu/ros2_ws/src/sensor_publisher/rviz/imu_config.rviz'],
        #     output='screen'
        # )
    ])
