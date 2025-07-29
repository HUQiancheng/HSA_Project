from setuptools import find_packages, setup

package_name = 'sensor_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/imu_launch.py']),
        ('share/' + package_name + '/launch', ['launch/slam_localization_launch.py']),
        ('share/' + package_name + '/config', ['config/ekf.yaml', 'config/gmapping_params.yaml']),
        ('share/' + package_name + '/rviz', ['rviz/slam.rviz']),
    ],
    install_requires=['setuptools','adafruit-blinka',
        'adafruit-circuitpython-bno08x',],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='liangyu.chen3208@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_node = sensor_publisher.imu_node:main',
            'encoder_node1 = sensor_publisher.encoder_node1:main',
            'lidar_node = sensor_publisher.lidar_node:main',
            'path_publisher = sensor_publisher.path_publisher:main',
        ],
    },
)
