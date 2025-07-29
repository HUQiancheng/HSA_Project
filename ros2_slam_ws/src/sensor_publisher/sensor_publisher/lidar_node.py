import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import serial
import time
import math
import numpy as np
from tf2_ros import Buffer, TransformListener
import tf2_py

class RotatingSlamLidar(Node):
    def __init__(self):
        super().__init__('rotating_slam_lidar_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('frame_id', 'laser')

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f"Connected to {self.port}")
            time.sleep(2)
            self.ser.reset_input_buffer()
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/odom_filtered', self.odom_callback, 10)

        self.num_angles = 72 
        self.scan_ranges = [float('inf')] * self.num_angles
        self.scan_intensities = [0.0] * self.num_angles

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.last_yaw = 0.0
        
        self.current_distance = 0.0
        self.distance_history = []
        self.max_history = 10

        self.timer = self.create_timer(0.02, self.timer_callback)  # 50Hz
        self.scan_timer = self.create_timer(0.1, self.publish_scan)  # 10Hz
        
        self.get_logger().info('Rotating SLAM Lidar started')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        quat = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        )

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            line_bytes = self.ser.readline()
            try:
                line = line_bytes.decode('utf-8', errors='ignore').strip()
            except:
                return
            if line.startswith("L:") and len(line) > 2:
                try:
                    distance_cm = int(line[2:]) - 10
                    if 1 <= distance_cm <= 4000:
                        distance_meters = distance_cm / 100.0
                        self.current_distance = distance_meters
                        
                        self.update_distance_at_angle(self.robot_yaw, distance_meters)
                        
                except ValueError:
                    self.get_logger().debug(f"Invalid distance value: {line}")

    def update_distance_at_angle(self, angle, distance):
        normalized_angle = angle % (2 * math.pi)
        

        angle_index = int(normalized_angle / (2 * math.pi) * self.num_angles)
        angle_index = min(angle_index, self.num_angles - 1)
        
        self.scan_ranges[angle_index] = distance
        self.scan_intensities[angle_index] = 100.0

        self.smooth_neighboring_angles(angle_index, distance)

    def smooth_neighboring_angles(self, center_index, distance):
        for offset in [-2, -1, 1, 2]:
            neighbor_index = (center_index + offset) % self.num_angles

            if self.scan_ranges[neighbor_index] == float('inf'):
                noise = np.random.normal(0, 0.05) 
                self.scan_ranges[neighbor_index] = max(0.01, distance + noise)
                self.scan_intensities[neighbor_index] = 80.0

    def publish_scan(self):
        msg = LaserScan()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id

        msg.angle_min = -math.pi 
        msg.angle_max = math.pi   
        msg.angle_increment = 2 * math.pi / self.num_angles
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.01
        msg.range_max = 40.0

        current_angle_index = int((self.robot_yaw % (2 * math.pi)) / (2 * math.pi) * self.num_angles)

        ranges = []
        intensities = []
        
        for i in range(self.num_angles):
            if i == current_angle_index:
                ranges.append(self.current_distance if self.current_distance > 0 else float('inf'))
                intensities.append(100.0)
            elif self.scan_ranges[i] != float('inf'):
                ranges.append(self.scan_ranges[i])
                intensities.append(self.scan_intensities[i])
            else:
                ranges.append(float('inf'))
                intensities.append(0.0)

        msg.ranges = ranges
        msg.intensities = intensities

        self.scan_pub.publish(msg)
        
        valid_points = sum(1 for r in ranges if r != float('inf'))
        
        if valid_points > 5: 
            self.get_logger().info(f"Published scan: {valid_points}/{self.num_angles} valid points, "
                                 f"robot yaw: {math.degrees(self.robot_yaw):.1f}°, "
                                 f"front distance: {self.current_distance:.2f}m")

def main(args=None):
    rclpy.init(args=args)
    node = RotatingSlamLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.ser.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()