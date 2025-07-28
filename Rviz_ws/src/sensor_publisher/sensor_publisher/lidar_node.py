# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# import serial
# import time

# class LidarPublisher(Node):
#     def __init__(self):
#         super().__init__('lidar_publisher_node')

#         self.declare_parameter('port', '/dev/ttyACM0')
#         self.declare_parameter('baudrate', 9600)
#         self.declare_parameter('frame_id', 'laser')

#         self.port = self.get_parameter('port').get_parameter_value().string_value
#         self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
#         self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

#         self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

#         try:
#             self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
#             self.get_logger().info(f"Connected to {self.port}")
#             time.sleep(2)
#             self.ser.reset_input_buffer()
#         except Exception as e:
#             self.get_logger().error(f"Failed to open serial port: {e}")
#             raise

#         self.timer = self.create_timer(0.02, self.timer_callback)  # 50Hz

#     def timer_callback(self):
#         if self.ser.in_waiting > 0:
#             line_bytes = self.ser.readline()
#             try:
#                 line = line_bytes.decode('utf-8', errors='ignore').strip()
#             except:
#                 return
#             if line.startswith("L:") and len(line) > 2:
#                 try:
#                     distance_cm = int(line[2:]) - 10
#                     if 1 <= distance_cm <= 4000:
#                         self.publish_laserscan(distance_cm)
#                 except ValueError:
#                     self.get_logger().debug(f"Invalid distance value: {line}")

#     def publish_laserscan(self, distance_cm):
#         msg = LaserScan()
#         now = self.get_clock().now().to_msg()
#         msg.header.stamp = now
#         msg.header.frame_id = self.frame_id

#         msg.angle_min = 0.0
#         msg.angle_max = 0.0
#         msg.angle_increment = 1.0
#         msg.time_increment = 0.0
#         msg.scan_time = 0.0
#         msg.range_min = 0.0
#         msg.range_max = 40.0

#         distance_in_meters = distance_cm / 100.0
#         msg.ranges = [distance_in_meters]
#         msg.intensities = [float(distance_cm)]

#         self.scan_pub.publish(msg)
#         self.get_logger().info(f"Distance: {distance_cm} cm ({distance_in_meters:.2f} m) -> LaserScan published")


# def main(args=None):
#     rclpy.init(args=args)
#     node = LidarPublisher()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.ser.close()
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
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

        # 串口连接
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f"Connected to {self.port}")
            time.sleep(2)
            self.ser.reset_input_buffer()
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise

        # 发布器和订阅器
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/odom_filtered', self.odom_callback, 10)

        # SLAM参数
        self.num_angles = 72  # 5度间隔，72个角度点
        self.scan_ranges = [float('inf')] * self.num_angles
        self.scan_intensities = [0.0] * self.num_angles
        
        # 机器人状态
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.last_yaw = 0.0
        
        # 距离数据缓存
        self.current_distance = 0.0
        self.distance_history = []
        self.max_history = 10
        
        # 定时器
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50Hz
        self.scan_timer = self.create_timer(0.1, self.publish_scan)  # 10Hz发布扫描
        
        self.get_logger().info('Rotating SLAM Lidar started')

    def odom_callback(self, msg):
        """获取机器人位姿"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # 从四元数转换为欧拉角
        quat = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        )

    def timer_callback(self):
        """读取距离传感器数据"""
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
                        
                        # 根据机器人朝向更新对应角度的距离
                        self.update_distance_at_angle(self.robot_yaw, distance_meters)
                        
                except ValueError:
                    self.get_logger().debug(f"Invalid distance value: {line}")

    def update_distance_at_angle(self, angle, distance):
        """根据机器人朝向更新对应角度的距离数据"""
        # 将角度标准化到[0, 2*pi]
        normalized_angle = angle % (2 * math.pi)
        
        # 计算对应的索引
        angle_index = int(normalized_angle / (2 * math.pi) * self.num_angles)
        angle_index = min(angle_index, self.num_angles - 1)
        
        # 更新距离数据
        self.scan_ranges[angle_index] = distance
        self.scan_intensities[angle_index] = 100.0
        
        # 平滑相邻角度的数据
        self.smooth_neighboring_angles(angle_index, distance)

    def smooth_neighboring_angles(self, center_index, distance):
        """平滑相邻角度的数据，增加数据密度"""
        # 在中心角度周围±2个角度内插值
        for offset in [-2, -1, 1, 2]:
            neighbor_index = (center_index + offset) % self.num_angles
            
            # 如果相邻角度没有数据，用当前距离+噪声填充
            if self.scan_ranges[neighbor_index] == float('inf'):
                noise = np.random.normal(0, 0.05)  # 5cm标准差的噪声
                self.scan_ranges[neighbor_index] = max(0.01, distance + noise)
                self.scan_intensities[neighbor_index] = 80.0

    def publish_scan(self):
        """发布激光扫描数据"""
        msg = LaserScan()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id

        # 设置扫描参数
        msg.angle_min = -math.pi  # -180度
        msg.angle_max = math.pi   # +180度
        msg.angle_increment = 2 * math.pi / self.num_angles
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.01
        msg.range_max = 40.0

        # 当前方向的距离数据
        current_angle_index = int((self.robot_yaw % (2 * math.pi)) / (2 * math.pi) * self.num_angles)
        
        # 构建扫描数据
        ranges = []
        intensities = []
        
        for i in range(self.num_angles):
            if i == current_angle_index:
                # 当前方向使用最新距离
                ranges.append(self.current_distance if self.current_distance > 0 else float('inf'))
                intensities.append(100.0)
            elif self.scan_ranges[i] != float('inf'):
                # 有历史数据的角度
                ranges.append(self.scan_ranges[i])
                intensities.append(self.scan_intensities[i])
            else:
                # 没有数据的角度
                ranges.append(float('inf'))
                intensities.append(0.0)

        msg.ranges = ranges
        msg.intensities = intensities

        self.scan_pub.publish(msg)
        
        # 计算有效数据点数量
        valid_points = sum(1 for r in ranges if r != float('inf'))
        
        if valid_points > 5:  # 有足够数据点时才打印
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