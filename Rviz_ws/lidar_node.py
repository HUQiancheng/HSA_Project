import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import time

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('frame_id', 'laser')

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f"Connected to {self.port}")
            time.sleep(2)
            self.ser.reset_input_buffer()
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise

        self.timer = self.create_timer(0.02, self.timer_callback)  # 50Hz

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
                        self.publish_laserscan(distance_cm)
                except ValueError:
                    self.get_logger().debug(f"Invalid distance value: {line}")

    def publish_laserscan(self, distance_cm):
        msg = LaserScan()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id

        msg.angle_min = 0.0
        msg.angle_max = 0.0
        msg.angle_increment = 1.0
        msg.time_increment = 0.0
        msg.scan_time = 0.0
        msg.range_min = 0.0
        msg.range_max = 40.0

        distance_in_meters = distance_cm / 100.0
        msg.ranges = [distance_in_meters]
        msg.intensities = [float(distance_cm)]

        self.scan_pub.publish(msg)
        self.get_logger().info(f"Distance: {distance_cm} cm ({distance_in_meters:.2f} m) -> LaserScan published")


def main(args=None):
    rclpy.init(args=args)
    node = LidarPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.ser.close()
    node.destroy_node()
    rclpy.shutdown()
