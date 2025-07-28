import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time
from math import atan2, degrees

class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
        self.sub = self.create_subscription(Odometry, '/wheel_odom', self.odom_callback, 10)
        self.last_print = time.time()

    def odom_callback(self, msg):
        now = time.time()
        if now - self.last_print > 1.0:  # 每秒打印一次
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            # 四元数转航向角 yaw (rad)
            yaw = atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
            yaw_deg = degrees(yaw)
            self.get_logger().info(f"Pose x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.1f}°")
            self.last_print = now

def main(args=None):
    rclpy.init(args=args)
    node = OdomListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
