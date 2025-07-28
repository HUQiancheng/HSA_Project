import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf_transformations
import math
from rclpy.qos import QoSProfile

class EncoderToOdomNode(Node):
    def __init__(self):
        super().__init__('encoder_to_odom_node')

        self.pulses_per_rev = 20         # encoder pulse per round
        self.wheel_radius = 0.03         # radius(m)
        self.wheel_base = 0.14           # distance between wheels

        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = self.get_clock().now()

        qos = QoSProfile(depth=10)
        self.sub_left = self.create_subscription(Int32, '/wheel_left_ticks', self.left_ticks_callback, qos)
        self.sub_right = self.create_subscription(Int32, '/wheel_right_ticks', self.right_ticks_callback, qos)

        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', qos)

    def left_ticks_callback(self, msg):
        self.left_ticks = msg.data

    def right_ticks_callback(self, msg):
        self.right_ticks = msg.data

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        if dt == 0:
            return
        self.last_time = current_time

        delta_left = self.left_ticks - self.prev_left_ticks
        delta_right = self.right_ticks - self.prev_right_ticks

        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks

        dist_per_pulse = 2 * math.pi * self.wheel_radius / self.pulses_per_rev
        d_left = delta_left * dist_per_pulse
        d_right = delta_right * dist_per_pulse

        d_center = (d_left + d_right) / 2.0
        delta_theta = (d_right - d_left) / self.wheel_base

        dx = d_center * math.cos(self.theta + delta_theta / 2.0)
        dy = d_center * math.sin(self.theta + delta_theta / 2.0)

        self.x += dx
        self.y += dy
        self.theta += delta_theta

        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        odom_msg.twist.twist.linear.x = d_center / dt
        odom_msg.twist.twist.angular.z = delta_theta / dt

        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderToOdomNode()

    node.create_timer(0.05, node.timer_callback)  # 20 Hz

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
