import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf_transformations
import tf2_ros
import math

class OdometryFusionNode(Node):
    def __init__(self):
        super().__init__('odometry_fusion_node')

        self.pulses_per_rev = 20 
        self.wheel_radius = 0.03 
        self.wheel_base = 0.14    

        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  

        self.subscription_imu = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.subscription_left = self.create_subscription(Int32, '/wheel_left_ticks', self.left_ticks_callback, 10)
        self.subscription_right = self.create_subscription(Int32, '/wheel_right_ticks', self.right_ticks_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.last_time = self.get_clock().now()

    def left_ticks_callback(self, msg):
        self.left_ticks = msg.data

    def right_ticks_callback(self, msg):
        self.right_ticks = msg.data

    def imu_callback(self, msg: Imu):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        delta_left = self.left_ticks - self.prev_left_ticks
        delta_right = self.right_ticks - self.prev_right_ticks

        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks

        dist_per_pulse = 2 * math.pi * self.wheel_radius / self.pulses_per_rev
        d_left = delta_left * dist_per_pulse
        d_right = delta_right * dist_per_pulse

        d_center = (d_left + d_right) / 2.0
        delta_theta = msg.angular_velocity.z * dt 

        self.theta += delta_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

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

        odom_msg.twist.twist.linear.x = d_center / dt if dt > 0 else 0.0
        odom_msg.twist.twist.angular.z = delta_theta / dt if dt > 0 else 0.0

        self.odom_pub.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
