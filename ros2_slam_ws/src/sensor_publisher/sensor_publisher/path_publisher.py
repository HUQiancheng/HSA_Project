import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.path = Path()
        self.path.header.frame_id = 'map'  

        self.sub = self.create_subscription(
            Odometry,
            '/odometry/odom_filtered',
            self.odom_callback,
            10)

        self.pub = self.create_publisher(Path, '/trajectory', 10)

    def odom_callback(self, msg: Odometry):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.path.poses.append(ps)

        self.path.header.stamp = msg.header.stamp
        self.pub.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
