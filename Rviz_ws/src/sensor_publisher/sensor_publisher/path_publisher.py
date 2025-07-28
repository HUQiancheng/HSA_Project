import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.path = Path()
        # 和你的地图/里程计坐标系一致
        self.path.header.frame_id = 'map'  

        # 订阅 robot_localization 输出的 odometry
        self.sub = self.create_subscription(
            Odometry,
            '/odometry/odom_filtered',
            self.odom_callback,
            10)

        # 发布 nav_msgs/Path
        self.pub = self.create_publisher(Path, '/trajectory', 10)

    def odom_callback(self, msg: Odometry):
        # 把每一帧 pose 加到 Path 里
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.path.poses.append(ps)

        # 更新 header（时间戳）
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
