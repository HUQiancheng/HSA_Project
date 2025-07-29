#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class SimpleMapPublisher(Node):
    def __init__(self):
        super().__init__('simple_map_publisher')
        
        self.publisher = self.create_publisher(OccupancyGrid, '/test_map', 10)
        self.timer = self.create_timer(1.0, self.publish_map)
        
        self.get_logger().info('Simple map publisher started')
        
    def publish_map(self):
        msg = OccupancyGrid()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  
        
        msg.info.resolution = 0.1  # 10cm per pixel
        msg.info.width = 10
        msg.info.height = 10
        msg.info.origin.position.x = -1.0
        msg.info.origin.position.y = -1.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        msg.data = [0] * (msg.info.width * msg.info.height)
        
        for i in range(4, 7):
            for j in range(4, 7):
                msg.data[i * msg.info.width + j] = 100
                
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMapPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()