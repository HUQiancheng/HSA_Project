import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import busio
import adafruit_bno08x
import adafruit_bno08x.i2c as bno08x_i2c
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_ROTATION_VECTOR
)


class BNO085Node(Node):
    def __init__(self):
        super().__init__('bno085_imu_node')

        i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = bno08x_i2c.BNO08X_I2C(i2c, address=0x4A)
        time.sleep(1.0)

        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        time.sleep(0.5)

        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        time.sleep(0.5)

        self.bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
        time.sleep(0.5)

        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.05, self.publish_imu_data)  # 20 Hz

    def publish_imu_data(self):
        accel = self.bno.acceleration
        gyro = self.bno.gyro

        if accel is None or gyro is None:
            self.get_logger().warn("IMU read failed")
            return

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]

        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]

        imu_msg.orientation_covariance[0] = -1.0

        self.publisher_.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BNO085Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
