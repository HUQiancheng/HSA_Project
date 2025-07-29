#!/home/ubuntu/env/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion
from builtin_interfaces.msg import Time
import board
import busio
import adafruit_bno08x
import adafruit_bno08x.i2c as bno08x_i2c
import math

class BNO085Publisher(Node):
    def __init__(self):
        super().__init__('bno085_imu_node')

        i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = bno08x_i2c.BNO08X_I2C(i2c, address=0x4A)

        self.bno.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
        self.bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)

        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 20)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 20)

        self.timer = self.create_timer(0.05, self.publish_data)  # 20Hz

    def publish_data(self):
        accel = self.bno.acceleration
        gyro = self.bno.gyro
        mag = self.bno.magnetic
        quat = self.bno.quaternion  # (i, j, k, real)

        now = self.get_clock().now().to_msg()

        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = 'imu_link'

        if quat:
            imu_msg.orientation.x = quat[0]
            imu_msg.orientation.y = quat[1]
            imu_msg.orientation.z = quat[2]
            imu_msg.orientation.w = quat[3]
        else:
            imu_msg.orientation_covariance[0] = -1.0

        if gyro:
            imu_msg.angular_velocity.x = gyro[0]
            imu_msg.angular_velocity.y = gyro[1]
            imu_msg.angular_velocity.z = gyro[2]

        if accel:
            imu_msg.linear_acceleration.x = accel[0]
            imu_msg.linear_acceleration.y = accel[1]
            imu_msg.linear_acceleration.z = accel[2]

        imu_msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        imu_msg.angular_velocity_covariance = [0.001]*9
        imu_msg.linear_acceleration_covariance = [0.01]*9

        self.imu_pub.publish(imu_msg)

        if mag:
            mag_msg = MagneticField()
            mag_msg.header.stamp = now
            mag_msg.header.frame_id = 'mag_link'
            mag_msg.magnetic_field.x = mag[0]
            mag_msg.magnetic_field.y = mag[1]
            mag_msg.magnetic_field.z = mag[2]
            self.mag_pub.publish(mag_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BNO085Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
