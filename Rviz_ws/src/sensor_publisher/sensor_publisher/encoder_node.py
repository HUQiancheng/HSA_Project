#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from math import sin, cos, pi
import RPi.GPIO as GPIO
import threading
import time

class Encoder:
    def __init__(self, gpio_pin):
        self.pin = gpio_pin
        self.count = 0
        self.lock = threading.Lock()

        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.pin, GPIO.FALLING, callback=self._pulse, bouncetime=5)

    def _pulse(self, channel):
        with self.lock:
            self.count += 1

    def get_count(self):
        with self.lock:
            return self.count

class EncoderOdomNode(Node):
    def __init__(self):
        super().__init__('encoder_odometry_node')

        # 参数声明
        self.declare_parameter('wheel_diameter', 0.065)      # 米
        self.declare_parameter('wheel_track', 0.15)          # 米
        self.declare_parameter('encoder_resolution', 20)     # 脉冲数/圈
        self.declare_parameter('gear_reduction', 1.0)        # 减速比

        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_track = self.get_parameter('wheel_track').value
        self.encoder_resolution = self.get_parameter('encoder_resolution').value
        self.gear_reduction = self.get_parameter('gear_reduction').value

        # 计算每米脉冲数
        self.ticks_per_meter = self.encoder_resolution * self.gear_reduction / (self.wheel_diameter * pi)

        GPIO.setmode(GPIO.BCM)
        self.encoder_left = Encoder(17)
        self.encoder_right = Encoder(27)

        self.odom_pub = self.create_publisher(Odometry, 'wheel_odom', 10)

        # 里程计状态
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_left = 0
        self.last_right = 0
        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.update)

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        left = self.encoder_left.get_count()
        right = self.encoder_right.get_count()

        delta_left = left - self.last_left
        delta_right = right - self.last_right

        self.last_left = left
        self.last_right = right
        self.last_time = now

        dleft = delta_left / self.ticks_per_meter
        dright = delta_right / self.ticks_per_meter

        dxy_ave = (dleft + dright) / 2.0
        dth = (dright - dleft) / self.wheel_track

        if dxy_ave != 0.0:
            dx = cos(dth) * dxy_ave
            dy = sin(dth) * dxy_ave
            self.x += cos(self.th) * dx - sin(self.th) * dy
            self.y += sin(self.th) * dx + cos(self.th) * dy

        if dth != 0.0:
            self.th += dth

        quat = Quaternion()
        quat.z = sin(self.th / 2.0)
        quat.w = cos(self.th / 2.0)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'wheel_odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x = dxy_ave / dt
        odom.twist.twist.angular.z = dth / dt

        self.odom_pub.publish(odom)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
