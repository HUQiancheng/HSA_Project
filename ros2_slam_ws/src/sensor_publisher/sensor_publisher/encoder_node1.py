#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from math import sin, cos, pi
import RPi.GPIO as GPIO
import threading
import tf2_ros
import time


ODOM_POSE_COVARIANCE = [
    1.0e-3, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 1.0e-3, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0e6, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0e6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0e6, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0e3
]

ODOM_POSE_COVARIANCE2 = [
    1.0e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 1.0e-3, 1.0e-9, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0e6, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0e6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0e6, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0e-9
]

ODOM_TWIST_COVARIANCE = [
    1.0e-3, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 1.0e-3, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0e6, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0e6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0e6, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0e3
]

ODOM_TWIST_COVARIANCE2 = [
    1.0e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 1.0e-3, 1.0e-9, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0e6, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0e6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0e6, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0e-9
]


def normalize_angle(angle):
    while angle > pi:
        angle -= 2.0 * pi
    while angle < -pi:
        angle += 2.0 * pi
    return angle

class Encoder:
    def __init__(self, gpio_pin, edge='falling', debounce=2):
        self.pin = gpio_pin
        self.count = 0
        self.lock = threading.Lock()

        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        if edge.lower() == 'both':
            edge_type = GPIO.BOTH
        elif edge.lower() == 'rising':
            edge_type = GPIO.RISING
        else:
            edge_type = GPIO.FALLING

        GPIO.add_event_detect(self.pin, edge_type, callback=self._pulse, bouncetime=debounce)

    def _pulse(self, channel):
        with self.lock:
            self.count += 1

    def get_and_reset_increment(self, last_count):
        with self.lock:
            inc = self.count - last_count
        return inc

    def get_count(self):
        with self.lock:
            return self.count

class EncoderOdomNode(Node):
    def __init__(self):
        super().__init__('encoder_odometry_node')

        self.declare_parameter('left_pin', 17)
        self.declare_parameter('right_pin', 27)
        self.declare_parameter('wheel_diameter', 0.07)     
        self.declare_parameter('wheel_track', 0.135)        
        self.declare_parameter('encoder_resolution', 40)     
        self.declare_parameter('gear_reduction', 1.0)
        self.declare_parameter('edges_per_pulse', 1.0)      
        self.declare_parameter('publish_rate', 50.0)         
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('odom_frame', 'odom')         
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('motors_reversed', True)    
        self.declare_parameter('debug', False)

        left_pin  = self.get_parameter('left_pin').value
        right_pin = self.get_parameter('right_pin').value
        self.wheel_diameter     = self.get_parameter('wheel_diameter').value
        self.wheel_track        = self.get_parameter('wheel_track').value
        self.encoder_resolution = self.get_parameter('encoder_resolution').value
        self.gear_reduction     = self.get_parameter('gear_reduction').value
        self.edges_per_pulse    = self.get_parameter('edges_per_pulse').value
        self.publish_rate       = self.get_parameter('publish_rate').value
        self.publish_tf         = self.get_parameter('publish_tf').value
        self.odom_frame         = self.get_parameter('odom_frame').value
        self.base_link_frame    = self.get_parameter('base_link_frame').value
        self.motors_reversed    = self.get_parameter('motors_reversed').value
        self.debug              = self.get_parameter('debug').value

        GPIO.setmode(GPIO.BCM)
        self.encoder_left  = Encoder(left_pin,  edge='falling', debounce=2)
        self.encoder_right = Encoder(right_pin, edge='falling', debounce=2)

        # ticks_per_rev = encoder_resolution * gear_reduction * edges_per_pulse
        self.ticks_per_rev   = self.encoder_resolution * self.gear_reduction * self.edges_per_pulse
        self.ticks_per_meter = self.ticks_per_rev / (self.wheel_diameter * pi)

        self.odom_pub = self.create_publisher(Odometry, 'wheel_odom', 10)
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_left  = 0
        self.last_right = 0
        self.last_time  = self.get_clock().now()

        self.timer = self.create_timer(1.0 / self.publish_rate, self.update)

        self.get_logger().info(
            f'encoder_odometry_node started.\n'
            f'  pins: L={left_pin}, R={right_pin}\n'
            f'  ticks_per_rev={self.ticks_per_rev}, ticks_per_meter={self.ticks_per_meter:.2f}\n'
            f'  publish_rate={self.publish_rate} Hz, publish_tf={self.publish_tf}'
        )

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        left_total  = self.encoder_left.get_count()
        right_total = self.encoder_right.get_count()

        delta_left  = left_total  - self.last_left
        delta_right = right_total - self.last_right

        self.last_left  = left_total
        self.last_right = right_total
        self.last_time  = now

        if self.motors_reversed:
            delta_left  = -delta_left
            delta_right = -delta_right

        dleft  = delta_left  / self.ticks_per_meter
        dright = delta_right / self.ticks_per_meter

        dxy = (dleft + dright) / 2.0
        dth = (dright - dleft) / self.wheel_track

        if abs(dxy) > 0.0:
            dx = dxy * cos(self.th)
            dy = dxy * sin(self.th)
            self.x += dx
            self.y += dy

        if abs(dth) > 0.0:
            self.th += dth
            self.th = normalize_angle(self.th)


        vx  = dxy / dt
        vth = dth / dt

        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = sin(self.th / 2.0)
        quat.w = cos(self.th / 2.0)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_link_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat

        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = 0.0
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)
        
        if vx == 0 and vth == 0 :
                odom.pose.covariance = ODOM_POSE_COVARIANCE2
                odom.twist.covariance = ODOM_TWIST_COVARIANCE2
        else :
            odom.pose.covariance = ODOM_POSE_COVARIANCE
            odom.twist.covariance = ODOM_TWIST_COVARIANCE

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_link_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = quat
            self.tf_broadcaster.sendTransform(t)

        # Debug
        if self.debug:
            self.get_logger().info(f'dt={dt:.3f}s, dleft={delta_left}, dright={delta_right}, x={self.x:.3f}, y={self.y:.3f}, th={self.th:.3f} rad')

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()