import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import RPi.GPIO as GPIO

class SENSpeedEncoderNode(Node):
    def __init__(self):
        super().__init__('senspeed_encoder_node')

        GPIO.setmode(GPIO.BCM)

        self.left_pin = 17
        self.right_pin = 27

        GPIO.setup(self.left_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.right_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.left_count = 0
        self.right_count = 0

        self.left_pub = self.create_publisher(Int32, 'wheel_left_ticks', 10)
        self.right_pub = self.create_publisher(Int32, 'wheel_right_ticks', 10)

        GPIO.add_event_detect(self.left_pin, GPIO.FALLING, callback=self.left_callback, bouncetime=2)
        GPIO.add_event_detect(self.right_pin, GPIO.FALLING, callback=self.right_callback, bouncetime=2)

        self.timer = self.create_timer(0.1, self.publish_counts)

        self.get_logger().info("SENSpeedEncoderNode started.")

    def left_callback(self, channel):
        self.left_count += 1

    def right_callback(self, channel):
        self.right_count += 1

    def publish_counts(self):
        left_msg = Int32()
        right_msg = Int32()
        left_msg.data = self.left_count
        right_msg.data = self.right_count

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

        self.get_logger().debug(f'Left ticks: {self.left_count}, Right ticks: {self.right_count}')

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SENSpeedEncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
