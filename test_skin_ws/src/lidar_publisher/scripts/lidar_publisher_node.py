#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan  # Changed to LaserScan message type
import serial
import time

class LidarPublisher:
    def __init__(self):
        rospy.init_node('lidar_publisher_node')
        
        # Parameters
        self.port = rospy.get_param('~port', '/dev/ttyACM0')
        self.baudrate = rospy.get_param('~baudrate', 9600)
        self.frame_id = rospy.get_param('~frame_id', 'lidar_lite')  # Match C++ code frame_id
        
        # Publisher - Changed to scan topic with LaserScan message type
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
        
        # Serial connection
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            rospy.loginfo(f"Connected to {self.port}")
            rospy.loginfo(f"LIDAR Publisher started on {self.port}, publishing LaserScan to /scan")
            time.sleep(2)  # Arduino reset time
            self.ser.reset_input_buffer()  # Clear any startup garbage
        except Exception as e:
            rospy.logerr(f"Failed to open serial port: {e}")
            exit(1)
            
    def run(self):
        rate = rospy.Rate(20)  # 20Hz
        
        while not rospy.is_shutdown():
            try:
                if self.ser.in_waiting > 0:
                    # Read with error handling
                    line_bytes = self.ser.readline()
                    try:
                        line = line_bytes.decode('utf-8', errors='ignore').strip()
                    except:
                        continue
                    
                    # Parse "L:xxx" format
                    if line.startswith("L:") and len(line) > 2:
                        try:
                            distance_cm = int(line[2:]) - 10  # Lidar has a 10cm correction as initial distance
                            if 1 <= distance_cm <= 4000:  # Valid range check
                                self.publish_laserscan(distance_cm)
                        except ValueError:
                            rospy.logdebug(f"Invalid distance value: {line}")
                            
            except serial.SerialException as e:
                rospy.logerr(f"Serial error: {e}")
                break
            except Exception as e:
                rospy.logerr(f"Unexpected error: {e}")
                
            rate.sleep()
        
        # Cleanup
        if self.ser.is_open:
            self.ser.close()
            rospy.loginfo("Serial port closed")
            
    def publish_laserscan(self, distance_cm):
        """
        Publish LaserScan message - wrap single-point LiDAR data into LaserScan format
        Following the C++ code configuration
        """
        msg = LaserScan()
        
        # Header configuration
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        
        # Angle configuration - single-point LiDAR, so angle range is 0
        msg.angle_min = 0.0
        msg.angle_max = 0.0
        msg.angle_increment = 1.0
        
        # Time configuration
        msg.time_increment = 0.0
        msg.scan_time = 0.0  # Commented in C++ code, but we can set it
        
        # Range configuration
        msg.range_min = 0.0
        msg.range_max = 40.0
        
        # Distance data - convert to meters
        distance_in_meters = distance_cm / 100.0
        msg.ranges = [distance_in_meters]  # Single-point data array
        
        # Intensity data - use original cm value as intensity (following C++ code)
        msg.intensities = [float(distance_cm)]
        
        # Publish message
        self.scan_pub.publish(msg)
        rospy.loginfo(f"Distance: {distance_cm} cm ({distance_in_meters:.2f} m) -> LaserScan published")

if __name__ == '__main__':
    try:
        lidar = LidarPublisher()
        lidar.run()
    except rospy.ROSInterruptException:
        pass