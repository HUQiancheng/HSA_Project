#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
import serial
import time

class LidarPublisher:
    def __init__(self):
        rospy.init_node('lidar_publisher_node')
        
        # Parameters
        self.port = rospy.get_param('~port', '/dev/ttyACM0')
        self.baudrate = rospy.get_param('~baudrate', 9600)
        self.frame_id = rospy.get_param('~frame_id', 'lidar_link')
        
        # Publisher
        self.range_pub = rospy.Publisher('/lidar_distance', Range, queue_size=10)
        
        # Serial connection
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            rospy.loginfo(f"Connected to {self.port}")
            rospy.loginfo(f"LIDAR Publisher started on {self.port}")
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
                            distance_cm = int(line[2:])-10 # Lidar has a 10cm correction as initial distance
                            if 1 <= distance_cm <= 4000:  # Valid range check
                                self.publish_range(distance_cm)
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
            
    def publish_range(self, distance_cm):
        msg = Range()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.008  # ~0.5 degree
        msg.min_range = 0.01  # 1cm
        msg.max_range = 40.0  # 40m
        msg.range = distance_cm / 100.0  # Convert to meters
        
        self.range_pub.publish(msg)
        rospy.loginfo(f"Distance: {distance_cm} cm ({msg.range:.2f} m)")

if __name__ == '__main__':
    try:
        lidar = LidarPublisher()
        lidar.run()
    except rospy.ROSInterruptException:
        pass