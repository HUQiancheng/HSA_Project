#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import serial
import time

# Serial configuration
SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 9600

# Force to PWM mapping parameters
F_MIN = 0.003      # Baseline force reading
F_MAX = 0.02       # Maximum expected force
PWM_MIN = 80       # Minimum PWM for motor movement
PWM_MAX = 255      # Maximum PWM

# E-skin cell ID mapping (from documentation)
CELL_FRONT = 0  # Index 0 = Cell ID 1 = Front
CELL_LEFT = 1   # Index 1 = Cell ID 2 = Left
CELL_BACK = 2   # Index 2 = Cell ID 3 = Back
CELL_RIGHT = 3  # Index 3 = Cell ID 4 = Right

def map_force_to_pwm(force, f_min=F_MIN, f_max=F_MAX, pwm_min=PWM_MIN, pwm_max=PWM_MAX):
    """Map force sensor reading to PWM value"""
    if force < f_min:
        return 0
    if force > f_max:
        force = f_max
    
    scale = (force - f_min) / (f_max - f_min)
    pwm = int(pwm_min + scale * (pwm_max - pwm_min))
    return min(max(pwm, 0), 255)

class MotorController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("motor_controller")
        
        # Initialize serial connection
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            rospy.loginfo(f"✅ Connected to Arduino on {SERIAL_PORT}")
        except serial.SerialException as e:
            rospy.logerr(f"❌ Failed to open serial port: {e}")
            rospy.logerr("Try: sudo usermod -a -G dialout $USER && logout/login")
            raise
        
        # Subscribe to skin forces topic
        rospy.Subscriber("/skin_forces", Float64MultiArray, self.force_callback)
        rospy.loginfo("✅ Motor controller ready, listening to /skin_forces")
        
    def force_callback(self, msg):
        """Process force data and send motor commands"""
        forces = msg.data
        
        if len(forces) != 4:
            rospy.logwarn("⚠️ Expected 4 force values, got %d", len(forces))
            return
        
        # Extract forces with correct mapping
        f_front = forces[CELL_FRONT]
        f_left = forces[CELL_RIGHT]
        f_back = forces[CELL_BACK]
        f_right = forces[CELL_LEFT]
        
        # Find direction with maximum force
        max_force = max(f_front, f_left, f_back, f_right)
        
        # Calculate PWM from force magnitude
        pwm = map_force_to_pwm(max_force)
        
        # Determine motor commands based on which sensor has max force
        if pwm == 0 or max_force < F_MIN:
            # Stop motors if no significant force
            self.send_motor_command(0, 0, 0, 0)
            rospy.loginfo("🛑 Stop (no force detected)")
            
        elif max_force == f_front:
            # Front touched → Move backward
            self.send_motor_command(pwm, 0, pwm, 0)  # Both motors reverse
            rospy.loginfo(f"⬇️ Backward (Front sensor: {f_front:.4f}, PWM: {pwm})")
            
        elif max_force == f_back:
            # Back touched → Move forward
            self.send_motor_command(pwm, 1, pwm, 1)  # Both motors forward
            rospy.loginfo(f"⬆️ Forward (Back sensor: {f_back:.4f}, PWM: {pwm})")
            
        elif max_force == f_left:
            # Left touched → Turn RIGHT (left forward, right reverse)
            self.send_motor_command(pwm, 1, pwm, 0)
            rospy.loginfo(f"➡️ Turn Right (Left sensor: {f_left:.4f}, PWM: {pwm})")
            
        elif max_force == f_right:
            # Right touched → Turn LEFT (right forward, left reverse)
            self.send_motor_command(pwm, 0, pwm, 1)
            rospy.loginfo(f"⬅️ Turn Left (Right sensor: {f_right:.4f}, PWM: {pwm})")
    
    def send_motor_command(self, pwmA, dirA, pwmB, dirB):
        """Send motor command to Arduino
        
        Motor A: Left side motors
        Motor B: Right side motors
        Direction: 1 = forward, 0 = reverse
        """
        cmd = f"{pwmA},{dirA},{pwmB},{dirB}\n"
        try:
            self.ser.write(cmd.encode())
            rospy.logdebug(f"Sent: {cmd.strip()}")
        except serial.SerialException as e:
            rospy.logerr(f"❌ Serial write failed: {e}")
    
    def shutdown(self):
        """Clean shutdown - stop motors and close serial"""
        rospy.loginfo("Shutting down motor controller...")
        self.send_motor_command(0, 0, 0, 0)
        self.ser.close()
    
    def run(self):
        """Main loop"""
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = MotorController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")