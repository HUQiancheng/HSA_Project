#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import serial
import time

# 串口设备路径（请根据你的系统确认是否是 ttyUSB0 或 ttyACM0）
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 9600

# 力和PWM映射参数
F_MIN = 0.001
F_MAX = 0.01
PWM_MIN = 80
PWM_MAX = 255

def map_force_to_pwm(force, f_min=F_MIN, f_max=F_MAX, pwm_min=PWM_MIN, pwm_max=PWM_MAX):
    """将 0.001~0.01 范围的力映射到 PWM（80~255）"""
    if force < f_min:
        return 0
    if force > f_max:
        force = f_max
    scale = (force - f_min) / (f_max - f_min)
    pwm = int(pwm_min + scale * (pwm_max - pwm_min))
    return min(max(pwm, 0), 255)

class SerialMotorController:
    def __init__(self):
        # 初始化串口连接
        self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        rospy.init_node("serial_motor_controller")
        rospy.Subscriber("/forces", Float64MultiArray, self.callback)
        rospy.loginfo("✅ 串口电机控制节点已启动，监听 /forces")

    def callback(self, msg):
        f = msg.data
        if len(f) != 4:
            rospy.logwarn("⚠️ /forces 消息必须包含4个方向的力值")
            return

        f_front, f_back, f_left, f_right = f

        # 找最大力的方向
        direction = max(
            [('front', f_front), ('back', f_back),
             ('left', f_left), ('right', f_right)],
            key=lambda x: abs(x[1])
        )
        name, strength = direction
        pwm = map_force_to_pwm(abs(strength))

        if name == 'front':     # 向后推 → 后退（电机反转）
            pwmA, dirA = pwm, 0
            pwmB, dirB = pwm, 0
        elif name == 'back':    # 向前推 → 前进（正转）
            pwmA, dirA = pwm, 1
            pwmB, dirB = pwm, 1
        elif name == 'left':    # 向右推 → 左转（右轮前进）
            pwmA, dirA = pwm, 1
            pwmB, dirB = pwm, 0
        elif name == 'right':   # 向左推 → 右转（左轮前进）
            pwmA, dirA = pwm, 0
            pwmB, dirB = pwm, 1
        else:
            pwmA = pwmB = dirA = dirB = 0

        cmd = f"{pwmA},{dirA},{pwmB},{dirB}\n"
        try:
            self.ser.write(cmd.encode())
            rospy.loginfo(f"📤 已发送: {cmd.strip()} （方向: {name}, 力: {strength:.4f}, PWM: {pwm}）")
        except serial.SerialException as e:
            rospy.logerr(f"❌ 串口写入失败: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = SerialMotorController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
