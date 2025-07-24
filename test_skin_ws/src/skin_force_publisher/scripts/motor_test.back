#!/usr/bin/env python3
import RPi.GPIO as GPIO, time

# -------- 1. 引脚映射（与你的接线一一对应） --------
PWM_A, PWM_B   = 18, 19          # ENA, ENB
DIR_A, DIR_B   = (17, 27), (22, 23)  # (IN1,IN2), (IN3,IN4)
MOTORS         = [(PWM_A, *DIR_A), (PWM_B, *DIR_B)]

# -------- 2. GPIO 初始化 --------
GPIO.setmode(GPIO.BCM)
for pwm_pin, in1, in2 in MOTORS:
    GPIO.setup([pwm_pin, in1, in2], GPIO.OUT, initial=GPIO.LOW)

pwms = [GPIO.PWM(p, 1000) for p, *_ in MOTORS]
for p in pwms: p.start(0)         # 初始 0% 占空比

def set_motor(idx, speed):        # speed -100…100
    pwm_pin, in1, in2 = MOTORS[idx]
    forward = speed >= 0
    GPIO.output(in1, forward)
    GPIO.output(in2, not forward)
    pwms[idx].ChangeDutyCycle(abs(speed))

try:
    print("▶ 前进")
    set_motor(0, 70)
    time.sleep(2)
finally:
    for p in pwms: p.stop()
    GPIO.cleanup()
    print("GPIO 释放完毕，测试结束")
