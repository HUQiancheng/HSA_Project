#!/usr/bin/env python3
import serial, time

PORT = "/dev/ttyACM0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)           # 让 MCU 完成复位
ser.reset_input_buffer()

# 可选：通过 CLI 指令切换输出格式，这里改为欧拉角
ser.write(b'#o[euler]\n')      # 详见固件 README

while True:
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if line:
        print(line)
