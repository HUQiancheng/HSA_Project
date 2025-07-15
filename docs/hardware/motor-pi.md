## 两块 L298N + 4 台减速直流电机 + Raspberry Pi 5 + AA 电池盒（6 × AA，≈ 9 V）

### -- 完整接线总览与注意事项 --

> **目标**：让 Raspberry Pi 5 精确控制 4 台 6 V-12 V 直流减速电机的正/反转和转速。
> **硬件**：
>
> * 2 × L298N 双路电机驱动板（共 4 路）
> * 4 × DC Getriebemotoren（车底盘配套）
> * 1 × 树莓派 5（GPIO 40 pin）
> * 1 × AA 电池盒（6 节 AA，≈ 9 V，带开关与两根线）用于 **电机** 供电
> * 1 × 官方 5 V-5 A USB-C PD 适配器用于 **树莓派** 供电

---

### 1. 供电与接地框架

```
9 V (电池 +) ─────────────┐
                         ├──► L298N-1 Vcc (12 V)
                         └──► L298N-2 Vcc (12 V)

电池 – (GND) ─────────────┐
                         ├──► L298N-1 GND
                         ├──► L298N-2 GND
                         └──► Raspberry Pi GND (任一 GND 针脚)
```

*树莓派的 5 V 轨 **绝对不要** 与电池 +9 V 相连；仅三块板 **共地**。*
若电池盒带开关，开关置于 + 线；电池容量建议 > 2000 mAh（NiMH）以提供峰值电流。

---

### 2. 单块 L298N 接线（每板控制 2 台电机）

| L298N 引脚   | 逻辑作用        | 连接到 Pi 5 BCM GPIO | 说明             |
| ---------- | ----------- | ----------------- | -------------- |
| ENA        | 电机 A 速度 PWM | 18 (PWM 0A) 🟢    | 拔掉 ENA 跳帽，接此引脚 |
| IN1        | 电机 A 方向位 1  | 17                | 高/低 配合 IN2     |
| IN2        | 电机 A 方向位 2  | 27                | —              |
| IN3        | 电机 B 方向位 1  | 22                | —              |
| IN4        | 电机 B 方向位 2  | 23                | —              |
| ENB        | 电机 B 速度 PWM | 19 (PWM 1A) 🟢    | 拔掉 ENB 跳帽      |
| 12 V / Vcc | 电机电源正       | 电池 +9 V           | 6-12 V 皆可      |
| 5 V        | 板载稳压输出      | **留空**            | 不回供 Pi         |
| GND        | 共地          | 电池 – & Pi GND     | —              |

> L298N-2 另一块完全相同，只是换一组 GPIO：

| L298N-2 引脚 | 逻辑作用        | 连接到 Pi 5 BCM GPIO  |
| ---------- | ----------- | ------------------ |
| ENA        | 电机 C 速度 PWM | **12** (PWM 0B) 🟢 |
| IN1        | 电机 C 方向位 1  | **5**              |
| IN2        | 电机 C 方向位 2  | **6**              |
| IN3        | 电机 D 方向位 1  | **20**             |
| IN4        | 电机 D 方向位 2  | **21**             |
| ENB        | 电机 D 速度 PWM | **13** (PWM 1B) 🟢 |
| 其余电源脚      | 同上          |                    |

这样共用 **8 根普通 GPIO + 4 根硬件 PWM**，Pi 5 的四路硬件 PWM 全部用上（18/19/12/13）。若只需全速，保留跳帽不连 PWM，GPIO 数可减半。

---

### 3. 电机接线

* **电机 A (前左)** ➜ L298N-1 `OUT1` - `OUT2`
* **电机 B (前右)** ➜ L298N-1 `OUT3` - `OUT4`
* **电机 C (后左)** ➜ L298N-2 `OUT1` - `OUT2`
* **电机 D (后右)** ➜ L298N-2 `OUT3` - `OUT4`

> 若装配后发现某轮转向与预期相反，可**对调该电机的两根 OUT 线**或在软件里交换 INx 输出高低。

---

### 4. 树莓派 GPIO 物理脚汇总

| 方向    | BCM | 物理 Pin                | 职责        |
| ----- | --- | --------------------- | --------- |
| PWM0A | 18  | 12                    | ENA-1（前左） |
| PWM1A | 19  | 35                    | ENB-1（前右） |
| PWM0B | 12  | 32                    | ENA-2（后左） |
| PWM1B | 13  | 33                    | ENB-2（后右） |
| DIR   | 17  | 11                    | IN1-1     |
| DIR   | 27  | 13                    | IN2-1     |
| DIR   | 22  | 15                    | IN3-1     |
| DIR   | 23  | 16                    | IN4-1     |
| DIR   | 5   | 29                    | IN1-2     |
| DIR   | 6   | 31                    | IN2-2     |
| DIR   | 20  | 38                    | IN3-2     |
| DIR   | 21  | 40                    | IN4-2     |
| GND   | —   | 6/9/14/20/25/30/34/39 | 共地        |

---

### 5. 上电与调试顺序

1. **断电状态**完成全部接线，确认：

   * Pi 5 **未**接 L298N 5 V 引脚。
   * 电机供电与树莓派供电**完全分离**仅共地。
   * ENA/ENB 跳帽已拔出（如需 PWM）。
2. 先给**树莓派**上电并 SSH 登录，**不接**电机电源，运行测试脚本检查 GPIO 输出无误（LED 或万用表测）。
3. 关闭脚本，接入 **AA 电池盒** 开关至 OFF 位置，将 +9 V 线接两块 L298N 的 Vcc，- 线接公共地。
4. 打开电池开关后再运行脚本，观察四轮能否按指令正转 / 反转 / 调速。
5. 若树莓派突然重启或出现欠压闪电符，说明 GND 未连好或电机电干扰回灌 5 V——检查布线并在电机端加 100 nF 陶瓷电容抑制火花。

---

### 6. Python 控制框架（四轮差速示例）

```python
# 文件: four_motor_driver.py
import RPi.GPIO as GPIO

# GPIO map：见上表
PWM_PINS = [18, 19, 12, 13]      # ENA1, ENB1, ENA2, ENB2
DIR_PINS = [
    (17, 27),  # A 方向
    (22, 23),  # B 方向
    (5, 6),    # C 方向
    (20, 21)   # D 方向
]

GPIO.setmode(GPIO.BCM)
for pin in sum(DIR_PINS, []) + PWM_PINS:
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

pwms = [GPIO.PWM(p, 1000) for p in PWM_PINS]
for p in pwms: p.start(0)

def set_motor(idx, speed):        # idx 0-3, speed -100…100
    a, b = DIR_PINS[idx]
    forward = speed >= 0
    GPIO.output(a, forward)
    GPIO.output(b, not forward)
    pwms[idx].ChangeDutyCycle(min(abs(speed), 100))

def tank_drive(v_left, v_right):  # -100…100
    set_motor(0, v_left)
    set_motor(2, v_left)
    set_motor(1, v_right)
    set_motor(3, v_right)

if __name__ == "__main__":
    try:
        tank_drive(70, 70)   # 前进
        time.sleep(2)
        tank_drive(-70, -70) # 后退
        time.sleep(2)
        tank_drive(70, -70)  # 原地右转
        time.sleep(2)
    finally:
        for p in pwms: p.stop()
        GPIO.cleanup()
```

---

### 7. 常见故障速查

| 现象                | 排查要点                                                           |
| ----------------- | -------------------------------------------------------------- |
| 电机不动、L298N LED 不亮 | 9 V 未接 Vcc / 跳帽未拔 / EN 未输出 PWM                                 |
| 前后轮方向颠倒           | 交换对应 OUT 线或反转 INx 逻辑                                           |
| 树莓派 USB 设备掉线      | 电机启动电流尖峰 → GND 回路干扰；缩短 GND 线、在电机并联 TVS 或 RC Snubber            |
| 电机空转无力            | 电池电压不足（AA 镍氢<7 V 时扭矩明显下降）或 L298N 饱和压降大；考虑换 2×18650 (7 V-8.4 V) |
| 无法调速              | 硬件 PWM 引脚被占用、频率太低或 EN 跳帽未拔；用 `gpio readall` 查看实际波形             |

> **温馨提示**：L298N 是老旧双极晶体管驱动（典型 Vsat 2 V-4 V），效率低且发热。若后期需要更大电流或更高效率，推荐升级 TB6612FNG、DRV8833 或 BTS7960 (MOSFET H-bridge)。在原型阶段 L298N 足够稳健且资料丰富，便于调试。祝接线顺利!
