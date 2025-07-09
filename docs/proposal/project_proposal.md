# Raspberry Pi 5 — 你圈出的全部部件速查表

| #  | 位置 / 外观            | 官方名称                             | 关键规格                                           | 在小车项目里的典型用途                                                |
| -- | ------------------ | -------------------------------- | ---------------------------------------------- | ---------------------------------------------------------- |
| 1  | 顶部 40-pin 黑色排针     | **HAT GPIO Header**              | 3 V3 逻辑、I²C/SPI/UART/PWM、5 V & GND (向下兼容 Pi 4) | 挂 HEX-o-SKIN、IMU、LiDAR-Lite、PWM 控电机、TRIG/ECHO、编码器等全部数字 I/O |
| 2  | USB-A 堆栈 (上黑、下蓝)   | **USB 2.0 ×2** / **USB 3.0 ×2**  | 双 USB2 480 Mbps；双 USB3 5 Gbps，合计 1.2 A VBUS    | USB-SSD、深度相机、调试 Dongle；黑口给低速设备，蓝口跑高速                       |
| 3  | 4-pin 白色 JST-SH    | **PWM Fan Header**               | 5 V / 1 A、PWM+TACHO、固件控温                       | 装主动散热风扇，封闭舱或高负载防降频                                         |
| 4  | 金属 RJ45 口          | **千兆 Ethernet + PoE+ MagJack**   | 10/100/1000 Base-T；PoE+ 经外接 HAT 可供 \~5 V/4 A   | 室内开发稳固网 + 供电；户外一般留空                                        |
| 5  | 黑色 Micro-Fit-4     | **PoE Power In (J19)**           | 5 V 回送，4 pin (5 V×2 GND×2)                     | ① PoE+ HAT 输出<br>② 可自制 5 V 高流输入                            |
| 6  | 黄色 M2.5 孔          | **固定孔 / 接地环**                    | 镀金铜环连 GND                                      | 螺柱固定；可把金属底盘与板 GND 共地                                       |
| 7  | 右侧双 22-pin FFC     | **CAM0/1 & DISP0/1 4-lane MIPI** | 每口 CSI-2 / DSI-1 可选，4-lane 4 Gbps              | 双目摄像、4K DSI 屏，省 USB 带宽                                     |
| 8  | 左侧 16-pin FFC      | **PCIe 2.0 x1 FPC**              | 5 GT/s；3 V3\@600 mA                            | NVMe SSD、2.5 G NIC、FPGA 卡等高速扩展                             |
| 9  | 双 micro-HDMI       | **HDMI 2.0 Type-D ×2**           | 单 4K 60 Hz；双 4K 30 Hz                          | 开发时接显示器 / 双屏演示，运行中可关闭省电                                    |
| 10 | 2-pin JST-SH       | **RTC Battery**                  | LIR/CR2032 → VBAT 备电                           | 断电仍跑时钟，日志时间不漂                                              |
| 11 | 3-pin JST-SH       | **UART Debug**                   | GND / TXD0 / RXD0 @ 3 V3                       | 永久串口控制台，救砖 & 无网调试                                          |
| 12 | USB-C 单口           | **主电源 PWR\_IN**                  | USB-PD 5 V / 5 A（无数据线）                         | 插官方 27 W PSU 或 PD Power-Bank 供电                            |
| 13 | 侧边贴片板 + 白色 2-pin焊盘 | **PWR/ACT LED + RUN Reset**      | 红 = 上电；绿 = 存储活动；短接 RUN = 复位                    | 外壳透光看状态；接按钮做硬重启键                                           |

> **使用提示**
>
> * **供电**：USB-C 只负责 5 V 进电；PoE Micro-Fit 只能进不出；别从 GPIO 5 V pin 给电机。
> * **高速排线**：PCIe FPC 16-pin 专用；双 MIPI 22-pin 不可混插。
> * **调试救援**：UART 3-pin + RUN 焊按钮 + LED 闪码 = 无屏也能复位与诊断。
