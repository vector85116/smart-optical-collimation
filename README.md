# 智能光路准直系统

本项目是一个基于 MaixCAM Pro 和 ESP32-S3 的智能光路准直系统。系统通过视觉识别黑色靶环中心与红色激光点的位置偏差，经 UART 向下位机发送方向指令，驱动双轴电机完成自动准直；当目标丢失时，下位机会进入逐渐扩大的方形搜索模式，尝试重新找回目标。

## 项目亮点

- **视觉闭环控制**：MaixCAM 识别靶环中心和激光点，计算像素偏差并输出方向指令。
- **双相机协同**：`camera1.py` 用于二维粗准直和细准直，`camera2.py` 用于第二阶段水平微调。
- **下位机实时执行**：ESP32-S3 解析 `left,up`、`stop`、`missing` 等串口指令，并控制两组电机动作。
- **目标丢失恢复**：收到 `missing` 后，下位机控制第一组电机进行逐渐扩大的方形搜索。
- **工程化整理**：仓库包含上位机视觉代码、下位机固件、串口协议、接线说明和标定流程。

## 仓库结构

```text
.
├── src/                    # ESP32-S3 下位机固件
├── include/                # 电机驱动协议头文件
├── host/maixcam/           # MaixCAM 上位机视觉脚本
├── docs/                   # 架构、协议、接线和标定文档
├── archive/                # MaixCAM 历史实验版本
├── assets/                 # 演示图片、动图和系统照片
└── platformio.ini          # PlatformIO 开发板配置
```

## 系统流程

```text
激光靶面图像
       ↓
MaixCAM 视觉识别
       ↓
计算靶环中心与激光点偏差
       ↓
通过 UART 发送方向指令
       ↓
ESP32-S3 解析指令
       ↓
EMM V5 电机驱动器执行动作
       ↓
双轴光路调整
```

## 当前硬件组成

| 模块 | 作用 |
| --- | --- |
| MaixCAM Pro | 视觉识别与 UART 指令输出 |
| ESP32-S3 DevKitC-1 | 下位机指令解析与电机控制 |
| EMM V5 电机驱动器 | 步进电机位置控制 |
| 两组电机 | 第一组完成二维调整，第二组用于补偿或微调 |

## 快速开始

### 下位机固件

安装 PlatformIO 后，在项目根目录编译 ESP32-S3 固件：

```powershell
platformio run
```

选择正确串口后上传固件：

```powershell
platformio run --target upload
```

### MaixCAM 上位机

将下面其中一个脚本复制到 MaixCAM 设备并运行：

```text
host/maixcam/camera1.py
host/maixcam/camera2.py
```

`camera1.py` 输出完整二维方向指令；`camera2.py` 只输出水平微调指令：`left only`、`right only` 和 `stop`。

## 串口协议

上位机以 `115200 bps` 通过 UART 发送文本指令，每条指令建议以换行结束。

```text
left,up
left,down
right,up
right,down
left only
right only
up only
down only
stop
missing
```

详细说明见 [串口协议文档](docs/protocol.md)。

## 演示指标

下面的数据需要在稳定实验环境中实测后填写：

| 指标 | 数值 |
| --- | --- |
| 视觉识别帧率 | 待测 |
| UART 指令周期 | 50 ms |
| 稳定后准直误差 | 待测 px |
| 平均收敛时间 | 待测 s |
| 目标丢失恢复 | 已支持 |

## 项目文档

- [系统架构](docs/architecture.md)
- [串口协议](docs/protocol.md)
- [接线说明](docs/wiring.md)
- [标定流程](docs/calibration.md)
- [演示记录模板](docs/demo.md)

## 当前状态

下位机固件已经可以在 PlatformIO 中针对 `esp32-s3-devkitc-1` 成功编译。MaixCAM 上位机脚本已经从现有实验代码整理进仓库，后续可以继续补充系统照片、演示动图和实测指标。
