# Smart Optical Collimation

基于 MaixCAM Pro 和 ESP32-S3 的智能光路准直系统。系统通过视觉识别黑色靶环中心与红色激光点位置偏差，经 UART 向下位机发送方向指令，驱动双轴电机完成自动准直；当目标丢失时，下位机进入扩展方形搜索模式恢复目标。

## Project Highlights

- **视觉闭环控制**：MaixCAM 识别靶环中心和激光点，计算像素偏差并输出方向指令。
- **双相机协同**：`camera1.py` 用于二维粗/细准直，`camera2.py` 用于第二阶段水平微调。
- **下位机实时执行**：ESP32-S3 解析 `left,up`、`stop`、`missing` 等串口协议并控制两组电机。
- **目标丢失恢复**：收到 `missing` 后，下位机控制第一组电机进行逐渐扩大的方形搜索。
- **工程化整理**：包含上位机代码、下位机固件、串口协议、接线说明和标定流程。

## Repository Layout

```text
.
├── src/                    # ESP32-S3 PlatformIO firmware
├── include/                # Motor driver protocol header
├── host/maixcam/           # MaixCAM upper-computer vision scripts
├── docs/                   # Architecture, protocol, wiring, calibration
├── archive/                # Historical MaixCAM experiment versions
├── assets/                 # Demo images/GIFs, system photos
└── platformio.ini          # PlatformIO board configuration
```

## System Flow

```text
Laser target image
       ↓
MaixCAM vision detection
       ↓
Ring center / laser spot offset
       ↓
Direction command over UART
       ↓
ESP32-S3 command parser
       ↓
EMM V5 motor drivers
       ↓
Two-axis optical adjustment
```

## Current Hardware

| Module | Role |
| --- | --- |
| MaixCAM Pro | Vision detection and UART command output |
| ESP32-S3 DevKitC-1 | Lower-computer command parsing and motor control |
| EMM V5 motor drivers | Stepper motor position control |
| Two motor groups | Primary two-axis adjustment and secondary compensation |

## Quick Start

### Firmware

Install PlatformIO, then build the ESP32-S3 firmware:

```powershell
platformio run
```

Upload after selecting the correct serial port:

```powershell
platformio run --target upload
```

### MaixCAM Host

Copy one of the MaixCAM scripts to the device and run it:

```text
host/maixcam/camera1.py
host/maixcam/camera2.py
```

`camera1.py` outputs full two-axis direction commands. `camera2.py` only outputs horizontal correction commands: `left only`, `right only`, and `stop`.

## UART Protocol

The host sends one text command per line at `115200 bps`.

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

See [docs/protocol.md](docs/protocol.md) for details.

## Demo Metrics

These values should be filled after a stable lab run:

| Metric | Value |
| --- | --- |
| Vision FPS | TBD |
| UART command period | 50 ms |
| Stable collimation error | TBD px |
| Average convergence time | TBD s |
| Missing-target recovery | Supported |

## Documentation

- [System architecture](docs/architecture.md)
- [UART protocol](docs/protocol.md)
- [Wiring notes](docs/wiring.md)
- [Calibration workflow](docs/calibration.md)
- [Demo record template](docs/demo.md)

## Status

The firmware currently builds successfully with PlatformIO for `esp32-s3-devkitc-1`. The MaixCAM scripts are copied from the working experiment versions and organized for repository use.
