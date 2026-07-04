# MaixCAM 上位机脚本

这个目录保存 MaixCAM Pro 上运行的视觉识别和串口控制脚本。

## 文件说明

| 文件 | 作用 |
| --- | --- |
| `camera1.py` | 第一台相机脚本，用于二维视觉控制，可以输出完整方向指令和 `missing`。 |
| `camera2.py` | 第二台相机脚本，用于水平微调，只输出 `left only`、`right only` 和 `stop`。 |

## 运行依赖

脚本主要使用：

- `maix.camera`
- `maix.display`
- `maix.image`
- `maix.uart`
- OpenCV `cv2`
- NumPy

默认串口参数：

```python
UART_DEVICE = "/dev/ttyS0"
UART_BAUD = 115200
UART_SEND_PERIOD_MS = 50
```

## 部署方式

将需要运行的脚本复制到 MaixCAM 设备，在 MaixPy 环境中运行即可。正常控制下位机时，建议保持调试文本关闭：

```python
UART_SEND_DEBUG_TEXT = False
```

如果需要观察指令判断过程，可以看屏幕叠加显示和控制台输出，不建议把完整调试文本发给 ESP32-S3。
