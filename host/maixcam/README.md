# MaixCAM Host Scripts

This folder contains the upper-computer vision scripts for MaixCAM Pro.

## Files

| File | Role |
| --- | --- |
| `camera1.py` | Primary two-axis vision controller. Outputs full direction commands and `missing`. |
| `camera2.py` | Secondary horizontal fine-adjustment controller. Outputs only `left only`, `right only`, and `stop`. |

## Runtime

The scripts use:

- `maix.camera`
- `maix.display`
- `maix.image`
- `maix.uart`
- OpenCV `cv2`
- NumPy

UART defaults:

```python
UART_DEVICE = "/dev/ttyS0"
UART_BAUD = 115200
UART_SEND_PERIOD_MS = 50
```

## Deployment

Copy the selected script to the MaixCAM device and run it from the MaixPy environment. Keep UART debug text disabled for normal lower-computer control:

```python
UART_SEND_DEBUG_TEXT = False
```

If you need to inspect command decisions on screen, use the display overlay and console output instead of sending verbose text to the ESP32-S3.
