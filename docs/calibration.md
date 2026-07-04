# 标定流程
## 1. 标定相机 ROI

在每个 MaixCAM 脚本中设置 ROI：

```python
PROCESS_ROI_ENABLE = True
PROCESS_ROI = (x, y, w, h)
```

ROI 应该覆盖黑色靶环，并保留少量边缘余量。ROI 越小，处理速度通常越高，背景误识别也越少。

## 2. 调整靶环识别

如果黑色靶环识别不稳定，优先调整这些参数：

- `BLACK_GRAY_MAX`
- `BLACK_RING_MIN_RADIUS`
- `BLACK_RING_MAX_RADIUS`
- `RING_HOUGH_PARAM2`
- `RING_EDGE_FIT_BAND`

调好后，屏幕叠加显示中的靶环中心应该稳定，不能频繁大幅跳动。

## 3. 调整激光点识别

如果红色激光点识别不稳定，优先调整 LAB 红色阈值和光斑过滤参数：

- `color_thresholds_red`
- `MIN_SPOT_AREA`
- `MAX_SPOT_AREA`
- `MIN_SPOT_CIRCULARITY`

建议保持 `LASER_RESTRICT_TO_RING = True`，这样靶环外的红色噪声不会触发电机运动。

## 4. 调整运动死区

当前死区参数为：

```python
DIRECTION_DEADZONE_PX = 8
```

如果光点靠近中心时电机来回抖动，可以适当增大死区。如果最终准直误差偏大，可以适当减小死区。

## 5. 调整电机步长

ESP32-S3 固件中主要调这两个参数：

```cpp
constexpr uint32_t kMovePulse = 50;
constexpr uint32_t kSecondMovePulse = 3;
```

如果光点容易越过中心，说明步长偏大，应减小脉冲数。如果收敛太慢，可以逐步增大脉冲数。

## 6. 记录实验结果

正式展示前建议记录：

- 视觉识别帧率。
- 收敛后的稳定像素误差。
- 平均收敛时间。
- `missing` 搜索是否能找回目标。
- 一张清晰系统照片或一段短动图。
