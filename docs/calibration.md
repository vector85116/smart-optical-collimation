# Calibration Workflow

Use this workflow before recording a demo or showing the system to a teacher.

## 1. Camera ROI

Set the ROI in each MaixCAM script:

```python
PROCESS_ROI_ENABLE = True
PROCESS_ROI = (x, y, w, h)
```

The ROI should cover the black target ring with a small margin. A smaller ROI improves frame rate and reduces false detections.

## 2. Ring Detection

Tune these values if the black ring is unstable:

- `BLACK_GRAY_MAX`
- `BLACK_RING_MIN_RADIUS`
- `BLACK_RING_MAX_RADIUS`
- `RING_HOUGH_PARAM2`
- `RING_EDGE_FIT_BAND`

The overlay should show a stable ring center without large jumps.

## 3. Laser Spot Detection

Tune the LAB red threshold and spot filters:

- `color_thresholds_red`
- `MIN_SPOT_AREA`
- `MAX_SPOT_AREA`
- `MIN_SPOT_CIRCULARITY`

Keep `LASER_RESTRICT_TO_RING = True` so red noise outside the target does not trigger movement.

## 4. Motion Deadzone

Set:

```python
DIRECTION_DEADZONE_PX = 8
```

Increase it if the system jitters near the center. Decrease it if final alignment error is too large.

## 5. Motor Step Size

Tune the ESP32-S3 firmware constants:

```cpp
constexpr uint32_t kMovePulse = 50;
constexpr uint32_t kSecondMovePulse = 3;
```

If the beam overshoots, reduce the pulse value. If convergence is slow, increase it gradually.

## 6. Record Results

Before showing the project, record:

- Vision FPS.
- Stable pixel error after convergence.
- Average convergence time.
- Whether `missing` search can recover the target.
- One clear photo or GIF of the full system running.
