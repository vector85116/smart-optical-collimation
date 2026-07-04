# System Architecture

This project is organized as a two-level closed-loop optical collimation system.

## Upper Computer: MaixCAM

The MaixCAM script captures a 640 x 480 image, optionally crops a configured ROI, detects the black target ring, detects the red laser spot, and converts the pixel offset into a motion command.

Main vision stages:

1. Crop the configured ROI to reduce noise and improve speed.
2. Detect the black ring with Hough circle detection.
3. Refine the ring with full-resolution local detection and edge fitting.
4. Detect the red laser spot with LAB color thresholding.
5. Restrict laser detection to the valid disk inside the ring.
6. Smooth ring and spot positions to avoid command jitter.
7. Convert offset signs into UART command text.

## Lower Computer: ESP32-S3

The ESP32-S3 receives commands from USB, HOST1 UART, or HOST2 UART. It parses the command text and sends EMM V5 driver frames over the motor UART.

The firmware supports:

- Primary motor group `M1M2`: pan and tilt correction.
- Secondary motor group `M3M4`: compensation or horizontal fine adjustment.
- Command prefixes such as `g1` and `g2` for manual group selection.
- `missing` search mode for target loss recovery.
- GPIO force-pause and restart buttons.

## Control Loop

```text
Image frame
  -> ring center and laser spot detection
  -> dx/dy error calculation
  -> direction command
  -> ESP32-S3 command parser
  -> motor pulse movement
  -> next image frame
```

The current control law is step-based rather than PID-based. Each command causes a fixed pulse movement. This keeps the first version stable and easy to tune; future versions can add proportional pulse scaling or PID control.
