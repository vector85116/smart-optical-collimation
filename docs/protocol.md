# UART Protocol

The MaixCAM upper-computer scripts communicate with the ESP32-S3 lower computer by sending one ASCII text command per line.

## Serial Settings

| Item | Value |
| --- | --- |
| Baud rate | 115200 |
| Format | 8N1 |
| Line ending | `\n` recommended |
| Host script send period | 50 ms |

The ESP32-S3 firmware also accepts commands when no line ending is sent, using a short receive timeout.

## Commands

| Command | Meaning |
| --- | --- |
| `left,up` | Move pan left and tilt up |
| `left,down` | Move pan left and tilt down |
| `right,up` | Move pan right and tilt up |
| `right,down` | Move pan right and tilt down |
| `left only` | Move pan left only |
| `right only` | Move pan right only |
| `up only` | Move tilt up only |
| `down only` | Move tilt down only |
| `stop` | Stop the selected motor group |
| `missing` | Start or continue target-loss search |

## Group Prefixes

Manual commands can specify a motor group:

```text
g1 left,up
g2 right only
m1m2 stop
m3m4 stop
```

If no prefix is supplied, each input channel uses its default group.

## Host Roles

| Input | Default group | Intended role |
| --- | --- | --- |
| USB serial | `M1M2` | Debug and manual test |
| HOST1 UART | `M1M2` | First MaixCAM / primary adjustment |
| HOST2 UART | `M3M4` | Second MaixCAM / compensation |

`camera1.py` can output all two-axis commands and `missing`. `camera2.py` intentionally limits UART output to `left only`, `right only`, and `stop`.

## Stop Behavior

The host scripts send several consecutive `stop` commands before stopping UART output. This ensures the lower computer receives a stable final stop state.

The lower computer repeats low-level stop frames to the motor driver, keeping holding torque after stopping.
