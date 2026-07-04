# Wiring Notes

These notes reflect the current constants in `src/main.cpp`.

## ESP32-S3 UART Pins

| Function | ESP32-S3 RX | ESP32-S3 TX | Baud |
| --- | ---: | ---: | ---: |
| HOST1 / first MaixCAM | GPIO38 | GPIO39 | 115200 |
| HOST2 / second MaixCAM | GPIO13 | GPIO14 | 115200 |
| EMM V5 motor drivers | GPIO18 | GPIO17 | 115200 |

Connect UART crosswise:

```text
ESP32 TX -> device RX
ESP32 RX <- device TX
GND      -> GND
```

## Motor Addresses

| Motor | Address | Role |
| --- | ---: | --- |
| M1 | 1 | Primary pan axis |
| M2 | 2 | Primary tilt axis |
| M3 | 3 | Secondary pan axis |
| M4 | 4 | Secondary tilt axis |

## Buttons

| GPIO | Function | Wiring |
| ---: | --- | --- |
| GPIO6 | Force pause | Button to GND, internal pull-up |
| GPIO7 | Program restart | Button to GND, internal pull-up |

## Bring-Up Checklist

1. Power motor drivers and ESP32-S3 with a shared ground.
2. Confirm each EMM V5 driver address matches the table above.
3. Open the ESP32-S3 USB serial monitor at `115200`.
4. Send `left only`, `right only`, `up only`, `down only`, and `stop`.
5. If directions are reversed, adjust the direction constants in `src/main.cpp`.
6. Connect the MaixCAM UART and confirm commands appear in the ESP32-S3 serial log.
