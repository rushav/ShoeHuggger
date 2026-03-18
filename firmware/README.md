# OpenCR Firmware Modifications

## Overview

AlienBot uses a modified version of the GIX OpenCR firmware that adds support for a 3rd DYNAMIXEL motor (ID 3) used to drive the wing mechanism.

## Source Repository

The firmware is based on the `t516_OpenCR` repository provided for TECHIN 516.

## Configuration

In `t516_OpenCR.ino`, select the robot model:
```cpp
#define MODEL GIX_Waffle
```

## Key Modifications

### Velocity Limit
- `LIMIT_X_MAX_VELOCITY` increased from **337** to **500** to allow faster driving

### Wheel Separation
- Modified from **0.287 m** to **0.1552 m** to match the actual wheel-to-wheel distance of the custom chassis

### GIX Motor Driver (Wing Motor — DYNAMIXEL ID 3)
The GIX motor driver firmware extension reads and writes to DYNAMIXEL ID 3 using these control table addresses:

| Address | Description |
|---------|-------------|
| 346     | Wing command input |
| 348-382 | Motor control registers |

The wing motor accepts:
- **Position commands** via `/wing_command` (Float64, 0.0–1.0 normalized)
- **Flap speed commands** via `/wing_flap_speed` (Float64, 0.0–1.0; 0.0 = stop flapping)

The bringup node on the Pi translates these ROS topics into DYNAMIXEL SDK writes.

## Flashing

1. Open `t516_OpenCR.ino` in Arduino IDE
2. Select board: **OpenCR**
3. Connect OpenCR via USB
4. Upload firmware
5. Verify by checking that all three motors respond (IDs 1, 2, 3)
