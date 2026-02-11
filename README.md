# vl53l0x_range

ROS 2 Jazzy driver for the ST VL53L0X Time-of-Flight (ToF) laser distance sensor over I2C.

## Features

- Publishes `sensor_msgs/Range` on the `range/data` topic
- **Fake mode** — generates random range data without physical hardware
- Calibrate and reset services for offset bias correction
- Runtime `publish_rate` change via `ros2 param set`
- Range mode selection: short (high accuracy), medium (balanced), long (max range)
- 940 nm VCSEL infrared laser, up to 2 m range

## Prerequisites

- ROS 2 Jazzy
- Python 3
- `smbus2` (only required when `fake_mode` is `false`)

```bash
pip3 install smbus2
```

## Installation

```bash
cd ~/ros2_ws
colcon build --packages-select vl53l0x_range
source install/setup.bash
```

## Usage

### Launch with default parameters (fake mode)

```bash
ros2 launch vl53l0x_range vl53l0x_launch.py
```

### Run with real hardware

```bash
ros2 run vl53l0x_range vl53l0x_node.py --ros-args -p fake_mode:=false
```

### Verify output

```bash
ros2 topic echo /range/data
```

### Services

```bash
ros2 service call /range/calibrate std_srvs/srv/Trigger
ros2 service call /range/reset std_srvs/srv/Trigger
```

### Change publish rate at runtime

```bash
ros2 param set /vl53l0x_range_node publish_rate 20.0
```

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `fake_mode` | bool | `true` | `true`: generate random data, `false`: read from real I2C device |
| `i2c_bus` | int | `1` | I2C bus number (`/dev/i2c-N`) |
| `device_address` | int | `0x29` | VL53L0X I2C address (default `0x29`) |
| `publish_rate` | double | `10.0` | Publishing rate in Hz (runtime changeable) |
| `frame_id` | string | `tof_link` | TF frame ID in message headers |
| `range_mode` | string | `medium` | Range mode: `short`, `medium`, `long` |
| `field_of_view` | double | `0.44` | Field of view in radians (~25 degrees) |
| `min_range` | double | `0.03` | Minimum range in metres (30 mm) |
| `max_range` | double | `2.0` | Maximum range in metres (2000 mm) |

## Range Modes

| Mode | Max Range | Description |
|---|---|---|
| `short` | ~1.2 m | High accuracy, short timing budget |
| `medium` | ~1.5 m | Balanced (default) |
| `long` | ~2.0 m | Maximum range, lower accuracy |

## Package Structure

```
vl53l0x_range/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── vl53l0x_params.yaml
├── launch/
│   └── vl53l0x_launch.py
├── vl53l0x_range/
│   ├── __init__.py
│   └── vl53l0x_driver.py
└── nodes/
    └── vl53l0x_node.py
```

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
