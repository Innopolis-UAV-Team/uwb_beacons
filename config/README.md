# Configuration Files

This directory contains configuration files for the UWB Localizer ROS2 package.

## Files

### `uwb_beacons_params_example.yaml`
**Template configuration file** - Copy this to create your own configuration.

```bash
cp uwb_beacons_params_example.yaml uwb_beacons_params.yaml
```

This file includes:
- Detailed comments for every parameter
- Example anchor layouts for different room sizes
- Calibration parameter explanations
- Setup tips and best practices

### `uwb_beacons_params.yaml` (ignored by git)
**Your personal configuration** - This file is ignored by git to prevent accidental commits of personal settings.

Edit this file to configure:
- Serial port and baud rate
- Anchor positions for your setup
- Calibration parameters
- Frame ID and coordinate system
- Performance settings

### `example_room_setup.yaml`
**Complete example** for a 4m × 4m room setup with 4 anchors.

## Quick Setup

1. Copy the example file:
   ```bash
   cp uwb_beacons_params_example.yaml uwb_beacons_params.yaml
   ```

2. Edit your configuration:
   ```bash
   nano uwb_beacons_params.yaml
   ```

3. Launch with your settings:
   ```bash
   ros2 launch uwb_beacons uwb_beacons.launch.py
   ```

# Parameters
### Serial Communication
- `port`: Serial port device (default: `/dev/ttyUSB0`)
- `baud`: Baud rate (default: `115200`)
- `timeout`: Serial timeout in seconds (default: `1.0`)

### Coordinate System
- `frame_id`: Frame ID for published poses (default: `map`)
- `z_sign`: Z-axis sign for position calculation (default: `1`)

### Performance
- `timer_frequency`: Update frequency in Hz (default: `100.0`)

### Range Sensor
- `min_range`: Minimum range in meters (default: `0.0`)
- `max_range`: Maximum range in meters (default: `100.0`)
- `field_of_view`: Field of view in radians (default: `0.1`)
- `radiation_type`: Radiation type (default: `1` for INFRARED)

### Calibration
- `calibration`: Calibration type - `linear`, `quadratic`, `cubic`, or `none` (default: `linear`)
- `calib_params`: Calibration parameters array:
  - Linear: `[scale_factor, offset]` (default: `[1.0, 0.0]`)
  - Quadratic: `[a, b, c]` (default: `[0.0, 1.0, 0.0]`)
  - Cubic: `[a, b, c, d]` (default: `[0.0, 0.0, 1.0, 0.0]`)

### Anchor Positions
- `anchors`: Dictionary of anchor positions in format `{anchor_id: [x, y, z]}`
  - At least 3 anchors required for trilateration
  - Positions in meters relative to the frame_id origin

## Important Notes

- **Anchor positions** must be measured accurately in meters
- **Calibration parameters** need to be determined for your specific UWB system
- **At least 3 anchors** are required for trilateration
- The `uwb_beacons_params.yaml` file is ignored by git to keep your personal settings private
