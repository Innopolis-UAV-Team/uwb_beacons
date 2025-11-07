# UWB beacons ROS2

This is the ROS2 version of the UWB (Ultra-Wideband) localization system. It provides real-time positioning using UWB anchors and trilateration algorithms.

## Features

- Real-time UWB distance measurements
- Trilateration-based positioning
- Multiple calibration methods (linear, quadratic, cubic)
- ROS2 native implementation
- Docker containerization support
- Configurable anchor positions
- Simplified deployment (no complex ROS2 workspace building required)

## Quick Start with Docker

### Prerequisites

- Docker and Docker Compose installed
- UWB device connected to `/dev/ttyUSB0` (or modify the path in docker-compose.yml)

### Running with Docker Compose

1. **Basic usage:**
   ```bash
   docker-compose up uwb-beacons
   ```

2. **With RViz visualization:**
   ```bash
   docker-compose --profile gui up
   ```

3. **With data recording:**
   ```bash
   docker-compose --profile recording up
   ```

4. **Build and run:**
   ```bash
   docker-compose build
   docker-compose up uwb-beacons
   ```

### Manual Docker Build

```bash
# Build the image
docker build -t uwb-beacons-ros2 .

# Run the container
docker run -it --rm \
  --device=/dev/ttyUSB0 \
  --network=host \
  uwb-beacons-ros2
```

## Manual ROS2 Installation

### Prerequisites

- ROS2 Humble installed
- Python 3.8+
- Required Python packages: numpy, pyserial, Localization

> [!NOTE]
> Localization should be installed outside the colcon build!


### Installation

1. **Clone and setup workspace:**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   # Copy the uwb_beacons package here
   ```

2. **Install dependencies:**
   ```bash
   sudo apt update
   sudo apt install python3-pip python3-numpy python3-scipy python3-serial
   pip3 install pyserial numpy Localization
   ```

3. **Build the workspace:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select uwb_beacon
   source install/setup.bash
   ```

4. **Run the node:**
   ```bash
   ros2 launch uwb_beacons uwb_beacons.launch.py
   ```

## Configuration

### Launch Parameters

- `port`: Serial port for UWB device (default: `/dev/ttyUSB0`)
- `baud`: Baud rate (default: `460800`)
- `frame_id`: Frame ID for published poses (default: `map`)
- `calibration`: Calibration type - `linear`, `quadratic`, or `cubic` (default: `linear`)
- `z_sign`: Z-axis sign for position calculation (default: `1`)

### Anchor Configuration

Edit the anchor positions in the launch file:

```python
'anchors': {
    "1": [0.0, 0.0, 0.0],    # Anchor 1 position (x, y, z)
    "3": [3.0, 0.0, 0.0],    # Anchor 3 position
    "7": [0.0, 3.0, 0.0],    # Anchor 7 position
}
```

### Calibration Parameters

Adjust calibration parameters in the launch file:

```python
'calib_params': [0.001, 0.0]  # [a, b] for linear: distance = a * raw + b
```

## Topics

### Published Topics

- `/uwb/pose` (geometry_msgs/PoseStamped): Calculated position
- `/uwb/ranges` (sensor_msgs/Range): Individual range measurements
- `/uwb/debug` (std_msgs/String): Debug information

### Example Usage

```bash
# View published topics
ros2 topic list

# Monitor position
ros2 topic echo /uwb/pose

# Monitor ranges
ros2 topic echo /uwb/ranges

# Monitor debug info
ros2 topic echo /uwb/debug
```

## Troubleshooting

### Serial Port Issues

1. **Check device permissions:**
   ```bash
   ls -l /dev/ttyUSB*
   sudo chmod 666 /dev/ttyUSB0
   ```

2. **Add user to dialout group:**
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in
   ```

3. **Check if device is connected:**
   ```bash
   dmesg | grep ttyUSB
   ```

### Docker Issues

1. **Device not found:**
   - Ensure the device path is correct in docker-compose.yml
   - Check if the device is accessible: `ls -l /dev/ttyUSB*`

2. **Permission denied:**
   - Run with `--privileged` flag or add device mapping
   - Check Docker daemon permissions

### ROS2 Issues

1. **Node not starting:**
   - Check ROS2 installation: `ros2 --version`
   - Source the workspace: `source install/setup.bash`
   - Check dependencies: `rosdep install --from-paths src --ignore-src -r -y`

2. **No data received:**
   - Verify serial connection and baud rate
   - Check anchor configuration
   - Monitor debug topic for error messages

## Development

### File Structure

```
RTK-DW-main/
├── scripts/
│   └── uwb_beacons_ros2.py    # Main ROS2 node
├── launch/
│   └── uwb_beacons_ros2.launch.py  # Launch file
├── package.xml                   # ROS2 package definition
├── CMakeLists.txt               # Build configuration
├── Dockerfile                   # Docker image definition
├── docker-compose.yml           # Docker Compose configuration
├── build_and_run.sh             # Build/run script
├── test_node.py                 # Test script
├── backup_ros1_files/           # Backup of original ROS1 files
└── uwb_beacons/                 # UWB hardware code
```

### Adding New Features

1. Modify the Python node in `scripts/uwb_beacons_ros2.py`
2. Update launch parameters if needed
3. Rebuild the Docker image: `docker-compose build`
4. Test with: `docker-compose up uwb-beacons`

## License

MIT License - see LICENSE file for details.
