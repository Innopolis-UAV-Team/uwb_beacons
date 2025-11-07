# Use ROS2 Humble as base image
FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=0

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-numpy \
    python3-scipy \
    python3-serial \
    python3-yaml \
    python3-setuptools \
    build-essential \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install additional Python packages
RUN pip3 install --no-cache-dir \
    pyserial \
    numpy \
    scipy \
    shapely \
    Localization

# Install ROS2 Python packages and build tools
RUN apt-get update && apt-get install -y \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-rclpy \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
WORKDIR /app

# Create directories
RUN mkdir -p /app/config /app/launch /app/uwb_beacons

# Copy application files
COPY scripts/uwb_beacons_ros2.py /app/uwb_beacons/
COPY scripts/algoritms.py /app/uwb_beacons/
COPY scripts/serial_messages.py /app/uwb_beacons/
COPY config/uwb_beacons_params.yaml /app/config/
COPY launch/uwb_beacons_ros2.launch.py /app/launch/
COPY package.xml /app/
COPY CMakeLists.txt /app/

# Create __init__.py for Python package
RUN touch /app/uwb_beacons/__init__.py

# Make the script executable
RUN chmod +x /app/uwb_beacons/uwb_beacons_ros2.py

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Source ROS2 setup\n\
source /opt/ros/humble/setup.bash\n\
\n\
# Build the workspace if needed\n\
if [ ! -f /app/install/setup.bash ]; then\n\
    cd /app\n\
    colcon build --packages-select uwb_beacons\n\
fi\n\
\n\
# Source the workspace\n\
source /app/install/setup.bash\n\
\n\
# Execute the command passed to the container\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["ros2", "launch", "uwb_beacons", "uwb_beacons_ros2.launch.py"]
