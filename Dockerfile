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
    python3-ament-package \
    build-essential \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install additional Python packages
RUN pip3 install --no-cache-dir \
    pyserial \
    numpy \
    scipy \
    shapely

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
WORKDIR /ros2_ws

# Create src directory for ROS2 workspace
RUN mkdir -p /ros2_ws/src

# Copy application files to workspace
COPY . /ros2_ws/src/uwb_beacons/

# Build the workspace (this will be done at build time)
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /ros2_ws && \
    colcon build --packages-select uwb_beacons"

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Source ROS2 setup\n\
source /opt/ros/humble/setup.bash\n\
\n\
# Build workspace if install directory is empty or missing\n\
if [ ! -f /ros2_ws/install/setup.bash ]; then\n\
    echo "Building ROS2 workspace..."\n\
    cd /ros2_ws && colcon build --packages-select uwb_beacons\n\
fi\n\
\n\
# Source the workspace\n\
if [ -f /ros2_ws/install/setup.bash ]; then\n\
    source /ros2_ws/install/setup.bash\n\
else\n\
    echo "ERROR: Workspace setup.bash not found after build attempt"\n\
    exit 1\n\
fi\n\
\n\
# Execute the command passed to the container\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["ros2", "launch", "uwb_beacons", "uwb_beacons.launch.py"]
