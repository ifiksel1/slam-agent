FROM ros:humble

# Install MAVROS and MAVROS Extras
RUN apt-get update && apt-get install -y \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib datasets (required for MAVROS GPS conversions)
RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# Create workspace directory
RUN mkdir -p /ws/launch

# Copy vision bridge script and launch file
COPY vision_bridge.py /ws/vision_bridge.py
COPY mavros_bridge_launch.py /ws/launch/mavros_bridge_launch.py

# Make vision bridge executable
RUN chmod +x /ws/vision_bridge.py

# Set ROS 2 environment variables
ENV ROS_DOMAIN_ID=1
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Working directory
WORKDIR /ws

# Default command (will be overridden by docker-compose)
CMD ["bash"]
