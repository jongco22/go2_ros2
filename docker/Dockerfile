# FROM robotis/ros:jazzy-ros-base-torch2.7.0-cuda12.8.0
FROM dustynv/ros:humble-desktop-l4t-r36.4.0

RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg


ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV COLCON_CURRENT_RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=100

# Install basic requirements and ROS packages (rviz2 is already included in desktop image)
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    python3-pip \
    portaudio19-dev \
    mesa-utils \
    libgl1-mesa-dri \
    software-properties-common \
    ffmpeg \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-ament-lint-auto \
    ros-${ROS_DISTRO}-ament-lint-common \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-visualization-msgs \
    ros-${ROS_DISTRO}-rosbag2 \
    ros-${ROS_DISTRO}-rosbag2-storage-default-plugins \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    ros-${ROS_DISTRO}-rosidl-generator-dds-idl \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-cmake-python \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Remove conflicting custom Jetson OpenCV packages and install standard Ubuntu OpenCV
# This prevents package conflicts with ROS packages that depend on libopencv-dev
RUN dpkg --purge --force-all opencv-dev opencv-libs opencv-licenses opencv-main opencv-python opencv-scripts || true

# Install OpenCV-dependent ROS packages (now that conflicts are resolved)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libopencv-dev \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-rosbag2-cpp \
    && rm -rf /var/lib/apt/lists/*

ENV PIP_INDEX_URL=https://pypi.jetson-ai-lab.io/jp6/cu126/+simple
ENV PIP_EXTRA_INDEX_URL=https://pypi.org/simple

# Install specific numpy version first
RUN pip install 'numpy<2.0.0'

# Create workspace
ENV COLCON_WS=/root/ros2_ws
WORKDIR ${COLCON_WS}

# Clone the repository with submodules
COPY . /${COLCON_WS}/src

# Initialize git submodules (required for aioice and other dependencies)
# If .git directory exists, update submodules; otherwise skip
RUN cd src && \
    (git submodule update --init --recursive 2>/dev/null || \
     echo "Git submodules not available, assuming dependencies are included") && \
    cd ..

# Install additional Python dependencies required by go2_driver_node
# These were identified during TF troubleshooting (see cursor_tf.md)
# Use standard PyPI as primary index to avoid timeout issues
# - aiortc: WebRTC connection support
# - wasmtime/wasmtime-py: Runtime dependency
RUN pip install --no-cache-dir \
    --index-url https://pypi.org/simple \
    --extra-index-url https://pypi.jetson-ai-lab.io/jp6/cu126/+simple \
    aiortc \
    wasmtime || pip install --no-cache-dir \
    --index-url https://pypi.org/simple \
    wasmtime-py

# Install Python requirements (with numpy constraint)
RUN cd src && pip install -r requirements.txt

# Build the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Set environment variables
ENV ROBOT_IP="192.168.123.161"
ENV CONN_TYPE="cyclonedds"
ENV WEBRTC_SERVER_HOST="0.0.0.0"
ENV WEBRTC_SERVER_PORT="9991"

# CMD ["ros2", "launch", "go2_robot_sdk", "robot.launch.py"]

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc  && \
    echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc  && \
    echo "alias cb='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'" >> ~/.bashrc
