# Build context: ros2_ws/. No ROS 1 source, credentials, or runtime logs copied.
ARG ROS_BASE_IMAGE=ros:jazzy-ros-base
FROM ${ROS_BASE_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    git build-essential python3-colcon-common-extensions python3-smbus2 \
    python3-smbus python3-serial python3-pytest \
    ros-jazzy-tf2-ros ros-jazzy-slam-toolbox ros-jazzy-rosbridge-suite \
    ros-jazzy-v4l2-camera ros-jazzy-image-transport-plugins \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/armpi_ros2
# Pin official LD06 driver, not a moving branch.
RUN git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git src/ldlidar_stl_ros2 \
    && cd src/ldlidar_stl_ros2 && git checkout bf668a89baf722a787dadc442860dcbf33a82f5a
# The pinned upstream header comments out pthread.h; newer compilers reject it.
COPY patches patches
RUN cd src/ldlidar_stl_ros2 \
    && git apply --check /opt/armpi_ros2/patches/ldlidar_jazzy_pthread.patch \
    && git apply /opt/armpi_ros2/patches/ldlidar_jazzy_pthread.patch
COPY src/armpi_ros2 src/armpi_ros2
ENV CMAKE_BUILD_PARALLEL_LEVEL=1
ENV MAKEFLAGS=-j1
RUN source /opt/ros/jazzy/setup.bash && colcon build --executor sequential \
    && colcon test --executor sequential --packages-select armpi_ros2 --python-testing pytest --event-handlers console_direct+ \
    && colcon test-result --verbose
ENV ROS_DOMAIN_ID=42
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
CMD ["bash", "-c", "source /opt/armpi_ros2/install/setup.bash && exec ros2 launch armpi_ros2 base.launch.py"]
