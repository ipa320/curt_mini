# syntax=docker/dockerfile:1
FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
      build-essential ca-certificates cmake git \
      libbluetooth-dev libusb-1.0-0-dev \
      python3-colcon-common-extensions python3-rosdep python3-vcstool \
      ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/ws
COPY docker/dependencies.repos /tmp/dependencies.repos
RUN mkdir -p src && vcs import --recursive src < /tmp/dependencies.repos

# Build this checkout, including its updated package-relative robot meshes.
COPY curt_mini_bringup/ src/curt_mini/curt_mini_bringup/
COPY curt_mini_description/ src/curt_mini/curt_mini_description/
COPY curt_mini_teleop/ src/curt_mini/curt_mini_teleop/
COPY ipa_ros2_control/ src/curt_mini/ipa_ros2_control/

RUN apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths src --ignore-src \
      --rosdistro jazzy -r -y \
      --dependency-types build \
      --dependency-types build_export \
      --dependency-types buildtool \
      --dependency-types buildtool_export \
      --dependency-types exec \
    && rm -rf /var/lib/apt/lists/*

# curt_mini_bringup launches openzen_driver but does not declare it as a package
# dependency, so request it explicitly alongside the base bringup package.
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --merge-install \
      --packages-up-to curt_mini_bringup openzen_driver \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# The entrypoint and interactive `docker compose exec ... bash` both source
# this bashrc. ROS_DOMAIN_ID comes from Compose and is not overridden here.
RUN printf '%s\n' \
      'source /opt/ros/jazzy/setup.bash' \
      'source /opt/ws/install/setup.bash' \
      > /root/.bashrc
COPY docker/entrypoint.sh /opt/curt_mini/entrypoint.sh
RUN chmod 0755 /opt/curt_mini/entrypoint.sh

ENTRYPOINT ["/opt/curt_mini/entrypoint.sh"]
CMD ["ros2", "launch", "curt_mini_bringup", "robot_base.launch.py"]
