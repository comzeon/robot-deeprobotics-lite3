# SPDX-License-Identifier: MulanPSL-2.0
# robonix_lite3_ros container — the ROS 2 home for the Lite3 primitives.
#
# The host is a bare L4T board (no ROS 2); every ROS 2 process — the
# chassis/camera drivers AND robot_state_publisher — runs inside this
# container and is `docker exec`'d from `rbnx boot` (ADR-0004). It mirrors the
# Webots sim container pattern, but launches robot_state_publisher directly
# with our URDF instead of a Webots controller.
#
# Built once by container/start.sh before `rbnx boot`.
ARG ROS_BASE_IMAGE=robonix-ros:humble-ros-base
FROM ${ROS_BASE_IMAGE}

# The base image carries rclpy / tf2_ros / robot_state_publisher / sensor_msgs
# / nav_msgs / urdf. Missing pieces for this deploy:
#   - rmw_zenoh_cpp (verified apt-installable as ros-humble-rmw-zenoh-cpp 0.1.8)
#   - the Orbbec SDK ROS 2 wrapper (orbbec_camera) for the Gemini 335
#   - numpy (the camera + lidar drivers convert images / point clouds with it)
# Both apt pieces are baked into the IMAGE so they survive container recreation —
# installing them via `docker exec` at runtime would only touch the container's
# writable layer and be lost on `compose down`/recreate.
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        ros-humble-rmw-zenoh-cpp \
        ros-humble-orbbec-camera \
        ros-humble-orbbec-description \
        ros-humble-pcl-ros \
        ros-humble-pcl-conversions \
        libpcl-dev \
        python3-yaml \
        python3-numpy \
        git \
        cmake \
        build-essential \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# ── Livox SDK2 + livox_ros_driver2 (MID-360S lidar) ─────────────────────────
# Not an apt package — built from source (colcon). livox_ros_driver2 v1.2.6+
# supports MID-360S; it requires Livox-SDK2 >= v1.3.1. Both are baked into the
# image so the lidar stack survives container recreation. The ROS2 driver build
# (./build.sh humble) cd's up two levels, so the source MUST live at
# /livox_ws/src/livox_ros_driver2 (colcon workspace root = /livox_ws).
#
# The sources are COPY'd from container/vendor/ — downloaded on the build host
# (the Docker build VM cannot reach github.com:443 from CN networks). See
# container/vendor/README.md for the exact pinned versions to fetch:
#   Livox-SDK2        v1.3.1   → vendor/Livox-SDK2
#   livox_ros_driver2 1.2.6    → vendor/livox_ros_driver2
COPY vendor/Livox-SDK2 /tmp/Livox-SDK2
RUN cd /tmp/Livox-SDK2 \
    && mkdir -p build && cd build \
    && cmake .. && make -j"$(nproc)" && make install \
    && rm -rf /tmp/Livox-SDK2

RUN mkdir -p /livox_ws/src
COPY vendor/livox_ros_driver2 /livox_ws/src/livox_ros_driver2
# livox_ros_driver2's build.sh does NOT source ROS itself; it expects the
# caller to. Without `source /opt/ros/humble/setup.bash` here, colcon/CMake
# cannot find PCL (its CMake config lives in the ROS CMAKE_PREFIX_PATH), so the
# build "fails" silently (build.sh has no `set -e` → returns 0 anyway). The
# `cp` after build.sh restores the package.xml that build.sh removed, so
# `ros2 pkg` can discover livox_ros_driver2 in the install tree.
RUN bash -lc 'source /opt/ros/humble/setup.bash \
    && cd /livox_ws/src/livox_ros_driver2 \
    && bash build.sh humble \
    && cp -f package_ROS2.xml /livox_ws/install/livox_ros_driver2/share/livox_ros_driver2/package.xml' \
    && rm -rf /livox_ws/build /livox_ws/log

# Entrypoint: source ROS, bring up the zenoh router, start robot_state_publisher
# with the Lite3 URDF, then stay alive so primitives can docker exec in.
COPY entrypoint.sh /lite3_entrypoint.sh
RUN chmod +x /lite3_entrypoint.sh

# The URDF + this package are bind-mounted from the host (compose volumes), so
# the image carries no robot model — a URDF edit on the host takes effect on the
# next container start without rebuild.
ENTRYPOINT ["/lite3_entrypoint.sh"]
CMD []