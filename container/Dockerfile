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
# / nav_msgs / urdf. The only missing piece for the shared zenoh RMW domain is
# rmw_zenoh_cpp (verified apt-installable as ros-humble-rmw-zenoh-cpp 0.1.8).
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        ros-humble-rmw-zenoh-cpp \
        python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# Entrypoint: source ROS, bring up the zenoh router, start robot_state_publisher
# with the Lite3 URDF, then stay alive so primitives can docker exec in.
COPY entrypoint.sh /lite3_entrypoint.sh
RUN chmod +x /lite3_entrypoint.sh

# The URDF + this package are bind-mounted from the host (compose volumes), so
# the image carries no robot model — a URDF edit on the host takes effect on the
# next container start without rebuild.
ENTRYPOINT ["/lite3_entrypoint.sh"]
CMD []