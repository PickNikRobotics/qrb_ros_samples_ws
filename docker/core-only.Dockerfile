# Clean ROS 2 Jazzy plus only the released MoveIt Pro Core packages, with no Runtime installed.
# Build context: ~/src/qrb_moveit_pro_ws. Expects docker/core-10.2.0/debs/*.deb (unzipped Core archive).
FROM ros:jazzy

COPY docker/core-10.2.0/debs /tmp/core-debs
RUN apt-get update \
    && apt-get install -y --no-install-recommends /tmp/core-debs/*.deb ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/* /tmp/core-debs

# Fail the build if any MoveIt 2 or MoveIt Pro Runtime package slipped in.
RUN ! dpkg-query -W -f="${Package}\n" | grep -xE "ros-jazzy-moveit-core|ros-jazzy-moveit-ros-.*|ros-jazzy-moveit-studio-agent|ros-jazzy-moveit-pro-behavior"

COPY src/qrb_pick_place_core /ws/src/qrb_pick_place_core
WORKDIR /ws
RUN apt-get update \
    && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
       --skip-keys "moveit_pro_base moveit_pro_planning_scene_monitor pro_rrt" \
    && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/jazzy/setup.sh && colcon build --packages-select qrb_pick_place_core
