#!/usr/bin/env bash
# Layer-2 gate for qrb_amr_sim: simulator + controllers only (no MoveIt Pro Runtime), driven by Qualcomm's
# unmodified simple_motion node. Runs in a throwaway container from the workspace's Runtime image.
set -u
WS=$(cd "$(dirname "$0")/.." && pwd)
IMAGE=${IMAGE:-moveit-pro-runtime:10.1.1-jazzy-qrb_moveit_pro_ws}
LICENSE=$(grep ^MOVEIT_LICENSE_KEY "$HOME/src/moveit_pro/.env" | cut -d= -f2)
# The install space records the path it was built at inside the Runtime container, so mount the workspace there.
UWS=/home/$(id -un)/user_ws
docker run --rm --network host -u "$(id -u):$(id -g)" -e HOME=/tmp -e MOVEIT_LICENSE_KEY="$LICENSE" \
  -e ROS_DOMAIN_ID=77 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e MUJOCO_GL=egl \
  -v "$WS/docker/cyclonedds_loopback.xml:/cyclonedds.xml:ro" -e CYCLONEDDS_URI=file:///cyclonedds.xml \
  -v "$WS:$UWS" -e UWS="$UWS" --entrypoint bash "$IMAGE" -c '
    source /opt/ros/jazzy/setup.bash; source /opt/overlay_ws/install/setup.bash; source $UWS/install/setup.bash
    ros2 launch qrb_amr_sim sim_drivers_only.launch.py > /tmp/sim.log 2>&1 &
    sleep 15
    echo "--- controllers"; timeout 20 ros2 control list_controllers 2>&1 | sed "s/\x1b\[[0-9;]*m//g"
    echo "--- topics"; timeout 20 ros2 topic list | grep -E "cmd_vel|odom"
    echo "--- simple_motion"; timeout 60 bash $UWS/src/qrb_amr_sim/test/drive_simple_motion.sh
    echo "--- sim log errors"; grep -aE "\[ERROR\]|\[error\]" /tmp/sim.log | grep -viE "Error retrieving|mesh_operations" | head -8
  '
