#!/usr/bin/env bash
# Layer-2 gate for qrb_amr_nav_sim: simulator, controllers, Cartographer, and Nav2 only (no MoveIt Pro Runtime), then
# one /navigate_to_pose goal. Runs in a throwaway container from the workspace's Runtime image.
# Usage: run_nav_drivers_only.sh [goal_x goal_y]
set -u
WS=$(cd "$(dirname "$0")/.." && pwd)
IMAGE=${IMAGE:-moveit-pro-runtime:10.2.0-jazzy-qrb_moveit_pro_ws}
LICENSE=$(grep ^MOVEIT_LICENSE_KEY "$HOME/src/moveit_pro/.env" | cut -d= -f2)
# The install space records the path it was built at inside the Runtime container, so mount the workspace there.
UWS=/home/$(id -un)/user_ws
docker run --rm --network host -u "$(id -u):$(id -g)" -e HOME=/tmp -e MOVEIT_LICENSE_KEY="$LICENSE" \
  -e ROS_DOMAIN_ID=77 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e MUJOCO_GL=egl \
  -v "$WS/docker/cyclonedds_loopback.xml:/cyclonedds.xml:ro" -e CYCLONEDDS_URI=file:///cyclonedds.xml \
  -v "$WS:$UWS" -e UWS="$UWS" -e GX="${1:-2.0}" -e GY="${2:-0.0}" --entrypoint bash "$IMAGE" -c '
    source /opt/ros/jazzy/setup.bash; source /opt/overlay_ws/install/setup.bash; source $UWS/install/setup.bash
    ros2 launch qrb_amr_nav_sim sim_drivers_only.launch.py > /tmp/sim.log 2>&1 &
    sleep 25
    echo "--- controllers"; timeout 20 ros2 control list_controllers 2>&1 | sed "s/\x1b\[[0-9;]*m//g"
    echo "--- scan rate"; timeout 8 ros2 topic hz /scan 2>&1 | grep -m2 average
    echo "--- base_footprint TF parents"; timeout 8 ros2 topic echo /tf 2>/dev/null | grep -B1 -A1 "child_frame_id: base_footprint" | grep -E "  frame_id" | sort | uniq -c
    echo "--- nav check"; timeout 200 python3 $UWS/src/qrb_amr_nav_sim/test/nav_check.py $GX $GY
    echo "--- sim log errors"; grep -aE "\[ERROR\]|\[error\]|\[FATAL\]" /tmp/sim.log | grep -viE "Error retrieving|mesh_operations" | sort | uniq -c | sort -rn | head -12
    echo "--- lifecycle"; grep -aE "Managed nodes are active|Failed to bring up|Activating" /tmp/sim.log | tail -3
  '
