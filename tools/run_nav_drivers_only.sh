#!/usr/bin/env bash
# Checks qrb_amr_nav_sim with the Simulator, controllers, Cartographer, and Nav2 only (no MoveIt Pro Runtime), then
# sends one /navigate_to_pose goal. Runs in a throwaway container from the workspace's user image; see
# tools/drivers_only_env.sh for the license, image, and ROS domain it uses.
# Usage: run_nav_drivers_only.sh [goal_x goal_y]
set -u
source "$(dirname "$0")/drivers_only_env.sh"
docker run "${DRIVERS_ONLY_ARGS[@]}" -e GX="${1:-2.0}" -e GY="${2:-0.0}" --entrypoint bash "$IMAGE" -c '
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
