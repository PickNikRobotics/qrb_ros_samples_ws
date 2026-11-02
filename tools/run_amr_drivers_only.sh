#!/usr/bin/env bash
# Checks qrb_amr_sim with the Simulator and controllers only (no MoveIt Pro Runtime), driven by Qualcomm's
# unmodified simple_motion node. Runs in a throwaway container from the workspace's user image; see
# tools/drivers_only_env.sh for the license, image, and ROS domain it uses.
set -u
source "$(dirname "$0")/drivers_only_env.sh"
docker run "${DRIVERS_ONLY_ARGS[@]}" --entrypoint bash "$IMAGE" -c '
    source /opt/ros/jazzy/setup.bash; source /opt/overlay_ws/install/setup.bash; source $UWS/install/setup.bash
    ros2 launch qrb_amr_sim sim_drivers_only.launch.py > /tmp/sim.log 2>&1 &
    sleep 15
    echo "--- controllers"; timeout 20 ros2 control list_controllers 2>&1 | sed "s/\x1b\[[0-9;]*m//g"
    echo "--- topics"; timeout 20 ros2 topic list | grep -E "cmd_vel|odom"
    echo "--- simple_motion"; timeout 60 bash $UWS/src/qrb_amr_sim/test/drive_simple_motion.sh
    echo "--- sim log errors"; grep -aE "\[ERROR\]|\[error\]" /tmp/sim.log | grep -viE "Error retrieving|mesh_operations" | head -8
  '
