#!/usr/bin/env bash
# Checks qrb_remote_assistant_sim with the Simulator, controllers, and Qualcomm's remote assistant pipeline (mapping,
# map save, relocalization, Nav2), no MoveIt Pro Runtime, and prints each stage from the launch log as it happens.
# Runs in a throwaway container from the workspace's user image; see tools/drivers_only_env.sh for the license,
# image, and ROS domain it uses. Usage: run_remote_assistant_drivers_only.sh
set -u
source "$(dirname "$0")/drivers_only_env.sh"
docker run "${DRIVERS_ONLY_ARGS[@]}" -e LAUNCH_ARGS="${LAUNCH_ARGS:-}" -e TIMEOUT_S="${TIMEOUT_S:-900}" \
  -e TASK_WAIT_S="${TASK_WAIT_S:-200}" --entrypoint bash "$IMAGE" -c '
    source /opt/ros/jazzy/setup.bash; source /opt/overlay_ws/install/setup.bash; source $UWS/install/setup.bash
    rm -f /var/tmp/cartographer_ros/map.pbstream
    ros2 launch qrb_remote_assistant_sim sim_drivers_only.launch.py ${LAUNCH_ARGS:-} > /tmp/sim.log 2>&1 &
    # Ground truth and SLAM pose every 2 s, to see where the base is at each stage.
    ( sleep 10; while :; do
        gt=$(timeout 3 ros2 run tf2_ros tf2_echo mj_world base_link_ground_truth 2>&1 | grep -m1 Translation | grep -oE "\[.*\]")
        sl=$(timeout 3 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -m1 Translation | grep -oE "\[.*\]")
        od=$(timeout 3 ros2 topic echo --once /odom nav_msgs/msg/Odometry 2>/dev/null | grep -A3 "    position:" | grep -oE "[xy]: [-0-9.e]+" | tr "\n" " ")
        echo "$SECONDS world=$gt map=$sl odom=$od" >> /tmp/poses.log; sleep 1; done ) &
    ( sleep 10; timeout 120 ros2 topic echo /cmd_vel geometry_msgs/msg/TwistStamped --field twist 2>/dev/null | grep -A1 "^linear:\|^angular:" | grep -E "  x:|  z:" > /tmp/cmd.log ) &
    end=$((SECONDS + ${TIMEOUT_S:-900}))
    while [ $SECONDS -lt $end ]; do
      sleep 5
      if grep -aqE "Relocalization successful, starting navigation|Command [0-9] execution failed|process has died" /tmp/sim.log; then break; fi
    done
    sed "s/\x1b\[[0-9;]*m//g" /tmp/sim.log | grep -aE "Control loop period|Entering (FORWARD|TURN)|Command [0-9] service response|Map saved|Load map response|Start localization response|Validate relocalization response: Relocalization (Successful|failed)|Relocalization successful|starting Nav2 on wall time|process has died|execution failed|Managed nodes are active" | uniq | tail -40
    echo "--- map file"; ls -la /var/tmp/cartographer_ros/ 2>&1 | tail -1
    if grep -aq "Relocalization successful, starting navigation" /tmp/sim.log; then
      echo "--- right after relocalization: world vs SLAM map"
      timeout 5 ros2 run tf2_ros tf2_echo mj_world base_link_ground_truth 2>&1 | grep -m1 Translation
      timeout 5 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -m1 -A1 Translation
      timeout 5 ros2 service call /get_trajectory_states cartographer_ros_msgs/srv/GetTrajectoryStates 2>&1 | grep -oE "trajectory_id=\[[^]]*\], trajectory_state=\[[^]]*\]"
      echo "--- task_manager_node (Qualcomm unchanged): go to office to check person"
      { sleep 3; echo "go to office to check person"; sleep ${TASK_WAIT_S:-200}; } | timeout $(( ${TASK_WAIT_S:-200} + 10 )) ros2 run simulation_remote_assistant task_manager_node 2>&1 \
        | sed "s/\x1b\[[0-9;]*m//g" | grep -aE "Navigating to|Navigation (goal|target|failed|timed)|Arrived|Found|not detected|Task finished" | head -8
      echo "--- base ground truth"; timeout 5 ros2 run tf2_ros tf2_echo mj_world base_link_ground_truth 2>&1 | grep -m1 Translation
      echo "--- base in map (SLAM)"; timeout 5 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -m1 Translation
      echo "--- nav2 log"; sed "s/\x1b\[[0-9;]*m//g" /tmp/sim.log | grep -aE "\[(bt_navigator|controller_server|planner_server|behavior_server)[^]]*\].*(WARN|ERROR|Failed|Aborting|Goal (succeeded|failed|reached)|collision|Running (spin|backup|wait)|failed to|Reached the goal|Passing new path)" | sed -E "s/\[[0-9-]+ [0-9:.]+\]//" | sort | uniq -c | sort -rn | head -14
    fi
    echo "--- poses (s, world truth, SLAM map)"; head -40 /tmp/poses.log
    echo "--- elapsed ${SECONDS}s"
  '
