#!/usr/bin/env bash
# End-to-end check of the ported pick-and-place node on MoveIt Pro Core alone: the rml63_sim drivers-only sim in a
# container from the workspace's user image (tools/drivers_only_env.sh), and the node in the Core-only image
# (docker/core-only.Dockerfile), on the same ROS domain.
# Usage: tools/run_core_e2e.sh [--kill-sim-at <seconds>]
#   CORE_IMAGE  the Core-only image; default qrb-core-only, built with
#               docker build -f docker/core-only.Dockerfile -t qrb-core-only .
# Exit: 0 only if the Core node exited cleanly (ros2 launch itself always exits 0) and, without --kill-sim-at,
# coke1 ends on the box.
set -u
source "$(dirname "$0")/drivers_only_env.sh"
CORE_IMAGE=${CORE_IMAGE:-qrb-core-only}
KILL_AT=""
[ "${1:-}" = "--kill-sim-at" ] && KILL_AT=$2
SIM=qrb-core-e2e-sim-$$
ROS_SETUP='source /opt/ros/jazzy/setup.bash && source /opt/overlay_ws/install/setup.bash && source $UWS/install/setup.bash'
sim() { docker exec "$SIM" bash -c "$ROS_SETUP && $1"; }
stop_sim() { docker stop -t 5 "$SIM" >/dev/null 2>&1; }
trap stop_sim EXIT

docker run -d --name "$SIM" "${DRIVERS_ONLY_ARGS[@]}" --entrypoint bash "$IMAGE" \
  -c "$ROS_SETUP && exec ros2 launch rml63_sim sim_drivers_only.launch.py" >/dev/null || exit 1
sleep 20
sim "python3 \$UWS/src/rml63_sim/test/coke_tracker.py home" || exit 1

if [ -n "$KILL_AT" ]; then
  ( sleep "$KILL_AT"; stop_sim ) &
fi
docker run --rm --network host -e MOVEIT_LICENSE_KEY="$MOVEIT_LICENSE_KEY" -e ROS_DOMAIN_ID="${SIDE_ROS_DOMAIN_ID:-77}" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v "$WS/docker/cyclonedds_loopback.xml:/cyclonedds.xml:ro" -e CYCLONEDDS_URI=file:///cyclonedds.xml "$CORE_IMAGE" \
  bash -c '. /opt/ros/jazzy/setup.sh && . /ws/install/setup.sh && \
    (ros2 launch qrb_pick_place_core pick_place_core.launch.py > /tmp/node.log 2>&1; echo $? > /tmp/rc) & \
    sleep 8; echo "--- nodes during run:"; ros2 node list; wait; \
    grep -vE "Error retrieving|mesh_operations" /tmp/node.log | sed "s/\x1b\[[0-9;]*m//g" | tail -40; \
    grep -q "qrb_ros_arm_pick_place-1\]: process has finished cleanly" /tmp/node.log'
NODE_RC=$?
wait
if [ -z "$KILL_AT" ]; then
  sim "python3 \$UWS/src/rml63_sim/test/coke_tracker.py moved" || NODE_RC=1
fi
echo "node exit code: $NODE_RC"
exit $NODE_RC
