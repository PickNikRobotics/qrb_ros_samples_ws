#!/usr/bin/env bash
# Task 5 end-to-end: drivers-only sim in the dev container, Core node in the Core-only container, same ROS domain.
# Usage: tools/run_core_e2e.sh [--kill-sim-at <seconds>]
# Exit: 0 only if the Core node exited cleanly (ros2 launch itself always exits 0) and, without --kill-sim-at,
# coke1 ends on the box.
cd /home/kiswatz/src/moveit_pro || exit 1
KILL_AT=""
[ "${1:-}" = "--kill-sim-at" ] && KILL_AT=$2
LICENSE=$(grep ^MOVEIT_LICENSE_KEY .env | cut -d= -f2)
DOMAIN=$(docker compose exec -T dev-nvidia printenv ROS_DOMAIN_ID | tr -d '\r')
RMW=$(docker compose exec -T dev-nvidia printenv RMW_IMPLEMENTATION | tr -d '\r')
dev() { docker compose exec -T -u kiswatz dev-nvidia bash -c "source /etc/skel/.moveit-pro-bashrc && source ~/user_ws/install/setup.bash && $1"; }

dev "~/user_ws/tools/stop_sim.sh"
dev "nohup ros2 launch rml63_sim sim_drivers_only.launch.py > /tmp/sim_launch.log 2>&1 &"
sleep 20
dev "python3 ~/user_ws/src/rml63_sim/test/coke_tracker.py home" || { dev "~/user_ws/tools/stop_sim.sh"; exit 1; }

if [ -n "$KILL_AT" ]; then
  ( sleep "$KILL_AT"; dev "~/user_ws/tools/stop_sim.sh" ) &
fi
docker run --rm --network host -e MOVEIT_LICENSE_KEY="$LICENSE" -e ROS_DOMAIN_ID="$DOMAIN" \
  -e RMW_IMPLEMENTATION="${RMW:-rmw_fastrtps_cpp}" \
  -v /home/kiswatz/src/qrb_moveit_pro_ws/docker/cyclonedds_loopback.xml:/cyclonedds.xml:ro \
  -e CYCLONEDDS_URI=file:///cyclonedds.xml qrb-core-only:10.2.0 \
  bash -c '. /opt/ros/jazzy/setup.sh && . /ws/install/setup.sh && \
    (ros2 launch qrb_pick_place_core pick_place_core.launch.py > /tmp/node.log 2>&1; echo $? > /tmp/rc) & \
    sleep 8; echo "--- nodes during run:"; ros2 node list; wait; \
    grep -vE "Error retrieving|mesh_operations" /tmp/node.log | sed "s/\x1b\[[0-9;]*m//g" | tail -40; \
    grep -q "qrb_ros_arm_pick_place-1\]: process has finished cleanly" /tmp/node.log'
NODE_RC=$?
wait
if [ -z "$KILL_AT" ]; then
  dev "python3 ~/user_ws/src/rml63_sim/test/coke_tracker.py placed" || NODE_RC=1
  dev "~/user_ws/tools/stop_sim.sh"
fi
echo "node exit code: $NODE_RC"
exit $NODE_RC
