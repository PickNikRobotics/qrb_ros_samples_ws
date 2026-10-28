#!/usr/bin/env bash
# Task 3 gate, run inside the dev container: offline grasp gate, then the live drivers-only sim with no move_group.
set -u
source /etc/skel/.moveit-pro-bashrc
cd ~/user_ws && colcon build --packages-select rml63_sim >/dev/null && source install/setup.bash
~/user_ws/tools/stop_sim.sh
python3 src/rml63_sim/test/grasp_gate.py src/rml63_sim/description/scene.xml --full || exit 1
ros2 launch rml63_sim sim_drivers_only.launch.py > /tmp/sim_launch.log 2>&1 &
sleep 20
if ros2 node list | grep -q move_group; then echo "FAIL: move_group present"; ~/user_ws/tools/stop_sim.sh; exit 1; fi
cd src/rml63_sim/test && timeout 80 python3 drive_sequence.py full && timeout 15 python3 coke_tracker.py placed
rc=$?
~/user_ws/tools/stop_sim.sh
exit $rc
