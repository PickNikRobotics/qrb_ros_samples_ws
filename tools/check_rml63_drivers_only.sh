#!/usr/bin/env bash
# Checks the rml63_sim scene and controllers, run inside the dev container: the offline grasp check in Python MuJoCo,
# then Qualcomm's joint targets driven straight to the controllers in the live drivers-only sim (no move_group), with
# the can tracked until it lands on the box.
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
