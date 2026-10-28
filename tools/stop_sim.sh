#!/usr/bin/env bash
# Stop sim_drivers_only.launch.py inside the dev container. The container shares the host PID namespace, so
# match anchored executable paths only; a bare "ros2 launch rml63_sim" pattern also matches the calling shell.
pattern="^(/opt/ros/jazzy/lib/controller_manager/(ros2_control_node|spawner)|/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)"
pkill -f "$pattern"
sleep 3
if pgrep -f "$pattern" >/dev/null; then
  echo "sim still running" >&2
  exit 1
fi
