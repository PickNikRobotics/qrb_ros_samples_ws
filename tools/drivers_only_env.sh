# Shared setup for the drivers-only checks in tools/: each runs the Simulator and controllers for one sample in a
# throwaway container from this workspace's user image, with no MoveIt Pro Runtime. Source it from a tools/ script:
#
#   source "$(dirname "$0")/drivers_only_env.sh"
#   docker run "${DRIVERS_ONLY_ARGS[@]}" <extra docker args> --entrypoint bash "$IMAGE" -c '...'
#
# Environment overrides:
#   MOVEIT_LICENSE_KEY  the MoveIt Pro license; default: the one `moveit_pro configure` stored in the CLI's config
#   IMAGE               the user image to run; default: the newest one `moveit_pro build user_image` built for this
#                       workspace (tagged <release>-<distro>-<workspace directory name>)
#   SIDE_ROS_DOMAIN_ID  the ROS domain the check runs on, kept apart from a running MoveIt Pro stack; default 77

WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
MOVEIT_PRO_CLI_CONFIG=${MOVEIT_PRO_CLI_CONFIG:-$HOME/.config/moveit_pro/moveit_pro_config.yaml}

if [ -z "${MOVEIT_LICENSE_KEY:-}" ] && [ -f "$MOVEIT_PRO_CLI_CONFIG" ]; then
  MOVEIT_LICENSE_KEY=$(sed -n 's/^MOVEIT_LICENSE_KEY: *//p' "$MOVEIT_PRO_CLI_CONFIG" | tr -d "\"'")
fi
if [ -z "${MOVEIT_LICENSE_KEY:-}" ]; then
  echo "No MoveIt Pro license: set MOVEIT_LICENSE_KEY, or store one with moveit_pro configure." >&2
  exit 1
fi

if [ -z "${IMAGE:-}" ]; then
  IMAGE=$(docker images --format '{{.Repository}}:{{.Tag}}' moveit-pro-runtime | grep -- "-$(basename "$WS")\$" | head -n 1)
fi
if [ -z "$IMAGE" ]; then
  echo "No user image for $(basename "$WS"): run moveit_pro build user_image, or set IMAGE." >&2
  exit 1
fi

# The install space records the path it was built at: the Runtime mounts the workspace at /home/<user>/user_ws, so
# the check mounts it there too.
UWS=/home/$(id -un)/user_ws

DRIVERS_ONLY_ARGS=(
  --rm --network host -u "$(id -u):$(id -g)" -e HOME=/tmp -e MOVEIT_LICENSE_KEY="$MOVEIT_LICENSE_KEY"
  -e ROS_DOMAIN_ID="${SIDE_ROS_DOMAIN_ID:-77}" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e MUJOCO_GL=egl
  -v "$WS/docker/cyclonedds_loopback.xml:/cyclonedds.xml:ro" -e CYCLONEDDS_URI=file:///cyclonedds.xml
  -v "$WS:$UWS" -e UWS="$UWS"
)
