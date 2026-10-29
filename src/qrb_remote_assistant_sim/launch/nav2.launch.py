"""Nav2 for qrb_remote_assistant_sim: nav2_bringup's navigation_launch.py with Nav2's own default parameters, on wall
time with stamped velocity commands.

MoveIt Pro publishes no /clock, so the sample's use_sim_time:=true would leave Nav2 at time 0, and Jazzy's
diff_drive_controller takes only TwistStamped. Qualcomm's nav_preparation_node starts Nav2 after relocalization;
scripts/nav2_on_wall_time/ros2 turns that command into this launch, so Nav2 starts at the same point in the
sequence. Nav2 must not run during mapping: its collision monitor publishes on /cmd_vel and would zero the mapping
drive's commands.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav_share = Path(get_package_share_directory("qrb_amr_nav_sim"))
    nav2_launch = Path(get_package_share_directory("nav2_bringup")) / "launch" / "navigation_launch.py"
    params = RewrittenYaml(
        source_file=str(nav_share / "config" / "nav2" / "nav2_params.yaml"),
        # Every Nav2 node that publishes a velocity command; each reads the parameter itself and it defaults to false.
        param_rewrites={
            f"{node}.ros__parameters.enable_stamped_cmd_vel": "true"
            for node in ("controller_server", "velocity_smoother", "collision_monitor", "behavior_server", "docking_server")
        },
        convert_types=True,
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(nav2_launch)),
                launch_arguments={"use_sim_time": "False", "params_file": params}.items(),
            )
        ]
    )
