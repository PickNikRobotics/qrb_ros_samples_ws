"""Start the sample's Cartographer SLAM and Nav2 next to the MoveIt Pro Simulator.

Runs the same nodes the sample's README starts by hand: Qualcomm's two Cartographer nodes with their arguments and
config/cartographer/qrb_amr_mapping_2d.lua, and nav2_bringup's navigation_launch.py with config/nav2/nav2_params.yaml,
Nav2's own defaults. Upstream cartographer_node has no default Lua file, unlike Qualcomm's fork, so this passes the
configuration directory and basename that launch/qrb_2d_lidar_slam.launch.py leaves out, naming the wrapper that
drops the fork-only option from Qualcomm's file.

Nav2's collision monitor publishes the final velocity on /cmd_vel, the topic diff_drive_controller reads. Jazzy's
controller takes only TwistStamped, so Nav2 runs with its own enable_stamped_cmd_vel set rather than through a relay;
that is the one parameter changed from the defaults file, as a launch-time rewrite.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

# MoveIt Pro runs on wall time; the sample's README passes use_sim_time:=true because Gazebo published /clock.
USE_SIM_TIME = False


def generate_launch_description():
    share = Path(get_package_share_directory("qrb_amr_nav_sim"))
    nav2_launch = Path(get_package_share_directory("nav2_bringup")) / "launch" / "navigation_launch.py"
    time_param = {"use_sim_time": USE_SIM_TIME}
    nav2_params = RewrittenYaml(
        source_file=str(share / "config" / "nav2" / "nav2_params.yaml"),
        # Every Nav2 node that publishes a velocity command; each reads the parameter itself and it defaults to false.
        param_rewrites={
            f"{node}.ros__parameters.enable_stamped_cmd_vel": "true"
            for node in ("controller_server", "velocity_smoother", "collision_monitor", "behavior_server", "docking_server")
        },
        convert_types=True,
    )
    return LaunchDescription(
        [
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                output="screen",
                parameters=[time_param],
                arguments=[
                    "-configuration_directory",
                    str(share / "config" / "cartographer"),
                    "-configuration_basename",
                    "qrb_amr_mapping_2d_upstream.lua",
                ],
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="cartographer_occupancy_grid_node",
                output="screen",
                parameters=[time_param],
                arguments=["-resolution", "0.05", "-publish_period_sec", "1.0", "-enable_standard_gridmap"],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(nav2_launch)),
                launch_arguments={
                    "use_sim_time": str(USE_SIM_TIME),
                    "params_file": nav2_params,
                }.items(),
            ),
        ]
    )
