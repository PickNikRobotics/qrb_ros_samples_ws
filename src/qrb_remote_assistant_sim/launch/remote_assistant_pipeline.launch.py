"""Qualcomm's remote assistant pipeline (simulation_remote_assistant scripts/map_nav_setup.py) for the MoveIt Pro
Simulator, with their three nodes unchanged.

map_nav_setup.py, in order: Cartographer for mapping, then build_map_node (drives a scripted route and saves the map
through /qrb_slam_command), then Cartographer again, then nav_preparation_node (loads the map, relocalizes, starts
Nav2). This runs the same sequence on upstream Cartographer, with changes only around their nodes:

- slam_backend:=upstream starts slam_command_shim, which serves /qrb_slam_command; on the fork it serves itself.
- The second Cartographer loads the saved map with -load_state_filename and no default trajectory, so
  START_LOCALIZATION adds the localization trajectory, as the fork's LOAD_MAP and START_LOCALIZATION do.
- nav_preparation_node still starts Nav2 itself after relocalization, but its `ros2 launch nav2_bringup ...` runs
  through scripts/nav2_on_wall_time/ros2, which runs launch/nav2.launch.py: wall time (MoveIt Pro publishes no
  /clock, so the sample's use_sim_time:=true would leave Nav2 at time 0) and stamped cmd_vel.
- build_map_node and nav_preparation_node publish Twist on /cmd_vel; it is remapped to /qrb_twist_cmd_vel and stamped
  for the drive controller, which takes TwistStamped only and is also fed by Nav2. Each Twist is sent once: both
  nodes publish every control tick.

Run task_manager_node (Qualcomm's interactive prompt) yourself once "Relocalization successful" is logged, or run the
QRB Remote Assistant Objective. qrb_amr_nav_sim's README covers switching Cartographer to Qualcomm's fork.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.events.process import ShutdownProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

USE_SIM_TIME = False
MAP_PATH = "/var/tmp/cartographer_ros/map.pbstream"
TWIST_TOPIC = "/qrb_twist_cmd_vel"


def generate_launch_description():
    nav_share = Path(get_package_share_directory("qrb_amr_nav_sim"))
    share = Path(get_package_share_directory("qrb_remote_assistant_sim"))
    carto_config = str(nav_share / "config" / "cartographer")
    upstream = IfCondition(PythonExpression(["'", LaunchConfiguration("slam_backend"), "' == 'upstream'"]))
    time_param = {"use_sim_time": USE_SIM_TIME}

    def cartographer(extra_arguments):
        return Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            output="screen",
            parameters=[time_param],
            arguments=["-configuration_directory", carto_config] + extra_arguments,
        )

    mapping = cartographer(["-configuration_basename", "qrb_amr_mapping_2d_upstream.lua"])
    occupancy = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[time_param],
        arguments=["-resolution", "0.05", "-publish_period_sec", "1.0", "-enable_standard_gridmap"],
    )
    build_map = Node(
        package="simulation_remote_assistant",
        executable="build_map_node",
        output="screen",
        parameters=[{"control_period": LaunchConfiguration("control_period")}],
        remappings=[("/cmd_vel", TWIST_TOPIC)],
    )
    localization = cartographer(
        [
            "-configuration_basename",
            "qrb_amr_localization_2d_upstream.lua",
            "-load_state_filename",
            MAP_PATH,
            "-start_trajectory_with_default_topics=false",
        ],
    )
    wrapper_dir = str(share / "scripts" / "nav2_on_wall_time")
    nav_preparation = Node(
        package="simulation_remote_assistant",
        executable="nav_preparation_node",
        output="screen",
        remappings=[("/cmd_vel", TWIST_TOPIC)],
        additional_env={"PATH": wrapper_dir + os.pathsep + os.environ.get("PATH", "")},
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("slam_backend", default_value="upstream", description="upstream or qualcomm"),
            DeclareLaunchArgument("detector", default_value="sam3", description="sam3 or qualcomm"),
            DeclareLaunchArgument(
                "control_period",
                default_value="0.05",
                description="build_map_node control period. Qualcomm's default is 0.2; their README lowers it to 0.05 when "
                "turns overshoot and the robot hits walls, which it does here at the scene's 0.74x real time.",
            ),
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            Node(
                package="qrb_amr_sim",
                executable="twist_stamper.py",
                name="qrb_twist_stamper",
                output="screen",
                parameters=[{"input_topic": TWIST_TOPIC, "output_topic": "/cmd_vel", "republish_rate": 0.0}],
            ),
            Node(
                package="qrb_remote_assistant_sim",
                executable="slam_command_shim.py",
                output="screen",
                condition=upstream,
                parameters=[
                    {
                        "map_path": MAP_PATH,
                        "configuration_directory": carto_config,
                        "localization_basename": "qrb_amr_localization_2d_upstream.lua",
                    }
                ],
            ),
            mapping,
            occupancy,
            build_map,
            # Publishes the QRB Detect Person Objective's count as the Detection2DArray Qualcomm's YOLO pipeline
            # would; on the IQ-9 launch their yolo_detectcion.launch.py instead.
            Node(
                package="qrb_remote_assistant_sim",
                executable="detection_bridge.py",
                output="screen",
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration("detector"), "' == 'sam3'"])),
            ),
            # map_nav_setup.py: when build_map_node exits, stop the mapping Cartographer, start it again (here on the
            # saved map), wait 5 s, then start nav_preparation_node.
            RegisterEventHandler(
                OnProcessExit(
                    target_action=build_map,
                    on_exit=[EmitEvent(event=ShutdownProcess(process_matcher=lambda action: action is mapping))],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=mapping,
                    on_exit=[localization, TimerAction(period=5.0, actions=[nav_preparation])],
                )
            ),
        ]
    )
