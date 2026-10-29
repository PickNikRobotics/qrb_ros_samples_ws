#!/usr/bin/env python3
"""Serve Qualcomm's /qrb_slam_command over upstream Cartographer, so their remote assistant nodes run unchanged.

Qualcomm's Cartographer fork serves qrb_ros_slam_msgs/srv/SlamCommand itself. Upstream Cartographer offers the same
operations as separate services; this node maps each command onto them:

  0 START_MAPPING            a trajectory is already active: mapping launches with one
  1 FINISH_MAPPING           finish_trajectory on the active trajectory
  2 SAVE_MAP                 write_state to map_path, finished submaps only
  3 LOAD_MAP                 map_path exists; the localization launch starts cartographer_node with
                             -load_state_filename on it, which loads it as a frozen trajectory
  4 START_LOCALIZATION       start_trajectory with the localization configuration
  5 VALIDATE_RELOCALIZATION  succeeds once /constraint_list holds a constraint between different trajectories:
                             the new trajectory has matched the loaded map. Fails with DEADLINE_EXCEEDED after
                             relocalization_timeout_s, so a caller that polls it cannot spin forever.

On a host with Qualcomm's fork, do not run this node: the fork serves the same service.
"""

import pathlib
import time

import rclpy
from cartographer_ros_msgs.msg import StatusCode as CartoStatus
from cartographer_ros_msgs.msg import TrajectoryStates
from cartographer_ros_msgs.srv import FinishTrajectory, GetTrajectoryStates, StartTrajectory, WriteState
from qrb_ros_slam_msgs.msg import CommandCode, CommandResponse
from qrb_ros_slam_msgs.msg import StatusCode as QrbStatus
from qrb_ros_slam_msgs.srv import SlamCommand
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

# cartographer_ros labels a constraint between two trajectories with this marker namespace.
CROSS_TRAJECTORY_CONSTRAINTS = "Inter constraints, different trajectories"
SERVICE_TIMEOUT_S = 10.0


class SlamCommandShim(Node):
    def __init__(self, map_path, configuration_directory, localization_basename, relocalization_timeout_s):
        super().__init__("qrb_slam_command_shim")
        self._map_path = pathlib.Path(map_path)
        self._configuration_directory = str(configuration_directory)
        self._localization_basename = localization_basename
        self._relocalization_timeout_s = relocalization_timeout_s
        self._localization_started_at = None
        self._relocalized = False
        clients = ReentrantCallbackGroup()
        self._states = self.create_client(GetTrajectoryStates, "get_trajectory_states", callback_group=clients)
        self._finish = self.create_client(FinishTrajectory, "finish_trajectory", callback_group=clients)
        self._write = self.create_client(WriteState, "write_state", callback_group=clients)
        self._start = self.create_client(StartTrajectory, "start_trajectory", callback_group=clients)
        self.create_subscription(MarkerArray, "constraint_list", self._on_constraints, 10, callback_group=clients)
        self.create_service(
            SlamCommand, "/qrb_slam_command", self._on_command, callback_group=MutuallyExclusiveCallbackGroup()
        )

    def _on_constraints(self, msg):
        if self._localization_started_at is not None and any(
            m.ns == CROSS_TRAJECTORY_CONSTRAINTS and m.points for m in msg.markers
        ):
            self._relocalized = True

    def _call(self, client, request):
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S):
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + SERVICE_TIMEOUT_S
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.result() if future.done() else None

    @staticmethod
    def _response(code, message):
        return CommandResponse(result=code == QrbStatus.OK, code=code, message=message)

    def _carto_response(self, reply, what):
        if reply is None:
            return self._response(QrbStatus.UNAVAILABLE, f"{what}: Cartographer did not answer")
        ok = reply.status.code == CartoStatus.OK
        return self._response(reply.status.code, f"{what}: {'done' if ok else reply.status.message}")

    def _active_trajectory(self):
        reply = self._call(self._states, GetTrajectoryStates.Request())
        if reply is None:
            return None
        states = reply.trajectory_states
        active = [t for t, s in zip(states.trajectory_id, states.trajectory_state) if s == TrajectoryStates.ACTIVE]
        return max(active) if active else -1

    def _on_command(self, request, response):
        command = request.command_id
        if command == CommandCode.START_MAPPING:
            active = self._active_trajectory()
            if active is None:
                response.response = self._response(QrbStatus.UNAVAILABLE, "Start mapping: Cartographer did not answer")
            elif active < 0:
                response.response = self._response(QrbStatus.FAILED_PRECONDITION, "Start mapping: no active trajectory")
            else:
                response.response = self._response(QrbStatus.OK, f"Start mapping: trajectory {active} is mapping")
        elif command == CommandCode.FINISH_MAPPING:
            active = self._active_trajectory()
            if active is None or active < 0:
                response.response = self._response(QrbStatus.FAILED_PRECONDITION, "Finish mapping: no active trajectory")
            else:
                reply = self._call(self._finish, FinishTrajectory.Request(trajectory_id=active))
                response.response = self._carto_response(reply, "Finish mapping")
        elif command == CommandCode.SAVE_MAP:
            self._map_path.parent.mkdir(parents=True, exist_ok=True)
            reply = self._call(
                self._write, WriteState.Request(filename=str(self._map_path), include_unfinished_submaps=False)
            )
            response.response = self._carto_response(reply, f"Save map to {self._map_path}")
        elif command == CommandCode.LOAD_MAP:
            if self._map_path.is_file():
                response.response = self._response(QrbStatus.OK, f"Load map: {self._map_path}")
            else:
                response.response = self._response(QrbStatus.NOT_FOUND, f"Load map: {self._map_path} does not exist")
        elif command == CommandCode.START_LOCALIZATION:
            reply = self._call(
                self._start,
                StartTrajectory.Request(
                    configuration_directory=self._configuration_directory,
                    configuration_basename=self._localization_basename,
                ),
            )
            response.response = self._carto_response(reply, "Start localization")
            if response.response.result:
                self._localization_started_at = time.monotonic()
                self._relocalized = False
        elif command == CommandCode.VALIDATE_RELOCALIZATION:
            response.response = self._validate_relocalization()
        else:
            response.response = self._response(QrbStatus.INVALID_ARGUMENT, f"Unknown command {command}")
        return response

    def _validate_relocalization(self):
        if self._localization_started_at is None:
            return self._response(QrbStatus.FAILED_PRECONDITION, "Relocalization: localization has not started")
        if self._relocalized:
            return self._response(QrbStatus.OK, "Relocalization Successful")
        if time.monotonic() - self._localization_started_at > self._relocalization_timeout_s:
            return self._response(
                QrbStatus.DEADLINE_EXCEEDED,
                f"Relocalization failed: no match to the loaded map within {self._relocalization_timeout_s:.0f} s",
            )
        # result is false and code OK: Qualcomm's nav_preparation_node keeps spinning and asking until it succeeds.
        return CommandResponse(result=False, code=QrbStatus.OK, message="Relocalization in progress")


def main():
    rclpy.init()
    params = Node("qrb_slam_command_shim_params")
    map_path = params.declare_parameter("map_path", "/var/tmp/cartographer_ros/map.pbstream").value
    configuration_directory = params.declare_parameter("configuration_directory", "").value
    localization_basename = params.declare_parameter("localization_basename", "").value
    timeout = params.declare_parameter("relocalization_timeout_s", 60.0).value
    params.destroy_node()
    if not configuration_directory or not localization_basename:
        raise SystemExit("qrb_slam_command_shim needs configuration_directory and localization_basename parameters")
    node = SlamCommandShim(map_path, configuration_directory, localization_basename, timeout)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
