"""Tests for slam_command_shim: /qrb_slam_command served over upstream Cartographer's services.

A fake Cartographer stands in for cartographer_node: it serves get_trajectory_states, finish_trajectory, write_state,
and start_trajectory, records each call, and publishes /constraint_list on demand.
"""

import pathlib
import sys
import threading
import time

import pytest
import rclpy
from cartographer_ros_msgs.msg import StatusCode, StatusResponse, TrajectoryStates
from geometry_msgs.msg import Point
from cartographer_ros_msgs.srv import FinishTrajectory, GetTrajectoryStates, StartTrajectory, WriteState
from qrb_ros_slam_msgs.msg import CommandCode
from qrb_ros_slam_msgs.msg import StatusCode as QrbStatus
from qrb_ros_slam_msgs.srv import SlamCommand
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "scripts"))
from slam_command_shim import SlamCommandShim  # noqa: E402

INTER_DIFFERENT = "Inter constraints, different trajectories"


class FakeCartographer(Node):
    def __init__(self):
        super().__init__("fake_cartographer")
        self.trajectories = {0: TrajectoryStates.ACTIVE}
        self.calls = []
        self.write_ok = True
        self.create_service(GetTrajectoryStates, "get_trajectory_states", self.on_states)
        self.create_service(FinishTrajectory, "finish_trajectory", self.on_finish)
        self.create_service(WriteState, "write_state", self.on_write)
        self.create_service(StartTrajectory, "start_trajectory", self.on_start)
        self.constraints = self.create_publisher(MarkerArray, "constraint_list", 10)

    @staticmethod
    def status(code=StatusCode.OK, message=""):
        return StatusResponse(code=code, message=message)

    def on_states(self, request, response):
        response.status = self.status()
        response.trajectory_states.trajectory_id = list(self.trajectories)
        response.trajectory_states.trajectory_state = list(self.trajectories.values())
        return response

    def on_finish(self, request, response):
        self.calls.append(("finish", request.trajectory_id))
        self.trajectories[request.trajectory_id] = TrajectoryStates.FINISHED
        response.status = self.status()
        return response

    def on_write(self, request, response):
        self.calls.append(("write", request.filename, request.include_unfinished_submaps))
        response.status = self.status() if self.write_ok else self.status(StatusCode.INVALID_ARGUMENT, "bad")
        if self.write_ok:
            pathlib.Path(request.filename).write_bytes(b"pbstream")
        return response

    def on_start(self, request, response):
        self.calls.append(("start", request.configuration_basename))
        new_id = max(self.trajectories) + 1
        self.trajectories[new_id] = TrajectoryStates.ACTIVE
        response.status = self.status()
        response.trajectory_id = new_id
        return response

    def publish_constraint(self, namespace):
        marker = Marker(ns=namespace, type=Marker.LINE_LIST, points=[Point(), Point(x=1.0)])
        self.constraints.publish(MarkerArray(markers=[marker]))


@pytest.fixture
def system(tmp_path):
    rclpy.init()
    fake = FakeCartographer()
    shim = SlamCommandShim(
        map_path=tmp_path / "map.pbstream",
        configuration_directory="/config",
        localization_basename="localization.lua",
        relocalization_timeout_s=1.0,
    )
    client_node = Node("test_client")
    client = client_node.create_client(SlamCommand, "/qrb_slam_command")
    executor = MultiThreadedExecutor()
    for node in (fake, shim, client_node):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    assert client.wait_for_service(timeout_sec=5.0)

    def call(command_id):
        future = client.call_async(SlamCommand.Request(command_id=command_id))
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert future.done(), f"command {command_id} did not respond"
        return future.result().response

    yield fake, call, tmp_path
    executor.shutdown()
    rclpy.shutdown()
    thread.join(timeout=5.0)


def test_start_mapping_succeeds_when_a_trajectory_is_active(system):
    fake, call, _ = system
    response = call(CommandCode.START_MAPPING)
    assert response.result and response.code == QrbStatus.OK


def test_finish_mapping_finishes_the_active_trajectory(system):
    fake, call, _ = system
    response = call(CommandCode.FINISH_MAPPING)
    assert response.result and response.code == QrbStatus.OK
    assert ("finish", 0) in fake.calls


def test_save_map_writes_finished_submaps_to_the_map_path(system):
    fake, call, tmp_path = system
    call(CommandCode.FINISH_MAPPING)
    response = call(CommandCode.SAVE_MAP)
    assert response.result and response.code == QrbStatus.OK
    assert ("write", str(tmp_path / "map.pbstream"), False) in fake.calls


def test_save_map_reports_cartographer_failure(system):
    fake, call, _ = system
    fake.write_ok = False
    response = call(CommandCode.SAVE_MAP)
    assert not response.result
    assert response.code == QrbStatus.INVALID_ARGUMENT


def test_load_map_fails_when_no_map_was_saved(system):
    _, call, _ = system
    response = call(CommandCode.LOAD_MAP)
    assert not response.result
    assert response.code == QrbStatus.NOT_FOUND


def test_load_map_succeeds_once_the_map_exists(system):
    _, call, tmp_path = system
    (tmp_path / "map.pbstream").write_bytes(b"pbstream")
    response = call(CommandCode.LOAD_MAP)
    assert response.result and response.code == QrbStatus.OK


def test_start_localization_starts_a_trajectory_with_the_localization_config(system):
    fake, call, _ = system
    response = call(CommandCode.START_LOCALIZATION)
    assert response.result and response.code == QrbStatus.OK
    assert ("start", "localization.lua") in fake.calls


def test_validate_relocalization_is_in_progress_before_a_cross_trajectory_constraint(system):
    fake, call, _ = system
    call(CommandCode.START_LOCALIZATION)
    response = call(CommandCode.VALIDATE_RELOCALIZATION)
    assert not response.result
    assert "Relocalization Successful" not in response.message


def test_validate_relocalization_succeeds_after_a_cross_trajectory_constraint(system):
    fake, call, _ = system
    call(CommandCode.START_LOCALIZATION)
    fake.publish_constraint("Inter constraints, same trajectory")
    time.sleep(0.2)
    assert not call(CommandCode.VALIDATE_RELOCALIZATION).result
    fake.publish_constraint(INTER_DIFFERENT)
    time.sleep(0.2)
    response = call(CommandCode.VALIDATE_RELOCALIZATION)
    assert response.result and response.code == QrbStatus.OK
    assert "Relocalization Successful" in response.message


def test_validate_relocalization_times_out(system):
    _, call, _ = system
    call(CommandCode.START_LOCALIZATION)
    time.sleep(1.2)
    response = call(CommandCode.VALIDATE_RELOCALIZATION)
    assert not response.result
    assert response.code == QrbStatus.DEADLINE_EXCEEDED


def test_validate_relocalization_before_localization_starts_is_a_precondition_failure(system):
    _, call, _ = system
    response = call(CommandCode.VALIDATE_RELOCALIZATION)
    assert not response.result
    assert response.code == QrbStatus.FAILED_PRECONDITION


def test_unknown_command_is_invalid(system):
    _, call, _ = system
    response = call(42)
    assert not response.result
    assert response.code == QrbStatus.INVALID_ARGUMENT
