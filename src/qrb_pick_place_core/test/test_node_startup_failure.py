"""The ported node must fail cleanly, exit code 1 with an error naming the missing server, when no controllers run.

Regression test for a startup throw that destroyed a joinable std::thread and aborted (SIGABRT, exit -6).
Run where nothing publishes /joint_states or serves the controller actions (an isolated ROS domain).
"""

import os
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory


def test_exits_one_without_a_simulator():
    exe = Path(get_package_prefix("qrb_pick_place_core")) / "lib/qrb_pick_place_core/qrb_ros_arm_pick_place"
    config = Path(get_package_share_directory("qrb_pick_place_core")) / "config"
    params = Path(os.environ.get("TMPDIR", "/tmp")) / "qrb_startup_failure_params.yaml"
    import yaml

    params.write_text(
        yaml.safe_dump(
            {
                "qrb_ros_arm_pick_place": {
                    "ros__parameters": {
                        "robot_description": (config / "rml63.urdf").read_text(),
                        "robot_description_semantic": (config / "rml_63.srdf").read_text(),
                    }
                }
            }
        )
    )
    env = dict(os.environ, ROS_DOMAIN_ID="231", ROS_LOCALHOST_ONLY="1")
    result = subprocess.run(
        [str(exe), "--ros-args", "--params-file", str(params)], env=env, capture_output=True, text=True, timeout=60
    )
    output = result.stdout + result.stderr
    assert result.returncode == 1, f"exit {result.returncode}\n{output[-800:]}"
    assert "/joint_states" in output
