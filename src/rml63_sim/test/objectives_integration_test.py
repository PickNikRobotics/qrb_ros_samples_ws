"""Integration test: the qrb_pick_and_place Objective picks coke1 off the table and carries it to the box.

Runs the full MoveIt Pro Runtime for rml63_sim via the moveit_pro_test_utils fixture. The final pose is not checked
for "upright on the box": Qualcomm's original re_raise target sweeps the open gripper through the released can and
knocks it over about one run in four (a known issue kept on purpose), so only "left the table" is a migration signal.
"""

import math

from moveit_pro_test_utils.objective_test_fixture import (
    ExecuteObjectiveResource,
    execute_objective_resource as execute_objective_resource,
    run_objective,
)

TABLE_TOP_Z = 1.015
SPAWN_XY = (3.0, -2.0)


def test_qrb_pick_and_place_moves_can_off_the_table(
    execute_objective_resource: ExecuteObjectiveResource,
) -> None:
    run_objective("QRB Pick and Place", False, execute_objective_resource, objective_wait_time=180.0)

    pose = execute_objective_resource.lookup_object_pose("mj_world", "coke1_site")
    moved = math.dist((pose.position.x, pose.position.y), SPAWN_XY)
    assert moved > 0.3 or pose.position.z > TABLE_TOP_Z + 0.1, (
        f"coke1 still at the pick spot ({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})"
    )
