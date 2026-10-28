"""Integration test: the qrb_pick_and_place Objective moves coke1 onto the cardboard box.

Runs the full MoveIt Pro Runtime for rml63_sim via the moveit_pro_test_utils fixture.
"""

import math

from moveit_pro_test_utils.objective_test_fixture import (
    ExecuteObjectiveResource,
    execute_objective_resource as execute_objective_resource,
    run_objective,
)

# Same acceptance as test/coke_tracker.py "placed": upright on the box top, at the PLACE release point.
BOX_TOP_Z = 1.32
PLACE_XY = (3.08, -2.52)


def test_qrb_pick_and_place_moves_can_to_box(
    execute_objective_resource: ExecuteObjectiveResource,
) -> None:
    run_objective("QRB Pick and Place", False, execute_objective_resource, objective_wait_time=180.0)

    pose = execute_objective_resource.lookup_object_pose("mj_world", "coke1_site")
    q = pose.orientation
    up = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    xy_error = math.dist((pose.position.x, pose.position.y), PLACE_XY)
    assert abs(pose.position.z - BOX_TOP_Z) < 0.01, f"coke1 z {pose.position.z:.3f}, expected {BOX_TOP_Z}"
    assert up > 0.95, f"coke1 tipped over (up {up:.2f})"
    assert xy_error < 0.05, f"coke1 {xy_error:.3f} m from the place point"
