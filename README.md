# qrb_ros_samples_ws

Qualcomm's [`qrb_ros_samples`](https://github.com/qualcomm-qrb-ros/qrb_ros_samples) robotics samples, ported from Gazebo to MoveIt Pro and its MuJoCo-based Simulator. Each sample is a MoveIt Pro robot configuration package that keeps the original's sequence, targets, and parameters, so a diff against the original shows only what the migration needed.

**[Migration guide](docs/migration_guide.md):** how these ports were done, step by step, with every error and gotcha hit along the way. It is written for porting any ROS 2 application to MoveIt Pro, with or without MoveIt 2.

## Ported samples

| Sample | Package | What runs on MoveIt Pro |
|---|---|---|
| `simulation_sample_pick_and_place` | [`rml63_sim`](src/rml63_sim), [`qrb_pick_place_core`](src/qrb_pick_place_core) | The RML-63 arm picks a can and places it on a box. The original's nine `MoveGroupInterface` steps run as the **QRB Pick and Place** Objective, and the original node is ported to plan with MoveIt Pro's in-process planner. |
| `simulation_sample_amr_simple_motion` | [`qrb_amr_sim`](src/qrb_amr_sim) | The QRB robot base on `diff_drive_controller` in the warehouse, driven from the Desktop App's teleoperation in place of the keyboard node. |
| `simulation_amr_navigation` (with `simulation_2d_lidar_slam`) | [`qrb_amr_nav_sim`](src/qrb_amr_nav_sim) | The QRB robot base mini maps the Ionic world with Cartographer and drives to goals clicked in the Desktop App through Nav2. |
| `simulation_remote_assistant` | [`qrb_remote_assistant_sim`](src/qrb_remote_assistant_sim) | Qualcomm's three remote-assistant nodes, unchanged: scripted mapping of the office world, map save and relocalization, Nav2 to a named location, and a person check with SAM3 standing in for the NPU-only YOLO model. |

The packages' own READMEs cover each port's details, including how to switch to Qualcomm's own stack (Cartographer fork, QNN YOLO) on Dragonwing hardware.

## Running a sample

Requires the [MoveIt Pro](https://docs.picknik.ai/) CLI, version 10.2 or newer, and a license.

```bash
moveit_pro configure -w <path to this repository> -c rml63_sim   # or qrb_amr_sim, qrb_amr_nav_sim, qrb_remote_assistant_sim
moveit_pro build user_workspace
moveit_pro run
```

The guide's [worked examples appendix](docs/migration_guide.md#appendix-a-the-worked-examples) lists what to run in the Desktop App for each sample. `qrb_amr_nav_sim` and `qrb_remote_assistant_sim` need Cartographer and Nav2 in the user image: run `moveit_pro build user_image` first.

## Claude Code skill

[`.claude/skills/migrate-to-moveit-pro`](.claude/skills/migrate-to-moveit-pro/SKILL.md) packages the guide as a [Claude Code](https://claude.com/claude-code) skill for planning and carrying out a migration. Claude Code picks it up when started in this repository. Its `reference.md` is a link to the guide, so to use the skill elsewhere, copy `SKILL.md` and `docs/migration_guide.md` (renamed `reference.md`) into `~/.claude/skills/migrate-to-moveit-pro/`.

## Licenses

Vendored packages keep their own licenses; see each one's `VENDORED.md`. The guide's [asset licenses](docs/migration_guide.md#asset-licenses) table lists the meshes and models the scenes use.
