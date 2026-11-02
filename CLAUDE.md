# CLAUDE.md

Qualcomm's `qrb_ros_samples`, ported to MoveIt Pro (one robot configuration package per sample), plus the
migration guide written from those ports and a Claude Code skill built on it. This file covers working on
the repository itself. For porting a new application, use the `migrate-to-moveit-pro` skill
(`.claude/skills/migrate-to-moveit-pro/`); for what each port does, see `README.md` and the guide's
Appendix A (`docs/migration_guide.md`).

## Never commit

This repository is public.

- Anything from a private repository, or generated from one.
- Machine-specific code: home directories, a user name, paths into a local `moveit_pro` source checkout
  (its `.env`, its dev containers), or image tags built from one machine's folder names. Take settings from
  the environment or the MoveIt Pro CLI's own configuration.
- The MoveIt Pro Core archive or any other PickNik binary.
- Text that pins the reader to one MoveIt Pro release or describes "the current release". A step that needs
  a feature may state its verified minimum ("requires MoveIt Pro 10.2 or newer").

Before committing, grep the staged files for `/home/`, the user name, and `src/moveit_pro`.

## Generated files

Regenerate these with their generator; never edit them by hand.

| File | Generator |
|---|---|
| `src/rml63_sim/description/rml63.xml` | `tools/postprocess_rml63_mjcf.py` from `description/mjcf/robot_description.xml` |
| `src/qrb_amr_sim/description/qrb_robot_base.xml`, `src/qrb_amr_nav_sim/description/qrb_robot_base_mini.xml` | `tools/postprocess_qrb_amr_mjcf.py`, then `tools/add_rangefinder_lidar.py` for the mini's lidar |
| `src/qrb_remote_assistant_sim/description/qrb_robot_base_mini.xml` | the mini MJCF above, through `tools/add_base_camera.py` |
| `src/*/description/mjcf/robot_description.urdf` | The converter's input: `xacro` of the package's `.urdf.xacro`, plus the `<mujoco>` block (guide Stage 3) |
| `src/qrb_pick_place_core/config/rml63.urdf`, `src/qrb_pick_place_core/test/fixtures/rml63.urdf` | `xacro` of `src/rml63_sim/description/rml63.urdf.xacro` |
| `src/qrb_amr_nav_sim/description/ionic.xml`, `src/qrb_remote_assistant_sim/description/office_walls.xml` | `tools/gen_sdf_collision_mjcf.py` |
| `src/qrb_remote_assistant_sim/description/office_furniture.xml` | `tools/gen_sdf_includes_mjcf.py` |
| `src/rml63_sim/objectives/qrb_load_scene.xml`, `src/qrb_amr_sim/objectives/qrb_amr_load_scene.xml` | `tools/gen_scene_objective.py`, from the compiled `scene.xml`; regenerate after changing the scene |
| `src/rml63_sim/waypoints/` | `tools/gen_waypoints.py` |

Each generator's docstring gives its usage. Confirm a generator still reproduces the committed file before
changing it.

## Vendored packages

`src/lunar_sim_behaviors`, `src/moveit_pro_sam3`, `src/simulation_remote_assistant`, and
`src/qrb_ros_slam_msgs` are copies of other repositories. Keep their files unchanged; record any change made
around them (`COLCON_IGNORE`, `colcon.pkg`, skipped `rosdep` keys) and their source commit in their
`VENDORED.md`.

## Ports keep the original's behavior

Each port keeps Qualcomm's sequence, targets, and parameters. Do not fix a sample's defects; document them in
the guide's Appendix A. Change a value only when it breaks the original's own sequence in the Simulator, and
list that change there too.

## The guide, the skill, and the README are linked

- `.claude/skills/migrate-to-moveit-pro/reference.md` is a symlink to `docs/migration_guide.md`. Edit the
  guide; never replace the symlink with a copy.
- The skill refers to the guide's stages and sections by name, and the README links to the guide's anchors.
  When you rename a guide heading, update both, and check that every internal link and every repository path
  the guide cites still resolves.
- When a port changes, update the guide's worked-example notes and its Appendix A entry for that sample.

## Writing conventions for `README.md`, `docs/`, and package READMEs (not vendored packages)

- One line per paragraph and per list item; GitHub renders them, and hard wraps show up as line breaks.
- Follow the MoveIt Pro tutorials' style (https://docs.picknik.ai/tutorials/quick_start_intro/): "we"
  narrates, "you" instructs; Title Case headings; checkpoints phrased "You should see…"; UI labels in bold,
  Objective names in backticks.
- Callouts use GitHub alerts (`> [!NOTE]`, `> [!TIP]`, `> [!IMPORTANT]`, `> [!WARNING]`) with a bold
  title. Sample-specific detail goes in a `> [!NOTE]` titled **Worked example:**, not in the general steps.
- Cite paths relative to the repository root, or name the other repository they belong to.

## Building and checking

Use the MoveIt Pro CLI, not hand-configured containers: the CLI supplies the license, networking, DDS, and
GPU settings that ad hoc `docker run` commands get wrong.

```bash
moveit_pro configure -w <path to this repository> -c <package>
moveit_pro build user_workspace      # moveit_pro build user_image after changing a package.xml dependency
moveit_pro test -c <package>         # the colcon tests of every package that registers them
```

The offline MuJoCo checks (the `test/` scripts that import `mujoco` rather than `rclpy`) run from
`moveit_pro shell`, with no Runtime; each one's docstring gives its usage. For a change to a sample's
behavior, also run that sample's Objective in the Desktop App, as Appendix A of the guide describes.
