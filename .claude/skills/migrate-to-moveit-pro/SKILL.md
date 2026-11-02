---
name: migrate-to-moveit-pro
description: Use when porting an existing ROS 2 robotics application or sample (MoveIt 2, mobile base or Nav2, teleoperation node, vendor arm SDK, perception or ML pipeline, Gazebo world) onto MoveIt Pro, or when planning, sizing, or ranking such a migration — including codebases that never used MoveIt 2.
---

# Migrate a ROS 2 Application to MoveIt Pro

## Overview

A migration is done when the application's original behavior runs on MoveIt Pro, in the MoveIt Pro Simulator, as an Objective a user can run from the Desktop App — and the guide records every step that took. The goal is proving the migration, not improving the original: keep its sequence, targets, and parameters unless the user asks otherwise, and write down defects instead of fixing them.

`reference.md` in this directory is the migration guide: section-by-section technique, error messages, and the worked ports (pick and place, two mobile bases, and the remote assistant pipeline). Read the sections the triage below sends you to before acting.

## Workflow

1. **Triage** (guide §1). Read every `package.xml`, launch file, and README run step in the sample and in any repository its README launches (simulation repos especially; spawn poses, worlds, and controllers often live there). Classify into the guide's triage rows. Record the result before planning.
2. **Find the closest MoveIt Pro reference config** in `moveit_pro_example_ws` and the MoveIt Pro docs for each row (arm: `lab_sim`, `factory_sim`; mobile base: `lunar_sim` (diff drive), `hangar_sim` (Nav2, SLAM); AprilTag: `april_tag_sim`). Check the docs' support tables for the exact variant (drive type, sensor type) and say so if no reference exists — that is the main sizing risk. The reference configs live in [`moveit_pro_example_ws`](https://github.com/PickNikRobotics/moveit_pro_example_ws). If the machine has no clone of it, ask the user to clone it (or to name where theirs is) before relying on a reference config; do not assume any MoveIt Pro source checkout exists. A clone can lag the release: compare it against `origin/main` (or `gh api repos/PickNikRobotics/moveit_pro_example_ws/contents/src`) before concluding a reference config is missing, and check out the release branch that matches the installed CLI (`moveit_pro --version`; list the branches with `git branch -r`).
3. **Check the environment before building anything** (guide §9): which MoveIt Pro CLI major is installed versus the images present, whether the dev container builds, whether Docker Hub is reachable, which config the user currently has set. Read the active workspace and config package from the `moveit_pro` CLI itself (`moveit_pro --version`, `moveit_pro run -l`), not by opening its config files. Changing the CLI, `.env`, or stopping a running stack needs the user's go-ahead.
4. **Agree on the plan with the user** before building: present it in the shape below, settle open decisions, then write the task plan. Order the work so the simulation scene comes first: everything else is validated against it.
5. **Build in this order**, validating each layer alone before the next:
   - MuJoCo scene, checked offline in Python MuJoCo (guide §3) — including driving the original targets through the actuators.
   - Robot configuration package (guide §4); before writing Objectives, drive the original commands straight to the controllers from a Runtime shell (`moveit_pro shell`).
   - The ported node or Behaviors (guide §2, §6), or for a mobile base the drive controller and teleoperation (guide §7). Where the original's own nodes should run unchanged, the shims at their interfaces (guide §8).
   - The Objective (guide §5), then make the scene visible in the Desktop App (guide §3, "Making the scene visible"; for a mobile base also the 3D Visualizer's reference frame, guide §7).
6. **Hand the user a running Objective or teleoperation session in the Desktop App** and ask them to watch it, or drive it. They will see problems a CLI check cannot (missing scene, robot not drawn in the 3D view, wrong geometry, unclear controls).
7. **Update `reference.md`** (and any copy of the guide the user keeps) with every new step, error, and gotcha, citing files relative to a named repository root.

## What a migration plan must contain

A plan you present for a sample is these parts, in order:

1. **Triage** — the rows that apply and the files that prove it.
2. **Reference mapping** — for each part of the sample, the MoveIt Pro config, docs page, or Behavior that hosts it, and whether a reference exists for this exact variant.
3. **Hidden configuration** — what the original sets outside its own package (launch arguments, simulation repos, OS image packages) and where it moves.
4. **Validation layers** — what proves each layer works before the next one is built.
5. **Desktop App visibility** — how the user will see the robot's world and the task's progress, not just the robot.
6. **Known-defect policy** — the original's behavior is kept; list anything already suspected.
7. **Environment preconditions** — CLI version, images, network, current user config.

## Quick reference

| Symptom | Section |
|---|---|
| `No acceleration limit was defined for joint` | §2 Acceleration limits |
| State monitor times out; `/joint_states` never arrives | §2 own executor; §9 `docker exec` bashrc |
| Node dies with SIGABRT on a startup error | §2 `std::jthread` |
| `Cannot locate rosdep definition for [pro_rrt]` | §2 Building |
| MuJoCo bodies "do not sit where the URDF places" links | §3 robot world pose |
| `Non-PNG texture` | §3 Scene |
| Desktop App shows only the robot | §3 Making the scene visible |
| `Field required` / `got '()' of type tuple` in `config.yaml` | §4 |
| `Can't find a tree with name` | §5 rebuild after adding an Objective |
| JTAC `sensor_frames must be specified` | §6 |
| `CollisionObject … already exists` | §3 `overwrite="true"` |
| `x509: certificate signed by unknown authority` pulling images | §9 |
| Base visible in the scene camera but not the 3D Visualizer | §7 Frames |
| Base ignores `cmd_vel`, or stops after half a second | §7 Drive controller |
| `free joint can only be used on top level` | §3 mobile base post-processing |
| `number of faces should be between 1 and 200000` | §3 decimate meshes |
| `Rejecting /do_teleoperate goal: no UI tab is subscribed` | §7 Teleoperation |
| Base shakes or flickers in the 3D Visualizer | §7 Frames (two TF parents) |
| Every lidar beam reads a few centimeters | §3 Planar lidar |
| Simulation runs slower than real time with a lidar | §3 Planar lidar |
| `no decoder found for mesh file` (`.glb`) | §3 Scene |
| `Key '…' was used the wrong number of times` (Cartographer) | §7 Navigation |
| Nav2 runs but the base never moves | §7 Navigation (`enable_stamped_cmd_vel`) |
| `no rosdep rule` for a ROS package | §9 |
| Original node sits at time 0 / waits forever (`use_sim_time:=true`) | §8 subprocesses; §9 no `/clock` |
| Vendor-only service, message package, or NPU model | §8 |
| Scripted route drifts or turns overshoot; odometry agrees with the command | §7 Drive controller; §9 simulation speed |
| Two message types on `/cmd_vel`, or the base moves at half speed | §7 Navigation (shared `/cmd_vel`) |
| Simulator slow with cameras on the CPU image | §3 Cameras on the CPU image |

## Common mistakes

- **Planning from the sample's own files only.** The robot's pose, world, sensors, and drive plugin often live in a separate simulation repository or an OS image. Read every launch the README runs.
- **Porting a bespoke teleop node.** A keyboard or joystick node that only publishes velocity commands is replaced by the Desktop App's teleoperation, not by Objectives. Keep the original node runnable against the simulation to prove the controller interface, and configure teleoperation (guide §7).
- **Assuming the platform supports the variant.** Check the support table for the exact drive type or sensor; a missing reference configuration is the biggest sizing risk and must be stated up front.
- **Skipping offline validation.** Testing grasp reach or drive kinematics in Python MuJoCo takes seconds; finding the same problem through the Runtime takes much longer.
- **Declaring done from the CLI.** The user must see it work in the Desktop App with the scene visible.
- **Fixing the original's demo.** Keep its steps and values; document defects in the guide's known-issues section.
- **Editing the original's parameter files.** Copy SLAM, Nav2, and controller parameter files unchanged and make each needed change in the launch file (a wrapper or a launch-time rewrite), so the port can be diffed against the original and switched back to a vendor fork.
- **Machine-specific paths in the guide.** Cite paths relative to a named repository root; commands take placeholders for checkout locations.
- **Version numbers in the guide.** Write "the current release" and link the release notes; the guide outlives any one version.
- **Editing the original's nodes.** When they should run on the vendor's hardware later, vendor them unchanged and put shims, relays, and wrappers at their interfaces (guide §8); record every change around them in `VENDORED.md`.
- **Reimplementing the original's success check.** If the original has a node that decides "done" (a task manager waiting for a detection), run it as the end-to-end check.
- **Vendoring code with no license.** Keep it local and say so; do not push it until the owner confirms terms.
- **Telling readers to write tests.** Validation scripts are for developing the port; the guide describes migration steps, not test authoring.
