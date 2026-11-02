# Migrating a ROS 2 Application to MoveIt Pro

A guide for moving an existing ROS 2 robotics application onto MoveIt Pro. It was written while porting four of Qualcomm's [`qrb_ros_samples`](https://github.com/qualcomm-qrb-ros/qrb_ros_samples) (`simulation_sample_pick_and_place`, an arm on MoveIt 2; `simulation_sample_amr_simple_motion`, a differential-drive base with no MoveIt; `simulation_amr_navigation`, the same kind of base mapping with Cartographer and driving with Nav2; and `simulation_remote_assistant`, a scripted map-save-relocalize-navigate pipeline that ends in a person check), and every step, rename, and gotcha below was hit on those ports. The ported code is this repository (robot configuration packages `rml63_sim`, `qrb_amr_sim`, `qrb_amr_nav_sim`, and `qrb_remote_assistant_sim`; the ported node `qrb_pick_place_core`; and the vendored `lunar_sim_behaviors`, `moveit_pro_sam3`, `simulation_remote_assistant`, and `qrb_ros_slam_msgs`), and its file references show where each pattern lives in the worked example. Paths below are relative to the repository root unless they name another repository (`qrb_ros_samples`, `qrb_ros_simulation`, `moveit_pro_example_ws`).

Not every application uses MoveIt 2, and it does not need to in order to benefit from MoveIt Pro. Start with [Triage](#1-triage-what-does-the-application-use), which routes each kind of application to the sections it needs.

## Contents

1. [Triage: what does the application use?](#1-triage-what-does-the-application-use)
2. [Migrating MoveIt 2 code to MoveIt Pro](#2-migrating-moveit-2-code-to-moveit-pro)
3. [Migrating the simulation environment to MuJoCo](#3-migrating-the-simulation-environment-to-mujoco)
4. [Creating the robot configuration package](#4-creating-the-robot-configuration-package)
5. [Migrating bespoke sequencing code to Behavior Trees](#5-migrating-bespoke-sequencing-code-to-behavior-trees)
6. [Moving to MoveIt Pro's controllers](#6-moving-to-moveit-pros-controllers)
7. [Mobile bases](#7-mobile-bases)
8. [Running the original's nodes unchanged](#8-running-the-originals-nodes-unchanged)
9. [Environment and tooling gotchas](#9-environment-and-tooling-gotchas)
10. [Known issues carried over from the original](#10-known-issues-carried-over-from-the-original)
11. [Reference: the ported samples](#11-reference-the-ported-samples)

---

## 1. Triage: what does the application use?

Before porting anything, read every `package.xml` and launch file in the application and sort it into the rows below. A codebase usually spans several.

```bash
# From the application root: list the dependencies that decide the migration path.
grep -rhoE "<(exec_|build_)?depend>[a-z0-9_]+" --include=package.xml . | sed 's/.*>//' | sort | uniq -c | sort -rn
grep -rhoE "\b(MoveGroupInterface|move_group|moveit_configs_utils|nav2_[a-z_]+|cartographer|slam_toolbox|apriltag_ros|cmd_vel|gz_ros2_control|ros_gz_[a-z]+)\b" .
```

| The application uses | What moves to MoveIt Pro | Sections |
|---|---|---|
| MoveIt 2 (`MoveGroupInterface`, `move_group`, MoveIt Setup Assistant config, OMPL, Pilz, Servo, MTC) | Planning calls, robot configuration, execution | 2, 3, 4, 5, 6 |
| An arm driven another way (vendor SDK, raw `JointTrajectory`, hand-written IK) | The arm becomes a MoveIt Pro robot: `ros2_control` driver, URDF/SRDF, Objectives replace the hand-written sequence | 3, 4, 5, 6 |
| A bespoke teleoperation node (keyboard, joystick to `cmd_vel` or jog commands) | Nothing to port: the Desktop App's teleoperation replaces it. Keep the original node runnable against the simulation to prove the controller interface matches | 7 |
| A mobile base (Nav2, `cmd_vel`, SLAM, Cartographer) | The base becomes a `ros2_control` drive controller in the MoveIt Pro Simulator; teleoperation moves to the Desktop App; navigation runs inside MoveIt Pro | 3, 4, 7; see the MoveIt Pro docs under **Mobile Navigation** |
| Perception or ML inference (camera nodes, ONNX/QNN models, AprilTag) | Becomes Behaviors in an Objective; MoveIt Pro ships detection, segmentation, AprilTag, and NPU inference Behaviors. A model that runs only on the vendor's NPU gets a stand-in behind the same topic | 4, 5, 8; see **Configure Qualcomm Dragonwing** for the NPU |
| Its own nodes the port should not touch (vendor scripts, orchestration that launches subprocesses) | Run them unchanged; put shims at their interfaces | 8 |
| Gazebo or another simulator | Rebuilt as a MoveIt Pro Simulator (MuJoCo) scene | 3 |

**What the platform adds, even without MoveIt 2:** the Desktop App (run, edit, and monitor Objectives; 3D view; teleoperation), the Behavior Tree runtime and its Behavior library, the MoveIt Pro Simulator, real-time controllers, and a single robot configuration package that both simulation and hardware use. An application with no arm still gains all of these.

### Survey of `qrb_ros_samples`

For reference, this is how the Qualcomm samples triaged (dependencies from each `package.xml`, stacks from source and launch files):

| Sample | Uses | Path |
|---|---|---|
| `robotics/simulation_sample_pick_and_place` | MoveIt 2 `MoveGroupInterface`, OMPL, Gazebo | MoveIt 2 row. **Done; this guide.** |
| `robotics/sample_robot_arm_grasping_with_pose_estimation` | Vendor RM arm API (no MoveIt), DenseFusion + YOLO via ONNX | Arm-without-MoveIt and perception rows |
| `robotics/simulation_amr_navigation` | Nav2 (`NavigateToPose`), Gazebo; runs on top of `simulation_2d_lidar_slam` | Mobile base row. **Done; section 7, Navigation.** |
| `robotics/simulation_2d_lidar_slam` | Cartographer (Qualcomm's fork), Gazebo, keyboard teleop | Mobile base row. **Done, as part of `simulation_amr_navigation`.** |
| `robotics/simulation_remote_assistant` | Qualcomm's Cartographer fork (`SlamCommand` service), Nav2 (`NavigateToPose`), QNN YOLO, Gazebo | Mobile base, perception, and unchanged-nodes rows. **Done; sections 7 and 8.** |
| `robotics/simulation_sample_amr_simple_motion` | Keyboard teleop publishing raw `cmd_vel`, Gazebo diff-drive plugin | Mobile base and bespoke-teleop rows. **Done; section 7.** |
| `robotics/simulation_follow_me`, `sample_followme` | Person tracking to raw `cmd_vel`, Gazebo | Mobile base and perception rows |
| `robotics/sample_apriltag` | `apriltag_ros`, `qrb_ros_camera` | Perception row |
| `robotics/sample_3d_lidar_semantic_segmentation`, `sample_fall_detection` | `qrb_ros_nn_inference`, camera | Perception row |
| `ai_vision/*` | `qrb_ros_nn_inference` + `qrb_ros_camera` | Perception row; no robot, so the value is Behaviors and the Desktop App |
| `ai_audio/*` | Speech recognition, TTS | No MoveIt Pro counterpart today |

---

## 2. Migrating MoveIt 2 code to MoveIt Pro

MoveIt Pro ships the MoveIt libraries under its own names and replaces the `move_group` node and its planners. Application code that called `MoveGroupInterface` plans in-process instead. The public guide **Migrate from MoveIt 2 to MoveIt Pro Core** covers package and namespace renames and the OMPL → ProRRT and Pilz → Cartesian-planning replacements; this section covers what that guide does not: `MoveGroupInterface` and `move_group`.

### What carries over unchanged

- URDF and SRDF, including planning groups and named group states.
- `ros2_control` hardware and controller names.
- `joint_limits.yaml` values (but see [Acceleration limits](#acceleration-limits-are-yours-to-apply)).

### `MoveGroupInterface` call mapping

| MoveIt 2 | MoveIt Pro |
|---|---|
| `MoveGroupInterface(node, group)` | `moveit_pro::base::RobotModel` built from the URDF and SRDF strings, plus `moveit_pro::planning_scene_monitor::CurrentStateMonitor` on `/joint_states` for the start state |
| `setNamedTarget("ready")` | `RobotState::setToDefaultValues(group, "ready")` reads the SRDF `group_state` |
| `setJointValueTarget(map)` | `RobotState::setVariablePositions(map)` on a copy of the start state |
| `setPlannerId("RRTConnect…")`, `setPlanningTime` | `pro_rrt::RRTParams` |
| `setMaxVelocity/AccelerationScalingFactor` | `pro_rrt::TrajectoryParams::velocity_scale_factor` / `acceleration_scale_factor` |
| `plan()` | `pro_rrt::planTrajectoryToJointGoal(group, start, goal, rrt_params, constraints, trajectory_params, scene)`, returning a `trajectory_msgs/JointTrajectory` or a `PlanningError` with a readable `message` |
| `execute()` (through `move_group`) | An action client on the controller: `FollowJointTrajectoryWithAdmittance` for the JTAC, `control_msgs/GripperCommand` for a gripper (see [section 6](#6-moving-to-moveit-pros-controllers)) |
| `move_group` node, `moveit_controllers.yaml`, `MoveItConfigsBuilder` launch | Removed |

The port wraps planning in one small class, `MotionPlanner` (`src/qrb_pick_place_core/include/qrb_pick_place_core/motion_planner.hpp`), and keeps Qualcomm's step structure, joint maps, and log text in `src/qrb_pick_place_core/src/qrb_ros_arm_pick_place.cpp`, so the before/after diff against the original reads one-for-one.

### Acceleration limits are yours to apply

`move_group` loaded `joint_limits.yaml` and applied its acceleration limits to the robot model. Nothing does that for in-process planning, and URDF has no acceleration limits, so time parameterization fails with "No acceleration limit was defined for joint …". Apply them yourself: `MotionPlanner`'s constructor sets `acceleration_bounded`, `min_acceleration`, and `max_acceleration` on each joint's `VariableBounds` from a joint-to-limit map, and the launch file passes `joint_limits.yaml` in as parameters (`src/qrb_pick_place_core/launch/pick_place_core.launch.py`).

### The state monitor needs its own executor

`MoveGroupInterface` spun a private executor to receive `/joint_states`. If the application's main node sits in `rclcpp::spin_until_future_complete` waiting on an action, a `CurrentStateMonitor` on that same node never receives a message, and `waitForCompleteState` times out. Give the monitor a second node spun on a background thread (`state_node_` / `state_executor_` / `state_thread_` in `src/qrb_pick_place_core/src/qrb_ros_arm_pick_place.cpp`).

Own that thread with `std::jthread` and stop the executor in an object destroyed before it. A plain `std::thread` member that is still joinable when the constructor throws (for example, "no robot state within 10 s") calls `std::terminate`, and the node dies with SIGABRT instead of printing the error.

### Cancel what you stop waiting for

`MoveGroupInterface::execute` cancelled its goal when interrupted. A hand-written action client that times out waiting for a result must call `async_cancel_goal` on the accepted goal before giving up; otherwise the controller finishes the motion with nothing supervising it. The port does this for both the arm and the gripper, with the wait set by a `result_timeout_s` parameter.

### Build the robot model with validation

Use `RobotModel::create(urdf, srdf)` rather than the `RobotModel` constructor where the release provides it (check the release notes). `create` validates the model (including mimic joint limits, [section 6](#gripper-couple-the-fingers)) and returns the reason it was rejected; the constructor skips that check, so a model the Runtime would refuse loads silently in the application.

### Mimic joints in named states

When the gripper uses mimic joints ([section 6](#gripper-couple-the-fingers)), keep only the driving joint in its SRDF `group_state`s. `setToDefaultValues` then sets the followers through the URDF `<mimic>` multipliers and offsets, and the node sends the single driving value to the gripper controller.

### Building

- Add `moveit_pro_base`, `moveit_pro_planning_scene_monitor`, and `pro_rrt` as dependencies (plus `moveit_pro_controllers_msgs` when talking to the JTAC). Remove `moveit_ros_planning_interface` and `moveit_msgs` if nothing else uses them.
- A gripper action that succeeds does not prove the grasp: with `allow_stalling`, a close on empty air and a close that stops on the object both report success. Check the object's pose if the application must know.
- `rosdep` has no keys for MoveIt Pro packages. `rosdep install` fails with "Cannot locate rosdep definition for [pro_rrt]" unless they are listed in `--skip-keys`.
- Qualcomm's sample has a QIRP SDK cross-compile path that hard-codes sysroot include directories for each MoveIt 2 package. It was not ported or tested; a QIRP build needs the MoveIt Pro packages in the Yocto sysroot first.

### MoveIt Pro Core alone, without the Runtime

The planner part of the port, the `motion_planner` library, builds and passes its tests against only the standalone **MoveIt Pro Core** packages (`docker/core-only.Dockerfile` installs the Core archive on a clean `ros:jazzy` image and builds with `-DQRB_CORE_ONLY=ON`, which leaves out the node). Two things limit a Core-only deployment:

- The JTAC's action type lives in `moveit_pro_controllers_msgs`, which the Core archive does not ship. A Core-only application must drive a standard `JointTrajectoryController` with `control_msgs/FollowJointTrajectory` instead.
- Core's license check reads a hardware fingerprint. In a container it needs the host's network interfaces (`--network host`, or the host's `/sys` mounted with `MOVEIT_HOST_SYSFS` set); with bridge networking it fails with "MoveIt Pro License is invalid".

---

## 3. Migrating the simulation environment to MuJoCo

Gazebo worlds do not run in MoveIt Pro; the MoveIt Pro Simulator uses the MuJoCo physics engine. Rebuild the scene in MJCF and drive the robot through `picknik_mujoco_ros/MujocoSystem`. The public guide **Create MoveIt Pro Simulator Models** covers the converter in detail; these are the steps and the problems this port hit.

### Robot model

1. Render the robot xacro to URDF, remove any Gazebo-specific `ros2_control` block, and add `<mujoco><option integrator="implicitfast"/><compiler fusestatic="false" discardvisual="false"/></mujoco>` under `<robot>`.
2. In a directory with the URDF: `ros2 run moveit_studio_utils_py flatten_meshes robot_description.urdf`, then `ros2 run moveit_studio_utils_py urdf_to_mjcf robot_description.urdf`. The converter adds position actuators, gravity compensation, and geom names.
3. Post-process the generated MJCF with a script rather than by hand, so it can be regenerated: `tools/postprocess_rml63_mjcf.py` points meshes at `assets/`, places the base, sets actuator gains, adds a `tcp` site, and couples the gripper.
4. **A mobile base needs more post-processing.** `urdf_to_mjcf` writes arm-shaped output: gravity compensation on every body and a position actuator on every joint, passive casters included. For a base, give the root body a `<freejoint>`, remove `gravcomp`, put velocity actuators on the drive wheels only (keep `armature / kv` below the timestep), and make casters frictionless spheres on ball joints. `tools/postprocess_qrb_amr_mjcf.py` does this. The public guide's "floating reference joint" step for mobile manipulators fails when the URDF root is already the base ("free joint can only be used on top level"); add the free joint in post-processing instead.
5. **Decimate large meshes first.** MuJoCo rejects an STL with more than 200,000 faces ("number of faces should be between 1 and 200000"). `decimate_mesh in.stl out.stl` (in the Runtime image) brought Qualcomm's 403,096-face base mesh to 4,170. Count first: the mini base's 151,674-face mesh loads as it is.
6. **`flatten_meshes` resolves `package://` URIs**, so the package must be on the ament index. Before the new package is built, give it a throwaway prefix: create `<prefix>/share/ament_index/resource_index/packages/<package>`, symlink `<prefix>/share/<package>` to the source package, and prepend `<prefix>` to `AMENT_PREFIX_PATH`.
7. **Put the robot's world pose in both the URDF and the MJCF.** Gazebo samples often spawn the robot at a pose given on the command line (Qualcomm's README: `initial_x:=2.2 … initial_yaw:=3.14159`). MuJoCo needs it on the base body, and the URDF needs it on the `world` → base joint origin. Without the URDF half, `MujocoSystem` warns that 14 bodies "do not sit where the URDF places the same-named link", and the planner's collision model is offset from the simulation.

### Scene

- **Static collision geometry:** if the Gazebo world models collision with primitives, generate the MJCF from the SDF instead of retyping it. `tools/gen_sdf_collision_mjcf.py <file.sdf> <model> <class> <mjcf name>` converts one model's boxes, cylinders, and spheres, with full roll-pitch-yaw: `warehouse.sdf`'s 40 shapes became `description/warehouse.xml`, and the Ionic world's 183 became `qrb_amr_nav_sim/description/ionic.xml`.
- **Look inside Fuel models before fitting collision by hand.** The Ionic world `include`s a single "Ionic interior" model whose visual is one mesh, but its `model.sdf` also carries 183 primitive colliders, which convert directly. Only when a model really collides with a mesh, fit primitives to it or decompose it into convex parts with `obj2mjcf`.
- **MuJoCo does not load glTF.** A `.glb` visual fails with "no decoder found for mesh file". Either convert it (to OBJ, with its textures as PNG) or draw the collision primitives as the visual, as the Ionic port does.
- **Worlds built from Fuel `include`s:** `tools/gen_sdf_includes_mjcf.py` flattens a world's includes through their nested model poses into one MJCF (the office world's 64 includes became `qrb_remote_assistant_sim/description/office_furniture.xml`). OBJ models become a textured visual mesh plus their `*_Col.obj` collider; COLLADA models fall back to their own SDF primitives. `gen_sdf_collision_mjcf.py` composes link poses too, so walls defined per link (`office.sdf`'s 40 boxes) convert directly.
- **An OBJ placed at its SDF pose lands where Gazebo put it.** MuJoCo recenters each mesh on its centroid and moves the geom to compensate, so do not offset it by hand.
- **One MuJoCo mesh takes one material.** For a multi-material OBJ, use the texture covering the most faces: picking the first one (a book covering 30 of the seated child's 1544 faces) turned the child white.
- **Fuel models:** look up licenses through the Fuel API (`curl https://fuel.gazebosim.org/1.0/<owner>/models/<name>` returns `license_name`); download with `…/<version>/files.zip`. Use a primitive collision geom and keep the mesh visual-only; a mesh can's contact is worse than a cylinder's.
- **Textures:** MuJoCo rejects JPEG ("Non-PNG texture, assuming custom binary file format"). Convert to PNG.
- **Graspable objects** are bodies with a `<freejoint>`. Give each a `<site>` so its pose is published on TF (`coke1_site` in `mj_world`), which is how anything outside the simulation finds out where the object is.
- **Keyframe:** add a `default` keyframe with the robot's start configuration. Its `ctrl` vector must have one value per actuator, so it changes length when actuators are removed (for example, by gripper coupling).

### Making the scene visible in the Desktop App

The 3D view shows the URDF and the planning scene, not the MuJoCo scene. A port that skips this shows the robot floating alone.

- **Scene camera:** add a `<camera>` and a `<site name="<camera>_optical_frame">` to the MJCF. The Simulator publishes `<camera>/color`, `/depth`, `/points`, and `/camera_info`, and the Desktop App shows them. The optical-frame site is the camera orientation rotated 180° about its x-axis: multiply the camera's `xyaxes` by `1 1 1 -1 -1 -1`.
- **Planning scene:** add the static furniture as collision objects in an Objective, so it is drawn and the planner avoids it. Generate the Objective from the compiled MJCF, not from the source XML or the Gazebo world: `tools/gen_scene_objective.py` loads the model with MuJoCo, reads each furniture geom's world pose and size, and writes `AddCollisionBox` and `AddCollisionCylinder` calls. Hand-transcribed poses drift from the compiled model (rotated tables that do not match the camera). Set `overwrite="true"` on those Behaviors, so the Objective can run more than once without "CollisionObject … already exists".
- Leave grasped objects out of the static scene; they move.

### Planar lidar

A Gazebo `gpu_lidar` becomes one MuJoCo `<rangefinder>` per beam, which `MujocoSystem` publishes as a `sensor_msgs/LaserScan`:

- **Names:** every beam is a site named `<lidar>-<index>` (zero-padded) with its own rangefinder. Its `user` field is `noise angle_min angle_max angle_increment range_min range_max`. The scan is published in `<lidar>_ROS`, whose x axis points at beam 0, so `angle_min` is 0 there.
- **Write the beams out.** `<replicate>` fails in a model that has a keyframe in the MuJoCo version the Simulator ships, so generate the sites: `tools/add_rangefinder_lidar.py` takes Gazebo's sample count, angles, and ranges, using Gazebo's increment `(max - min) / (samples - 1)`.
- **Topic and rate:** a `<sensor name="<lidar>">` block in the `ros2_control` hardware with `<param name="topic">/scan</param>` sets the topic, and the hardware parameter `lidar_publish_rate` sets the rate.
- **A rangefinder ignores only its own body.** Every beam of the QRB mini's lidar first hit the base's visual mesh at 4 cm, because Gazebo's `range_min` near-clip had hidden it. Move visual-only geoms that enclose the sensor into the lidar's body, keeping their pose (`add_rangefinder_lidar.py --exclude-geom`).
- **Every beam is cast every physics step**, whatever the publish rate; the rangefinder's `interval` attribute is accepted but does not throttle it. 1800 beams cost 2.9 ms per step, which ran the simulation at 0.7× real time with a 2 ms step. Use the Gazebo world's own `max_step_size` (4 ms for Ionic, which gives 1.4×) or fewer beams, and measure the real-time factor offline before going further.
- **Ray cost grows with the scene.** Rangefinder rays test every geom, whatever its group, `contype`, or alpha; the rangefinder sensor in the MuJoCo version the Simulator ships has no group filter. The same 1800 beams that ran at 1.4× in the Ionic room ran at 0.74× in the furnished office (1.46× with the furniture's visual meshes removed). Moving visual meshes to other groups does not help. An image-based lidar (one camera with `lidar_output scan`, excluding the visual group) renders on the render thread instead of the physics step, but it changes the sensor model; the remote assistant port kept Qualcomm's ray lidar and its slower simulation.
- **An offline render full of yellow lines is not a scene bug.** MuJoCo's own viewer draws every rangefinder ray; the Simulator turns that off.

### Cameras on the CPU image

The CPU image renders with Mesa llvmpipe: 214 ms of CPU per 1280×720 frame. Two cameras at `render_publish_rate` 15 need 6.4 s of CPU per wall second, which competes with physics and slowed the remote assistant's Simulator from 0.74× to 0.34× real time in the full Runtime. Set `render_publish_rate` to what the application needs (2 Hz for a person check), or use the CUDA image on a host with an NVIDIA GPU. A robot camera from Gazebo carries over as a MuJoCo `<camera>` with Gazebo's resolution and its horizontal field of view converted to `fovy` (`tools/add_base_camera.py`).

### Checking the scene before ROS is involved

Load and step the scene in Python MuJoCo first (`mujoco.MjModel.from_xml_path`, `mj_resetDataKeyframe`, `mj_step`). Objects should rest where they spawn, and the arm should hold its keyframe. Driving the original joint targets through the actuators offline answers "can the gripper reach and hold the object?" in seconds, before any controller or planner is involved.

---

## 4. Creating the robot configuration package

A MoveIt Pro application is a robot configuration package. Copy the schema from a current package: `factory_sim` and `lab_sim` in older checkouts of `moveit_pro_example_ws` still use keys that current releases reject.

| File | Contents |
|---|---|
| `config/config.yaml` | `runtime_launch_file`, URDF/SRDF, `moveit_params`, `ros2_control`, behavior loader plugins, Objective paths, waypoints file |
| `config/moveit/` | SRDF and `joint_limits.yaml` (copied from the MoveIt 2 config), kinematics, `pose_jog.yaml`, `joint_jog.yaml` |
| `config/control/<robot>.ros2_control.yaml` | Controllers, keeping the original controller names |
| `description/` | Xacro with a `MujocoSystem` `ros2_control` block, `scene.xml`, the robot MJCF, `assets/` |
| `launch/runtime.launch.xml` | Includes `$(find-pkg-share moveit_studio_agent)/launch/studio_agent_bridge.launch.xml` |
| `objectives/`, `waypoints/` | [Section 5](#5-migrating-bespoke-sequencing-code-to-behavior-trees) |

`config.yaml` problems hit on this port, with the error each produced:

- `hardware.simulated`, `robot_driver_persist_launch_file`, and `simulated_robot_driver_persist_launch_file` were removed (their values move to `additional_driver_launch_file`), and `runtime_launch_file` is required. Older reference configs in `moveit_pro_example_ws` checkouts can still carry them. The user workspace migration guide for the release that removed them lists the replacements.
- `moveit_params.pose_jog` and `joint_jog` are required ("Field required"), even for a robot with no jog controllers.
- An empty list anywhere in the resolved parameters (`planning_groups: []`, `controllers_inactive_at_startup: []`) crashes the launch with "Expected 'value' to be one of […] but got '()' of type tuple". Omit optional list keys rather than setting them to `[]`, and give the jog files a real group and controller.

### Workspace scaffolding

`moveit_pro build user_image` needs `Dockerfile`, `docker-compose.yaml`, and `colcon-defaults.yaml` at the workspace root. Copy them from a workspace created by `moveit_pro setup_workspace` or from `moveit_pro_example_ws` for the same major release.

---

## 5. Migrating bespoke sequencing code to Behavior Trees

A hand-written sequence of motions becomes an Objective: a Behavior Tree of existing Behaviors and Subtrees. For a joint-target sequence like Qualcomm's, no C++ Behavior is needed.

1. **Joint targets become waypoints.** Each hard-coded joint map, and each SRDF named state the code used, becomes an entry in `waypoints/<robot>_waypoints.yaml`. Generate the file from the original values (`tools/gen_waypoints.py`) so nothing is retyped.
2. **Each move becomes a Subtree call.** Wrap `Move to Named Waypoint` once with the robot's group, controller, and the original speed scaling (`objectives/qrb_move_arm.xml`), then call it with only `waypoint_name`. Override the core `Open Gripper` and `Close Gripper` placeholder Objectives for the robot's gripper controller (`objectives/open_gripper.xml`, `close_gripper.xml`); the core versions log an error and fail.
3. **Group the steps** into named `Sequence`s that match the original's phases (Prepare, Pick, Place) so the tree reads like the original code.
4. **Load the scene first:** call the planning-scene Objective from [section 3](#making-the-scene-visible-in-the-desktop-app) at the start of the tree.

Before and after for this port:

| `qrb_ros_arm_pick_place.cpp` step | `QRB Pick and Place` (`objectives/qrb_pick_and_place.xml`) |
|---|---|
| — | `QRB Load Scene` |
| 1. `moveToNamedTarget(arm, "ready")` | `QRB Move Arm` `ready` |
| 2. `moveToNamedTarget(gripper, "open")` | `Open Gripper` |
| 3. `moveToJointAngles(getPickJointAngles())` | `QRB Move Arm` `pick` |
| 4. `moveToNamedTarget(gripper, "close")` | `Close Gripper` |
| 5. `moveToJointAngles(getRaiseJointAngles())` | `QRB Move Arm` `raise` |
| 6. `moveToJointAngles(getPlaceJointAngles())` | `QRB Move Arm` `place` |
| 7. `moveToNamedTarget(gripper, "open")` | `Open Gripper` |
| 8. `moveToJointAngles(getReRaiseJointAngles())` | `QRB Move Arm` `re_raise` |
| 9. `moveToNamedTarget(arm, "ready")` | `QRB Move Arm` `ready` |

**Rebuild after adding an Objective file.** The Runtime reads Objectives from the colcon install space, which links the files that existed at build time. A new `.xml` is invisible ("Can't find a tree with name …") until `moveit_pro build user_workspace` runs again; edits to existing files show up without a rebuild.

To run an Objective without the Desktop App:

```bash
ros2 action send_goal /do_objective moveit_studio_sdk_msgs/action/DoObjectiveSequence "{objective_name: 'QRB Pick and Place'}"
```

---

## 6. Moving to MoveIt Pro's controllers

Qualcomm's sample drove both arm and gripper with plain `JointTrajectoryController`s. That works on MoveIt Pro, but the platform's own controllers are what its Objectives, speed controls, and safety checks are built around.

### Arm: JointTrajectoryController → JointTrajectoryAdmittanceController (JTAC)

For an application running on the full MoveIt Pro Runtime, this is a configuration change:

```yaml
rm_group_controller:
  ros__parameters:
    planning_group_name: rm_group_controller   # joints come from the SRDF group, not a joints: list
    sensor_frames: [link8]                     # required even with no force-torque sensor
    ee_frames: [link8]
    stop_accelerations: [3.0, 3.0, 3.0, 3.0, 3.0, 3.0]
```

- Change the controller `type` to `joint_trajectory_admittance_controller/JointTrajectoryAdmittanceController` and keep its name.
- With `ft_sensor_names` empty, the JTAC runs without admittance: it is a trajectory controller with MoveIt Pro's speed scaling and stop handling.
- It refuses to configure without `sensor_frames` ("One of 'sensor_frame' (deprecated) or 'sensor_frames' (preferred) must be specified"), even with no sensor.
- It enforces the goal tolerance (`default_goal_tolerance`; see the controller's parameter reference for the default) and aborts a trajectory that ends outside it. Raise `default_goal_time_tolerance` for a simulation or robot that lags the command.
- Objectives call `Move to Named Waypoint` with `execution_pipeline="jtac"` (the default) instead of the `(JTC)` variant.
- **Custom nodes change their action type.** The JTAC serves `moveit_pro_controllers_msgs/action/FollowJointTrajectoryWithAdmittance` on `<controller>/follow_joint_trajectory`, not `control_msgs/FollowJointTrajectory`. An empty `goal_tolerance` uses the controller's default, an empty `admittance_parameters` disables admittance for that goal, and the result's text field is `error_message`.

### Gripper: couple the fingers

A gripper modelled as independent joints with separate position commands (Qualcomm's has four) works, but the fingers open at different rates. In this port, that asymmetry reached 0.13 rad as the fingers let go. MoveIt Pro's gripper Objectives expect one driven joint. Couple the fingers in three places, all generated from the same multipliers:

1. **URDF:** `<mimic joint="joint01" multiplier="…" offset="…"/>` on each follower. Fit the multipliers through the gripper's open and closed SRDF states (`tools/postprocess_rml63_mjcf.py` has the fit for this gripper). Then bound the driving joint so that its full range, mapped through each mimic, stays inside the follower's own limits, and give each follower velocity and acceleration limits of at least |multiplier| times the driver's. Current releases reject a robot description that breaks this ("Mimic joint '…' can violate its position limits"), and the Runtime does not start. For this gripper that meant limiting `joint01` to [0, 0.2], its open-to-closed range, and raising the followers' rate limits by 1.25×.
2. **MJCF:** remove the followers' actuators and add `<equality><joint joint1="<follower>" joint2="joint01" polycoef="<offset> <multiplier> 0 0 0"/></equality>`. Update the keyframe's `ctrl` to the remaining actuators, and its finger `qpos` to values that satisfy the coupling.
3. **ros2_control:** keep a command interface for the driving joint only, and run `position_controllers/GripperActionController` on it with `allow_stalling: true` so a close that stops on an object succeeds.

Afterwards the fingers stay mirrored to within 0.001 rad.

---

## 7. Mobile bases

A differential-drive base such as Qualcomm's QRB robot base needs no planner and no MoveIt 2 code; the migration is the drive controller, the frames, and moving teleoperation into the Desktop App. `lunar_sim` in `moveit_pro_example_ws` is the reference configuration for a differential-drive base. For SLAM and Nav2, `hangar_sim` is the reference: it launches Nav2 from the robot configuration package and drives it from Objectives.

### Drive controller

Replace Gazebo's `gz-sim-diff-drive-system` plugin with `diff_drive_controller/DiffDriveController` (`src/qrb_amr_sim/config/control/qrb_robot_base.ros2_control.yaml`):

- Carry over the wheel names, radius, separation, and any separation multiplier from the Gazebo xacro. Expect to recalibrate them: Qualcomm's 1.1525 separation multiplier turns about 9% faster than commanded in MuJoCo, where 1.05 matches. Measure against simulation ground truth, not against `/odom`, which is computed from the same parameters and so always agrees with the command. Current releases publish `<base_link>_ground_truth` under `mj_world`; use that frame, because `mj_world → base_footprint` resolves through the controller's odometry there. Recalibrating stops being optional once the application closes a loop on odometry: the remote assistant's scripted mapping route turns on `/odom`, Qualcomm's 1.125 made every 90° turn about 99° in MuJoCo, and the route drove into a wall. That port uses 1.05.
- Exactly one node owns `odom` → base frame. With wheel odometry alone, that is the controller (`enable_odom_tf: true`). With a SLAM node that provides the odometry frame (Cartographer's `provide_odom_frame = true`), set `enable_odom_tf: false`, which is also Qualcomm's Gazebo default.
- Name it as `lunar_sim` does (`platform_velocity_controller`) and map its topics to plain `/cmd_vel` and `/odom` with `controller_shared_topics` in `config.yaml`.
- **Commands must be `TwistStamped`, and they expire.** The ROS 2 Jazzy controller accepts only `geometry_msgs/TwistStamped` and drops any command older than `cmd_vel_timeout` (0.5 s), so a `Twist` publisher does nothing and a single command stops the base after half a second. Gazebo's plugin took `Twist` and kept driving on the last command. To keep an original `Twist` node working unchanged, relay it through a node that stamps each message and re-sends the last one at a fixed rate (`src/qrb_amr_sim/scripts/twist_stamper.py`).

### Frames

- **SRDF:** declare a planar virtual joint from `odom` to the base frame, so the planning scene follows the base through the controller's TF. Put the drive wheels in a planning group (for example `base`); the Runtime needs one even though nothing plans for it.
- **Planning-scene objects** for a base are expressed in `odom`. That matches the MuJoCo world only while odometry has not drifted; for a sample whose robot spawns at the origin, `odom` and the simulation world start out coinciding.
- **Simulator:** set `odom_frame` to `odom`, `publish_mj_world_tf` to `true`, and `base_link_name` on the `MujocoSystem` hardware block, as `lunar_sim` does. With SLAM, the tree's root is `map`, so set `odom_frame` to `map`: `MujocoSystem` then publishes `mj_world → map` as identity, and the scene camera hangs off `map`.
- **A shaking base in the 3D Visualizer means two TF parents.** When the MJCF's free joint is on a body above `base_link_name` (here `base_footprint`, the URDF root), older MoveIt Pro releases take the base for a scene object and publishes `mj_world → base_footprint` at 60 Hz next to the odometry's `odom → base_footprint`. Check with `ros2 topic echo /tf | grep -B1 -A1 "child_frame_id: <base>"`: more than one parent `frame_id` is the bug. If you see it, set `<param name="publish_object_tf">false</param>` in the hardware block (it only removes free scene objects' TF, and these scenes have none); current releases exclude the base's free body themselves, so the parameter is needed only on releases that predate that fix.
- **3D Visualizer:** it renders relative to `world` by default, and a base's TF tree has no `world` frame, so the Desktop App shows the scene camera but not the robot. Ship `config/frontend_settings.yaml` with `referenceFrame: odom` (or `map` with SLAM, together with the SRDF virtual joint's parent) in the robot configuration package. A per-user `frontend_settings.yaml` saved earlier overrides it; changing the 3D Visualizer's Fixed Frame to `odom` once fixes that case.

### Teleoperation replaces a bespoke teleop node

A keyboard or joystick node that publishes velocity commands has no Behavior Tree counterpart to write: the Desktop App's teleoperation does its job. For a differential-drive base today that needs one custom Behavior, `TeleoperateBase`, from `lunar_sim_behaviors` in `moveit_pro_example_ws`:

1. Vendor `lunar_sim_behaviors` into the workspace unchanged (BSD-3-Clause; record the source commit) and load `lunar_sim_behaviors::LunarSimBehaviorsLoader` in `config.yaml`.
2. Override `Request Teleoperation` with `lunar_sim`'s version, which runs `TeleoperateBase` in Pose mode (`src/qrb_amr_sim/objectives/request_teleoperation.xml`). Set its velocity limits to the original node's speeds.
3. Point `pose_jog.yaml` at the base group and the drive controller.

In the Pose tab, **X ±** drives and **Roll ±** turns: `TeleoperateBase` keeps only the jog command's `linear.x` and `angular.z`, and the Pose pad labels its `angular.z` buttons "Roll" because they are meant for an arm's end effector. The other buttons do nothing. Tell users this; the labels do not.

If the installed release provides a core base-teleoperation mode (`moveit_params.base_jog`), use it in place of the vendored Behavior and the Roll workaround.

`Request Teleoperation` cannot be run without a connected Desktop App: `DoTeleoperateAction`'s goal is rejected ("no UI tab is subscribed to /moveit_pro_ui/do_teleoperate/goal"). To check teleoperation from a script, run `TeleoperateBase` on its own with the same ports and publish `moveit_pro_controllers_msgs/VelocityForceCommand` on `/pose_jog/<group>`, then confirm in the app.

### Navigation: SLAM and Nav2

Qualcomm's navigation sample is a set of commands run next to Gazebo: Cartographer, then `nav2_bringup`'s `navigation_launch.py`, then RViz's "2D Goal Pose". All three move into the robot configuration package (`src/qrb_amr_nav_sim`):

- **Launch them as drivers.** `hardware.additional_driver_launch_file` in `config.yaml` names a launch file that starts with the Simulator (`launch/navigation_drivers.launch.py`). Declare `cartographer_ros` and `nav2_bringup` as `exec_depend`s: the base images do not ship Nav2, SLAM Toolbox, AMCL, or Cartographer, and `rosdep` installs them when the user image is rebuilt (`moveit_pro build user_image`).
- **Keep the sample's parameter files.** Copy its Cartographer Lua and `nav2_params.yaml` (here Nav2's own defaults) unchanged, and make every needed change in the launch file, so a diff against the original stays empty.
- **`use_sim_time` is false.** MoveIt Pro runs on wall time and the Simulator publishes no `/clock`; the sample's `use_sim_time:=true` was for Gazebo.
- **Nav2 must publish `TwistStamped`.** Jazzy's Nav2 publishes `Twist` unless each node that sends velocity sets `enable_stamped_cmd_vel` (`controller_server`, `velocity_smoother`, `collision_monitor`, `behavior_server`, `docking_server`). Nav2's `RewrittenYaml` adds a missing key only when given its full path, `<node>.ros__parameters.enable_stamped_cmd_vel`; a bare key changes only keys already in the file. The collision monitor's output, `/cmd_vel`, then reaches the drive controller with no relay.
- **The goal comes from an Objective.** `GetPoseFromUser` and `NavigateToPoseAction` on `/navigate_to_pose` replace RViz's "2D Goal Pose" (`objectives/navigate_to_clicked_pose.xml`). Load `moveit_pro::behaviors::NavBehaviorsLoader`. The sample's mapping teleop is the Desktop App's teleoperation, as above.

**Cartographer: upstream or a vendor fork.** Qualcomm's samples use their fork (`ros-jazzy-qcom-cartographer*` from `ppa:ubuntu-qcom-iot/qirp`), which publishes arm64 packages only. Upstream Cartographer (`ros-jazzy-cartographer-ros`) builds for both architectures and installs through `rosdep`. Its mapping is the same code; the fork adds a `SlamCommand` map save/load service. Running the fork's files on upstream takes two changes:

- The fork's `cartographer_node` builds its Lua file's path in as the default, so Qualcomm's launch file passes no `-configuration_directory` or `-configuration_basename`; upstream requires both.
- Qualcomm's Lua sets the fork-only option `save_pose_to_scv`, and upstream aborts on options it does not read ("Key 'save_pose_to_scv' was used the wrong number of times"). A two-line wrapper keeps their file untouched: `include "qrb_amr_mapping_2d.lua"`, `options.save_pose_to_scv = nil` (`config/cartographer/qrb_amr_mapping_2d_upstream.lua`).

To switch to the fork later, for example to build on the IQ-9, follow `src/qrb_amr_nav_sim/README.md`: add the PPA in the workspace `Dockerfile`, install the three fork packages instead of upstream's, and load Qualcomm's Lua directly.

**Velocity publishers that share `/cmd_vel`.** Before adding a node that publishes `Twist` on `/cmd_vel` next to Nav2 and the drive controller, list the topic's publishers (`ros2 topic info -v /cmd_vel`). Two message types on one topic is a type clash, and an extra publisher of zeros (Nav2's idle `collision_monitor`, for example) can halve the commanded speed. Remap each `Twist` node to its own topic and stamp it on the way in. A relay that re-sends the last command, as `twist_stamper.py` does for a press-once teleop node, is wrong for a node that publishes every control tick itself; give it `republish 0` there.

---

## 8. Running the original's nodes unchanged

The remote assistant port (`src/qrb_remote_assistant_sim`) runs Qualcomm's three nodes (`build_map_node`, `nav_preparation_node`, `task_manager_node`) byte-identical from a vendored copy, so that the same nodes run against the simulation now and against Qualcomm's own stack on the IQ-9 later. Everything the port needed went at the nodes' interfaces. These patterns apply to any application whose nodes should not be edited.

### Vendor the original's packages and record where they came from

Copy the packages unchanged and write a `VENDORED.md` with the source repository, commit, and license, and every change made *around* them (`COLCON_IGNORE`, `colcon.pkg`, skipped `rosdep` keys); none go inside their files. A message package published only as an arm64 binary (`ros-jazzy-qrb-ros-slam-msgs`) builds from its interface files with a new `CMakeLists.txt` (`src/qrb_ros_slam_msgs`).

### Shim a vendor-only service

Qualcomm's nodes call `/qrb_slam_command` (`SlamCommand` 0–5: start, finish, save, load, start localization, validate relocalization), which only their Cartographer fork serves. `scripts/slam_command_shim.py` serves the same service over upstream Cartographer's own services: `finish_trajectory`, `write_state`, `cartographer_node -load_state_filename` with a localization `start_trajectory`, and, for "relocalized", the first `/constraint_list` constraint between different trajectories. It fails with `DEADLINE_EXCEEDED` after a timeout, so a caller that polls it cannot spin forever. On the IQ-9 the shim is switched off (`slam_backend:=qualcomm`) and the fork serves the service itself.

### Stand in for a model that needs the vendor's NPU

The sample's person check is YOLOv8 compiled for the IQ-9's HTP (`yolov8_det_qcs9075.bin`), publishing `vision_msgs/Detection2DArray` on `/yolo_detect_result`; the task manager reads only whether a `person` is there. The port keeps that topic as the seam. The `QRB Detect Person` Objective runs SAM3 with the text prompt "person" (`GetMasks2DFromExemplar`, vendored `moveit_pro_sam3`) and publishes the mask count, and `scripts/detection_bridge.py` turns each count into the message the task manager expects. On the IQ-9, `detector:=qualcomm` drops the stand-in and Qualcomm's own pipeline publishes the topic.

Check the stand-in's latency against the original's timeouts. SAM3 on the CPU takes about 30 s per frame, and `task_manager_node` waits only 10 s after arrival, so the Objective loops continuously from before the robot arrives (the task manager ignores detections until then).

### Original orchestration that starts subprocesses

Qualcomm's `map_nav_setup.py` and `nav_preparation_node` start their own `ros2 launch` subprocesses, with `use_sim_time:=true` and a launch file only the fork ships. MoveIt Pro publishes no `/clock`, so those nodes would sit at time 0.

- **Re-run a launcher script's sequence** from the robot configuration package's driver launch, in the same order and with the same commands to the original nodes (`launch/remote_assistant_pipeline.launch.py`).
- **Intercept a subprocess a node starts itself** with a wrapper ahead of the real `ros2` on that node's `PATH` only. `scripts/nav2_on_wall_time/ros2` matches `ros2 launch nav2_bringup navigation_launch.py`, runs the port's wall-time, stamped Nav2 launch instead, and passes every other command through.
- **Keep the original's start order.** Start Nav2 where the original does (after relocalization), not at launch, so nothing but the original's nodes publishes velocity commands during the scripted mapping drive. Starting a second Nav2 next to a running one is not a fallback: the duplicate-named lifecycle nodes sit unconfigured, and their `Twist` publishers clash on `/cmd_vel`.

### Use the original's own end-to-end check

`task_manager_node` with its own prompt ("go to office to check person") is the end-to-end test: when it reports "Navigation target reached" and gets its detection, the migration works as the original defined it. That needs no new Behavior and no reimplementation of the original's success criterion. Check for a Behavior before writing one, too: search the installed Behavior library for one that waits on the event (the core library's waits include `WaitForDuration` and `WaitForUserDecision`), and where the original already has a node that waits for it, run that node instead of writing a Behavior.

---

## 9. Environment and tooling gotchas

- **Use the matching MoveIt Pro CLI.** The robot configuration schema and `moveit_pro` flags change between major releases; check `moveit_pro run --help` for the installed CLI rather than copying flags from older instructions. Install the CLI release that matches the images you will run, and rebuild the user workspace from clean after changing it.
- **Gazebo's spawn-time configuration lives outside the robot files.** Pose, world, and controllers loaded by separate launch files all have to be found and moved into the URDF/MJCF and `config.yaml`. Check every launch file the README tells users to run, including in the simulation repository (`qrb_ros_simulation`).
- **`docker exec` into the Runtime container needs the bashrc.** The container's own environment leaves `CYCLONEDDS_URI` empty; only `/etc/skel/.moveit-pro-bashrc` (inside the image) sets it. A node started from a plain `docker exec` shell never discovers `/joint_states`. Run `source /etc/skel/.moveit-pro-bashrc` first.
- **A container outside the MoveIt Pro stack must match its DDS settings.** The stack runs Cyclone DDS over loopback, unicast only. A separate container needs `rmw_cyclonedds_cpp`, the same `ROS_DOMAIN_ID`, and the same Cyclone configuration (`docker/cyclonedds_loopback.xml`, copied from `~/.ros/cyclonedds.xml` inside the Runtime container, reachable with `moveit_pro shell`).
- **`ros2 launch` exits 0 when its node dies.** Judge a run by the node's own exit line ("process has finished cleanly" vs. "process has died"), not by the launch command's exit code.
- **The dev containers share the host PID namespace.** `pkill -f "ros2 launch <pkg>"` also matches and kills the shell running it. Match anchored executable paths (`tools/stop_sim.sh`).
- **Running a workspace in another container** (for example a controllers-only check without the Runtime): mount it at the path it was built at inside the Runtime container (`/home/<user>/user_ws`). colcon's setup files record their install prefix, so a different mount point fails with "Package '…' not found".
- **Cyclone DDS rejects high domain IDs.** A `ROS_DOMAIN_ID` above roughly 232 fails with "resulting port numbers … are out of range". Use a domain under 100 for side containers.
- **TLS-inspecting corporate networks** re-sign Docker Hub's certificate with the gateway's own CA, so image pulls and builds fail with "x509: certificate signed by unknown authority" (check the issuer with `openssl s_client -connect registry-1.docker.io:443`). Pull through a registry mirror instead: add `"registry-mirrors": ["https://mirror.gcr.io"]` to `/etc/docker/daemon.json` and restart Docker. `docker manifest inspect` ignores the mirror, so test with `docker pull`. Some tags, such as the CUDA images, can be incomplete on the mirror; `MOVEIT_TARGET= moveit_pro build user_image` (empty) builds on the CPU base image instead.
- **`rosdep` reports "no rosdep rule" when `ROS_DISTRO` is unset**, outside a sourced ROS environment. Set `ROS_DISTRO=jazzy` before concluding a package has no rule.
- **There is no `/clock`.** Any `use_sim_time:=true` in the original, including inside a subprocess a node launches itself, leaves that node at time 0. See [section 8](#original-orchestration-that-starts-subprocesses).
- **Simulation speed varies between test setups.** The same office scene ran at 0.74× physics-only, about 0.48× in a drivers-only container, and 0.34× in the full Runtime with two cameras rendering. Any application that closes loops on wall-clock timers (Qualcomm's `build_map_node` checks its turns every 0.2 s of wall time) behaves differently in each. Measure the real-time factor in the setup you will demonstrate in.
- **SAM3:** `moveit_pro configure --sam3-license` acts only on a clone of `moveit_pro_example_ws`. Elsewhere, vendor `moveit_pro_sam3` and record the Meta SAM License acceptance in its `VENDORED.md`. On the CPU it takes about 30 s per frame, and the Objective server held 16 GB RSS once the model was loaded.
- **Moving to a newer release mid-port:** upgrade the CLI and images together, rebuild the user workspace from clean, and re-run each sample's checks. A newer release can make workarounds unnecessary (the `publish_object_tf` parameter, [Frames](#frames)) and can change which TF path reaches the base ([Drive controller](#drive-controller)). Mark each workaround with the release that needs it, so it can be found and removed.

---

## 10. Known issues carried over from the original

Each port keeps the original's sequence, targets, and parameters; its job is to prove the migration, not to fix the demo. The exception is a value that breaks the original's own sequence in MuJoCo; those are changed and listed.

**Pick and place.** One defect in Qualcomm's joint targets shows up in MuJoCo:

- **The can is knocked over after release in roughly one run in four.** `re_raise` does not lift the gripper straight up: the tool moves 16 cm sideways while rising 20 cm, and for the first 40% of that motion the open fingers sweep through the released can's footprint (clearance −0.016 to −0.041 m, measured by `rml63_sim/test/release_sweep.py`). The can lands upright on the box and is then clipped. A vertical retreat pose after the release would fix it: (−31.0, −44.68, −40.52, 0, −15.8, 0)° lifts the tool 10 cm straight up. Not applied, to keep the sequence identical. Whether it also happens in Qualcomm's Gazebo setup is untested.
- **The ported node does not plan around the tables; the Objective does.** Every target is a hard-coded joint configuration that happens to clear them. `QRB Load Scene` puts the tables in the planning scene, so the Objective's plans avoid them, but the node plans in an empty scene (self-collision only), as Qualcomm's original did. Neither chose its targets with the scene in mind.

**Simple motion.**

- **Turns run about 9% fast** with Qualcomm's wheel separation multiplier (1.1525) in MuJoCo; 1.05 matches the commanded rate. Kept at Qualcomm's value.
- **The base stops when commands stop**, 0.5 s after the last one, where Gazebo kept driving on the last command. This is `diff_drive_controller`'s safety behavior and is kept; the relay that lets the original node run re-sends its last command to preserve press-once-to-drive for that node only.

**Navigation.**

- **Turns run about 9% fast** here too: the mini base's 1.125 separation multiplier gives 1.094 rad/s for 1.0 commanded. Kept at Qualcomm's value.
- **The room is drawn with its collision shapes**, not the Fuel model's textured mesh, which MuJoCo cannot load.

**Remote assistant.**

- **Changed: `wheel_separation_multiplier` 1.05**, not Qualcomm's 1.125. `build_map_node` closes each turn on odometry, and with 1.125 each 90° turn is about 99° true, which drives the route into a station wall.
- **Changed: `control_period` 0.05 s**, not 0.2. Qualcomm's README gives this remedy "if the robot overshoots turns and hits walls"; at 0.74× real time, a 0.2 s wall-clock loop overshoots each turn by about 4°, and the route clears one wall with a 4.4° heading budget.
- **The mapping route is timing-sensitive.** It depends on the Simulator's speed (see [section 9](#9-environment-and-tooling-gotchas)), so the route and saved map differ between runs in the full Runtime. In Gazebo at 1.0× real time, the same arithmetic gives about 3° of overshoot per turn.
- **The Simulator runs below real time** in the office (0.74×) with Qualcomm's 1800-beam lidar, kept as is; Cartographer and Nav2 run on wall time.
- **Cameras render at 2 Hz** on the CPU image (`render_publish_rate`), enough for the person check.
- **Validated setups:** `task_manager_node`'s end-to-end run ("Navigation target reached", then the detection) passes in the drivers-only setup; SAM3 detection passes on saved frames (one person at the office goal, none at an empty wall) and runs live in the Runtime.

---

## 11. Reference: the ported samples

This repository holds one robot configuration package per ported sample; switch between them with `moveit_pro configure -c <package>`.

| Path | Role |
|---|---|
| `src/qrb_pick_place_core/src/qrb_ros_arm_pick_place.cpp` | Qualcomm's node ported to MoveIt Pro: same class, steps, joint maps, and log text |
| `src/qrb_pick_place_core/src/motion_planner.cpp` | `MotionPlanner`: named states and joint targets → ProRRT trajectories |
| `src/qrb_pick_place_core/launch/pick_place_core.launch.py` | Starts only the node, with the URDF, SRDF, and joint limits as parameters |
| `src/rml63_sim/config/config.yaml` | Robot configuration package |
| `src/rml63_sim/config/control/rml63.ros2_control.yaml` | JTAC arm and gripper action controller |
| `src/rml63_sim/description/scene.xml`, `rml63.xml`, `warehouse.xml` | MuJoCo pick cell, robot, and warehouse |
| `src/rml63_sim/objectives/qrb_pick_and_place.xml` | The nine-step Objective |
| `src/rml63_sim/objectives/qrb_load_scene.xml` | Generated planning-scene furniture |
| `src/rml63_sim/launch/sim_drivers_only.launch.py` | Simulator, controllers, and `robot_state_publisher` only, with no Runtime |
| `tools/` | The generators and post-processing scripts cited in sections 3 and 5, and the drivers-only run scripts |
| `src/qrb_amr_sim/config/config.yaml` | Robot configuration package for the QRB robot base |
| `src/qrb_amr_sim/config/control/qrb_robot_base.ros2_control.yaml` | `diff_drive_controller` replacing Gazebo's diff-drive plugin |
| `src/qrb_amr_sim/description/scene.xml`, `qrb_robot_base.xml` | Base in the warehouse, with a scene camera |
| `src/qrb_amr_sim/objectives/request_teleoperation.xml` | Pose-tab teleoperation for the base |
| `src/qrb_amr_sim/objectives/qrb_amr_load_scene.xml` | Generated warehouse planning-scene objects |
| `src/qrb_amr_sim/scripts/twist_stamper.py` | Lets Qualcomm's unmodified `Twist` node drive the controller |
| `src/qrb_amr_nav_sim/config/config.yaml` | Robot configuration package for the QRB robot base mini with SLAM and Nav2 |
| `src/qrb_amr_nav_sim/launch/navigation_drivers.launch.py` | Cartographer and Nav2, started with the drivers |
| `src/qrb_amr_nav_sim/config/cartographer/`, `config/nav2/` | The sample's Cartographer Lua and Nav2's default parameters, unchanged, plus the upstream wrapper |
| `src/qrb_amr_nav_sim/description/scene.xml`, `ionic.xml`, `qrb_robot_base_mini.xml` | Base with its 1800-beam lidar in the Ionic world |
| `src/qrb_amr_nav_sim/objectives/navigate_to_clicked_pose.xml` | Replaces RViz's "2D Goal Pose" |
| `src/qrb_amr_nav_sim/README.md` | Switching to Qualcomm's Cartographer fork |
| `src/qrb_remote_assistant_sim/config/config.yaml` | Robot configuration package for the remote assistant |
| `src/qrb_remote_assistant_sim/launch/remote_assistant_pipeline.launch.py` | `map_nav_setup.py`'s sequence around Qualcomm's unchanged nodes |
| `src/qrb_remote_assistant_sim/scripts/slam_command_shim.py` | `/qrb_slam_command` over upstream Cartographer |
| `src/qrb_remote_assistant_sim/scripts/nav2_on_wall_time/ros2` | Runs the wall-time Nav2 launch when `nav_preparation_node` starts Nav2 |
| `src/qrb_remote_assistant_sim/scripts/detection_bridge.py`, `objectives/detect_person.xml` | SAM3 person check in place of QNN YOLO |
| `src/qrb_remote_assistant_sim/description/scene.xml`, `office_walls.xml`, `office_furniture.xml` | The office world, generated from `office.sdf` |
| `src/qrb_remote_assistant_sim/README.md` | Running it, and switching to Qualcomm's stack on the IQ-9 |
| `src/simulation_remote_assistant/`, `src/qrb_ros_slam_msgs/` | Qualcomm's nodes and `SlamCommand` messages, vendored |
| `src/moveit_pro_sam3/` | Vendored SAM3 models package |
| `src/lunar_sim_behaviors/` | Vendored `TeleoperateBase` (see `VENDORED.md`) |
| `docker/core-only.Dockerfile` | MoveIt Pro Core archive on a clean `ros:jazzy` image |

### Run it

```bash
moveit_pro configure -w <path to this repository> -c rml63_sim
moveit_pro build user_workspace
moveit_pro run
```

Then, in the Desktop App, run **QRB Pick and Place**. `Reset MuJoCo Sim` puts the can back.

For the base, configure `-c qrb_amr_sim`, run **QRB AMR Load Scene**, and drive from teleoperation's Pose tab (X ± drives, Roll ± turns).

For navigation, configure `-c qrb_amr_nav_sim` (rebuild the user image first, so Cartographer and Nav2 are installed). Drive around in teleoperation to build the map, then run **Navigate to Clicked Pose** and click a goal on the floor.

For the remote assistant, configure `-c qrb_remote_assistant_sim` and run. The mapping drive starts on its own; wait for "Relocalization successful, starting navigation" in the drivers log. Run **QRB Detect Person** in the Desktop App, then in a Runtime shell run `ros2 run simulation_remote_assistant task_manager_node` and type `go to office to check person`.

To run the ported node instead of the Objective, while the Runtime is up:

```bash
docker exec -it -u $USER <Runtime container name from docker ps> bash -c \
  'source /etc/skel/.moveit-pro-bashrc && ros2 launch qrb_pick_place_core pick_place_core.launch.py'
```

### Asset licenses

| Asset | License |
|---|---|
| RML-63 arm meshes (RealMan `ros2_rm_robot`) | Apache-2.0 |
| Gripper meshes and URDF (`qrb_ros_samples`) | BSD-3-Clause-Clear |
| Fuel `OpenRobotics/Table` | CC0-1.0 |
| Fuel `OpenRobotics/Coke`, `QCforward/carboard box` | CC-BY-4.0 |
| Warehouse collision geometry (`qrb_ros_simulation`) | BSD-3-Clause-Clear |
| QRB robot base URDF and `wheelTec_AMR.STL` (`qrb_ros_simulation`) | BSD-3-Clause-Clear |
| QRB robot base mini URDF and `wheelTec_AMR_mini.STL` (`qrb_ros_simulation`) | BSD-3-Clause-Clear |
| Fuel `OpenRobotics/Ionic interior` collision shapes | CC-BY-4.0 |
| Qualcomm's Cartographer Lua and launch file (`ros-jazzy-qcom-cartographer-ros`) | Apache-2.0, as upstream Cartographer |
| `lunar_sim_behaviors` (`moveit_pro_example_ws`) | BSD-3-Clause |
| `simulation_remote_assistant` and `qrb_ros_slam_msgs` (Qualcomm) | BSD-3-Clause-Clear |
| Office world Fuel models (people, furniture, `Coke`, `MonitorAndKeyboard`; list in `qrb_remote_assistant_sim/description/assets/LICENSES.md`) | CC-BY-4.0 or CC0-1.0 |
| `moveit_pro_sam3` package (`moveit_pro_example_ws`) | BSD-3-Clause |
| SAM 3 ONNX models it downloads | Meta SAM License (use restrictions; each user accepts it before building, see `src/moveit_pro_sam3/README.md`) |

Each vendored package records its source commit and license in its `VENDORED.md`, and the office world's Fuel models are listed in `src/qrb_remote_assistant_sim/description/assets/LICENSES.md`.
