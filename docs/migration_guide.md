# Migrating a ROS 2 Application to MoveIt Pro

## Guide Overview

This guide shows you how to move an existing ROS 2 robotics application onto MoveIt Pro without rewriting what it does. We port the application in stages and prove each stage works before starting the next: first we understand the application, then we rebuild its world in the MoveIt Pro Simulator, then we give the robot a configuration package and MoveIt Pro's controllers, and only then do we port its behavior. Throughout, we keep the original's sequence, targets, and parameters, because the goal is to prove the migration, not to build a better demo.

Your application does not need to use MoveIt 2 to benefit. Even with no arm, it gains the Desktop App (run, edit, and monitor Objectives, a 3D view, teleoperation), the Behavior Tree runtime and its Behavior library, the MoveIt Pro Simulator, real-time controllers, and one robot configuration package that simulation and hardware share.

By the end of this guide, you will have:

- A triage of your application: what it uses, where its configuration hides, and which MoveIt Pro reference covers each part.
- Its robot and world rebuilt in the MoveIt Pro Simulator.
- A robot configuration package that runs in the Desktop App, with the robot visible in its scene and moving under teleoperation.
- The original's behavior running as an Objective, or as the original's own nodes, started from the Desktop App by someone who did not write the port.

| Stage | You end with |
|---|---|
| [Pre-reqs](#pre-reqs) | The tools, the source, and the ground rules |
| [1. Triage the Application](#stage-1-triage-the-application) | A list of what the application uses and where its configuration hides |
| [2. Set Up the Workspace](#stage-2-set-up-the-workspace) | A workspace and an empty robot configuration package that build |
| [3. Rebuild the World in MuJoCo](#stage-3-rebuild-the-world-in-mujoco) | A robot and scene that load, settle, and move offline |
| [4. Robot Configuration Package and Controllers](#stage-4-robot-configuration-package-and-controllers) | The robot running in the Desktop App, visible in its scene, moving under teleoperation |
| [5. Port the Behavior](#stage-5-port-the-behavior) | The original's behavior running as an Objective, or as the original's own nodes |
| [6. Hand It Over](#stage-6-hand-it-over) | A port someone else can run, with its known issues written down |

Work through the stages in order. Each one ends with a **Checkpoint** that tells you what you should see before you move on. Stage 1 tells you which parts of stages 4 and 5 your application needs, so you can skip the rest. If something goes wrong, [Troubleshooting](#troubleshooting) indexes every error message this guide covers.

> [!NOTE]
> **About the worked examples:** We wrote this guide while porting four of Qualcomm's [`qrb_ros_samples`](https://github.com/qualcomm-qrb-ros/qrb_ros_samples): a pick-and-place arm on MoveIt 2, a differential-drive base, a base that maps with Cartographer and drives with Nav2, and a scripted remote-assistant pipeline. Every step, rename, and gotcha below came up on those ports, and their code is in this repository. Notes marked **Worked example** show how a step played out in one of them and which file to look at. Paths are relative to this repository unless they name another one (`qrb_ros_samples`, `qrb_ros_simulation`, `moveit_pro_example_ws`). [Appendix A](#appendix-a-the-worked-examples) describes each port.

> [!IMPORTANT]
> **Required Version:** The worked examples in this repository require MoveIt Pro 10.2 or newer. The steps in this guide apply to other releases too; where a step needs a minimum release, it says so.

---

## Pre-reqs

You should have MoveIt Pro installed and licensed: the `moveit_pro` CLI, a license stored with `moveit_pro configure`, and Docker. If you haven't installed it yet, follow the installation guide in the [MoveIt Pro documentation](https://docs.picknik.ai/) before continuing. We assume you have some familiarity with ROS 2 Jazzy, URDF and SRDF, and `ros2_control`, and that you have worked through the [MoveIt Pro tutorials](https://docs.picknik.ai/tutorials/quick_start_intro/).

You will also need:

- **The original application's source**, plus every repository its README launches. Simulation repositories in particular hold spawn poses, worlds, and controllers that the application's own packages do not.
- **A clone of [`moveit_pro_example_ws`](https://github.com/PickNikRobotics/moveit_pro_example_ws)** on the release branch that matches your CLI (`moveit_pro --version`; list the branches with `git branch -r`). Its robot configurations are the references this guide points to: `lab_sim` for an arm, `lunar_sim` for a differential-drive base, `hangar_sim` for SLAM and Nav2, and `april_tag_sim` for AprilTags. An older clone can lag the release, so compare it against `origin/main` before you conclude that a reference is missing.

> [!TIP]
> **Use the matching CLI:** Install the CLI release that matches the MoveIt Pro images you will run. The configuration schema and CLI flags change between major releases, so check `moveit_pro run --help` for the installed CLI rather than copying flags from older instructions.

> [!NOTE]
> **GPU Requirements:** If your scene has cameras, use a host with an NVIDIA GPU. The CPU image renders cameras in software, slowly enough to slow the whole simulation ([Cameras](#cameras)).

The offline checks in Stage 3 use Python MuJoCo, which ships in the MoveIt Pro image. Run them from a MoveIt Pro shell (`moveit_pro shell`).

### Ground Rules

We follow three rules on every port:

- **Keep the original's behavior.** Port its sequence, targets, speeds, and parameters as they are. When one of its values is wrong in MuJoCo, write the defect down ([Stage 6](#stage-6-hand-it-over)) rather than fixing it. Change a value only when it breaks the original's own sequence in the Simulator, and record that change too.
- **Keep the original's files.** Copy parameter files (SLAM, Nav2, controllers) unchanged and make each needed change in a launch file or a wrapper. That way a diff against the original stays empty, and switching back to the vendor's stack stays easy. Vendor the original's packages unchanged too ([Running the Original's Nodes Unchanged](#running-the-originals-nodes-unchanged)).
- **Generate, don't transcribe.** Convert worlds, waypoints, and scene objects from the original's files with scripts, so nothing is retyped and everything can be regenerated.

---

## Stage 1: Triage the Application

In this stage, we work out what the application uses, so we know which of the later stages it needs. Read every `package.xml` and launch file in the application, and every launch file its README tells users to run, then sort what it uses into the rows below. A codebase usually spans several.

```bash
# From the application root: list the dependencies that decide the migration path.
grep -rhoE "<(exec_|build_)?depend>[a-z0-9_]+" --include=package.xml . | sed 's/.*>//' | sort | uniq -c | sort -rn
grep -rhoE "\b(MoveGroupInterface|move_group|moveit_configs_utils|nav2_[a-z_]+|cartographer|slam_toolbox|apriltag_ros|cmd_vel|gz_ros2_control|ros_gz_[a-z]+)\b" .
```

| The application uses | What moves to MoveIt Pro | Where |
|---|---|---|
| Gazebo or another simulator | Rebuilt as a MoveIt Pro Simulator (MuJoCo) scene | Stage 3 |
| MoveIt 2 (`MoveGroupInterface`, `move_group`, a MoveIt Setup Assistant config, OMPL, Pilz, Servo, MTC) | Planning calls, robot configuration, execution | Stages 3, 4; [Sequencing](#sequencing-code-becomes-an-objective) and [MoveIt 2 code](#moveit-2-code-plans-in-process) |
| An arm driven another way (vendor SDK, raw `JointTrajectory`, hand-written IK) | The arm becomes a MoveIt Pro robot: `ros2_control` driver, URDF/SRDF, an Objective in place of the hand-written sequence | Stages 3, 4; [Sequencing](#sequencing-code-becomes-an-objective) |
| A mobile base (Nav2, `cmd_vel`, SLAM, Cartographer) | A `ros2_control` drive controller in the Simulator; navigation runs inside MoveIt Pro | Stages 3, 4 ([Mobile base](#mobile-base)); [Navigation](#navigation-slam-and-nav2); the MoveIt Pro docs under **Mobile Navigation** |
| A bespoke teleoperation node (keyboard or joystick to `cmd_vel` or jog commands) | Nothing to port: the Desktop App's teleoperation replaces it. Keep the original node runnable against the simulation to prove the controller interface matches | Stage 4 ([Teleoperation](#teleoperation)) |
| Perception or ML inference (camera nodes, ONNX or QNN models, AprilTag) | Behaviors in an Objective; MoveIt Pro ships detection, segmentation, AprilTag, and NPU inference Behaviors. A model that runs only on a vendor's NPU gets a stand-in behind the same topic | [Sequencing](#sequencing-code-becomes-an-objective), [Running the original's nodes unchanged](#running-the-originals-nodes-unchanged); see **Configure Qualcomm Dragonwing** for the NPU |
| Its own nodes the port should not touch (vendor scripts, orchestration that launches subprocesses) | Run them unchanged, with shims at their interfaces | [Running the original's nodes unchanged](#running-the-originals-nodes-unchanged) |

**Find the hidden configuration.** Gazebo samples set much of the robot up outside the robot's own files: the spawn pose on the command line, the world in a separate launch file, controllers loaded by another package. Each of those has to be found and moved into the URDF, the MJCF, or `config.yaml`. Check every launch file the README runs, including in the simulation repository.

**Find the closest reference.** For each row, find the `moveit_pro_example_ws` configuration and MoveIt Pro docs page that cover it, and check the docs' support tables for the exact variant (drive type, sensor type). A missing reference for your variant is the biggest sizing risk; say so before planning.

> [!NOTE]
> **Worked example:** Qualcomm's pick-and-place README spawns the arm with `initial_x:=2.2 … initial_yaw:=3.14159` on the Gazebo command line, and the world comes from `qrb_ros_simulation`. [Appendix B](#appendix-b-survey-of-qrb_ros_samples) shows how every Qualcomm sample triaged.

### Checkpoint

You should now have a written list of:

- the rows that apply, and the files that prove each one;
- the hidden configuration, and where each piece moves;
- the reference configuration for each part of the application.

---

## Stage 2: Set Up the Workspace

In this stage, we create the workspace the port lives in, with an empty robot configuration package that later stages fill in.

> [!NOTE]
> **What is a robot configuration package?** A MoveIt Pro application is a robot configuration package: a ROS 2 package whose `config/config.yaml` names the robot's URDF and SRDF, its controllers, the Behaviors to load, and where its Objectives live. The same package runs the robot in simulation and on hardware. [Stage 4](#stage-4-robot-configuration-package-and-controllers) fills it in.

1. Add the workspace scaffolding. `moveit_pro build user_image` needs `Dockerfile`, `docker-compose.yaml`, and `colcon-defaults.yaml` at the workspace root. Copy them from a workspace created by `moveit_pro setup_workspace`, or from `moveit_pro_example_ws` for the same major release.
2. Vendor what the port will reuse. Copy the original's packages, and any example-workspace packages you need, into `src/` unchanged, each with a `VENDORED.md` that records its source commit and license.
3. Create the robot configuration package. Copy its schema from a current package rather than an old one: `factory_sim` and `lab_sim` in older clones of `moveit_pro_example_ws` still use keys that newer releases reject.
4. Point the CLI at the workspace and the new package:

   ```bash
   moveit_pro configure -w <workspace> -c <package>
   ```

5. Build the workspace:

   ```bash
   moveit_pro build user_workspace
   ```

### Checkpoint

You should see the build finish with your vendored packages and the new package in it. When a package declares new system dependencies, run `moveit_pro build user_image` as well, which installs them through `rosdep`.

---

## Stage 3: Rebuild the World in MuJoCo

In this stage, we rebuild the robot and its world for the MoveIt Pro Simulator. Gazebo worlds do not run in MoveIt Pro; the MoveIt Pro Simulator uses the MuJoCo physics engine. Rebuild the robot and scene in MJCF; `picknik_mujoco_ros/MujocoSystem` drives the robot from `ros2_control`. The public guide **Create MoveIt Pro Simulator Models** covers the converter in detail; this stage covers the steps and the problems the ports hit. We do this first, because everything after it is validated against this scene.

### Robot Model

1. **Prepare the URDF.** Render the robot xacro to URDF, remove any Gazebo-specific `ros2_control` block, and add `<mujoco><option integrator="implicitfast"/><compiler fusestatic="false" discardvisual="false"/></mujoco>` under `<robot>`.
2. **Decimate large meshes.** MuJoCo rejects an STL with more than 200,000 faces. `decimate_mesh in.stl out.stl` (in the MoveIt Pro image) reduces one; count first, since many meshes load as they are.
3. **Convert.** In a directory with the URDF: `ros2 run moveit_studio_utils_py flatten_meshes robot_description.urdf`, then `ros2 run moveit_studio_utils_py urdf_to_mjcf robot_description.urdf`. The converter adds position actuators, gravity compensation, and geom names. `flatten_meshes` resolves `package://` URIs, so the package must be on the ament index: before it is built, create `<prefix>/share/ament_index/resource_index/packages/<package>`, symlink `<prefix>/share/<package>` to the source package, and prepend `<prefix>` to `AMENT_PREFIX_PATH`.
4. **Post-process with a script**, not by hand, so the model can be regenerated.
5. **Put the robot's world pose in both the URDF and the MJCF**: on the base body in MuJoCo, and on the `world` → base joint origin in the URDF. Without the URDF half, `MujocoSystem` warns that bodies "do not sit where the URDF places the same-named link", and the planner's collision model is offset from the simulation.

> [!NOTE]
> **Worked example:** `tools/postprocess_rml63_mjcf.py` points the robot's meshes at the URDF's own copies (so each STL exists once), places the arm at Qualcomm's spawn pose, sets actuator gains, adds a `tcp` site, and couples the gripper ([Gripper](#gripper-couple-the-fingers)). `decimate_mesh` took Qualcomm's 403,096-face base mesh to 4,170; the mini base's 151,674-face mesh loads as it is.

**A mobile base needs more post-processing.** `urdf_to_mjcf` writes arm-shaped output: gravity compensation on every body and a position actuator on every joint, passive casters included. For a base, give the root body a `<freejoint>`, remove `gravcomp`, put velocity actuators on the drive wheels only (keep `armature / kv` below the timestep), and make casters frictionless spheres on ball joints. The public guide's "floating reference joint" step for mobile manipulators fails when the URDF root is already the base ("free joint can only be used on top level"); add the free joint in post-processing instead.

> [!NOTE]
> **Worked example:** `tools/postprocess_qrb_amr_mjcf.py` does this for the QRB robot base.

### Scene

- **Static collision geometry:** when the Gazebo world models collision with primitives, generate the MJCF from the SDF. `tools/gen_sdf_collision_mjcf.py <file.sdf> <model> <class> <mjcf name>` converts one model's boxes, cylinders, and spheres, with full roll-pitch-yaw, composing link poses so walls defined per link convert directly.
- **Worlds built from Fuel `include`s:** `tools/gen_sdf_includes_mjcf.py` flattens a world's includes through their nested model poses into one MJCF. OBJ models become a textured visual mesh plus their `*_Col.obj` collider; COLLADA models fall back to their own SDF primitives. An OBJ placed at its SDF pose lands where Gazebo put it: MuJoCo recenters each mesh on its centroid and moves the geom to compensate, so do not offset it by hand.
- **Look inside Fuel models before fitting collision by hand.** A model whose visual is one mesh may still carry primitive colliders in its `model.sdf`, which convert directly. Only when a model really collides with a mesh, fit primitives to it or decompose it into convex parts with `obj2mjcf`.
- **Formats MuJoCo rejects:** glTF (`.glb`: "no decoder found for mesh file"; convert it to OBJ with PNG textures, or draw the collision primitives as the visual) and JPEG textures ("Non-PNG texture, assuming custom binary file format"; convert to PNG). One MuJoCo mesh takes one material: for a multi-material OBJ, use the texture that covers the most faces.
- **Fuel licenses and downloads:** `curl https://fuel.gazebosim.org/1.0/<owner>/models/<name>` returns `license_name`; download with `…/<version>/files.zip`.
- **Graspable objects** are bodies with a `<freejoint>`, a primitive collision geom (a mesh's contact is worse than a cylinder's), and the mesh as visual only. Give each a `<site>`, so its pose is published on TF; that is how anything outside the simulation finds out where the object is.
- **Keyframe:** add a `default` keyframe with the robot's start configuration. Its `ctrl` vector has one value per actuator, so it changes length when actuators are removed.

> [!NOTE]
> **Worked example:** `warehouse.sdf`'s 40 shapes became `src/rml63_sim/description/warehouse.xml`. The Ionic world `include`s a single "Ionic interior" model whose visual is one glTF mesh, but whose `model.sdf` carries 183 primitive colliders; they became `src/qrb_amr_nav_sim/description/ionic.xml`, drawn with their collision shapes. The office world's 64 includes became `src/qrb_remote_assistant_sim/description/office_furniture.xml`. Picking an OBJ's first material instead of its largest (a book covering 30 of a seated child's 1544 faces) turned the child white. The can's site is `coke1_site` in `mj_world`.

### Sensors

#### Planar Lidar

A Gazebo `gpu_lidar` becomes one MuJoCo `<rangefinder>` per beam, which `MujocoSystem` publishes as a `sensor_msgs/LaserScan`:

- **Names:** every beam is a site named `<lidar>-<index>` (zero-padded) with its own rangefinder. Its `user` field is `noise angle_min angle_max angle_increment range_min range_max`. The scan is published in `<lidar>_ROS`, whose x axis points at beam 0, so `angle_min` is 0 there.
- **Write the beams out.** `<replicate>` fails in a model that has a keyframe in the MuJoCo version the Simulator ships, so generate the sites: `tools/add_rangefinder_lidar.py` takes Gazebo's sample count, angles, and ranges, using Gazebo's increment `(max - min) / (samples - 1)`.
- **Topic and rate:** a `<sensor name="<lidar>">` block in the `ros2_control` hardware with `<param name="topic">/scan</param>` sets the topic, and the hardware parameter `lidar_publish_rate` sets the rate.
- **A rangefinder ignores only its own body.** Move visual-only geoms that enclose the sensor into the lidar's body, keeping their pose (`add_rangefinder_lidar.py --exclude-geom`).
- **Rays are expensive.** Every beam is cast every physics step, whatever the publish rate (the rangefinder's `interval` attribute is accepted but does not throttle it), and each ray tests every geom, whatever its group, `contype`, or alpha (the rangefinder in the MuJoCo version the Simulator ships has no group filter). Use the Gazebo world's own `max_step_size` or fewer beams, and measure the real-time factor offline before going further. An image-based lidar (one camera with `lidar_output scan`, excluding the visual group) renders on the render thread instead of the physics step, but changes the sensor model.
- **An offline render full of yellow lines is not a scene bug.** MuJoCo's own viewer draws every rangefinder ray; the Simulator turns that off.

> [!NOTE]
> **Worked example:** every beam of the QRB mini's lidar first hit the base's visual mesh at 4 cm, because Gazebo's `range_min` near-clip had hidden it. Its 1800 beams cost 2.9 ms per step: 0.7× real time at a 2 ms step, 1.4× at the Ionic world's 4 ms, and 0.74× in the furnished office (1.46× with the furniture's visual meshes removed). The remote assistant port kept Qualcomm's ray lidar and its slower simulation.

#### Cameras

A robot camera from Gazebo carries over as a MuJoCo `<camera>` with Gazebo's resolution and its horizontal field of view converted to `fovy` (`tools/add_base_camera.py`). On the CPU image, cameras render in software (Mesa llvmpipe), and rendering competes with physics: set `render_publish_rate` to what the application needs, or use the CUDA image on a host with an NVIDIA GPU.

> [!NOTE]
> **Worked example:** llvmpipe took 214 ms of CPU per 1280×720 frame. Two cameras at `render_publish_rate` 15 needed 6.4 s of CPU per wall second and slowed the office scene from 0.74× to 0.34× real time; the remote assistant renders at 2 Hz, enough for its person check.

### Checkpoint

Load and step the scene in Python MuJoCo (`mujoco.MjModel.from_xml_path`, `mj_resetDataKeyframe`, `mj_step`), with no ROS involved. You should see that:

- objects rest where they spawn, and the robot holds its keyframe;
- the original's joint targets, driven through the actuators, reach and hold what they should (can the gripper reach and hold the object?);
- a base driven at the original's speeds moves at those speeds, measured against the body's pose;
- the real-time factor is above 1, or you know why it is not.

These checks take seconds, and they catch problems that would take much longer to find through the Runtime.

> [!NOTE]
> **Worked example:** `src/rml63_sim/test/grasp_check.py` drives Qualcomm's pick-and-place targets through the actuators and checks the can; `src/qrb_amr_nav_sim/test/offline_check.py` drives the base, reads the lidar, and times the simulation.

---

## Stage 4: Robot Configuration Package and Controllers

In this stage, the robot gets its MoveIt Pro configuration package and runs in the Desktop App. We do this before porting any behavior, because planning code and Objectives need a robot to run against.

### The Configuration Package

| File | Contents |
|---|---|
| `config/config.yaml` | `runtime_launch_file`, URDF/SRDF, `moveit_params`, `ros2_control`, behavior loader plugins, Objective paths, waypoints file |
| `config/moveit/` | SRDF and `joint_limits.yaml` (copied from the MoveIt 2 config, if there is one), kinematics, `pose_jog.yaml`, `joint_jog.yaml` |
| `config/control/<robot>.ros2_control.yaml` | Controllers, keeping the original controller names |
| `description/` | Xacro with a `MujocoSystem` `ros2_control` block, `scene.xml`, the robot MJCF, `assets/` |
| `launch/runtime.launch.xml` | Includes `$(find-pkg-share moveit_studio_agent)/launch/studio_agent_bridge.launch.xml` |
| `objectives/`, `waypoints/` | Objectives and named joint states ([Stage 5](#sequencing-code-becomes-an-objective)) |

From a MoveIt 2 application, the URDF and SRDF (planning groups, named group states), the `ros2_control` hardware and controller names, and the `joint_limits.yaml` values carry over unchanged.

`config.yaml` errors the ports hit:

- `hardware.simulated`, `robot_driver_persist_launch_file`, and `simulated_robot_driver_persist_launch_file` were removed (their values move to `additional_driver_launch_file`), and `runtime_launch_file` is required. Older reference configs can still carry them; the user workspace migration guide for the release that removed them lists the replacements.
- `moveit_params.pose_jog` and `joint_jog` are required ("Field required"), even for a robot with no jog controllers.
- An empty list anywhere in the resolved parameters (`planning_groups: []`, `controllers_inactive_at_startup: []`) crashes the launch with "Expected 'value' to be one of […] but got '()' of type tuple". Omit optional list keys rather than setting them to `[]`, and give the jog files a real group and controller.

### Arm: JointTrajectoryController → JointTrajectoryAdmittanceController (JTAC)

> [!NOTE]
> **What is the JTAC?** The JointTrajectoryAdmittanceController is MoveIt Pro's trajectory controller. It follows trajectories like a `JointTrajectoryController` and adds MoveIt Pro's speed scaling, stop handling, and, with a force-torque sensor, admittance control. MoveIt Pro's Objectives execute through it by default.

A plain `JointTrajectoryController` works on MoveIt Pro, but the platform's own controllers are what its Objectives, speed controls, and safety checks are built around. Moving an arm to the JTAC is a configuration change:

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

### Gripper: Couple the Fingers

A gripper modelled as independent joints with separate position commands works, but the fingers open at different rates, and MoveIt Pro's gripper Objectives expect one driven joint. Couple the fingers in three places, all generated from the same multipliers:

1. **URDF:** `<mimic joint="<driver>" multiplier="…" offset="…"/>` on each follower. Fit the multipliers through the gripper's open and closed SRDF states. Then bound the driving joint so that its full range, mapped through each mimic, stays inside the follower's own limits, and give each follower velocity and acceleration limits of at least |multiplier| times the driver's. MoveIt Pro rejects a robot description that breaks this ("Mimic joint '…' can violate its position limits"), and the Runtime does not start.
2. **MJCF:** remove the followers' actuators and add `<equality><joint joint1="<follower>" joint2="<driver>" polycoef="<offset> <multiplier> 0 0 0"/></equality>`. Update the keyframe's `ctrl` to the remaining actuators, and its finger `qpos` to values that satisfy the coupling.
3. **ros2_control:** keep a command interface for the driving joint only, and run `position_controllers/GripperActionController` on it with `allow_stalling: true`, so a close that stops on an object succeeds. That also means a successful close does not prove a grasp: a close on empty air succeeds too. Check the object's pose if the application must know.

Keep only the driving joint in the SRDF's gripper `group_state`s; the followers come from the mimic multipliers.

> [!NOTE]
> **Worked example:** Qualcomm's gripper has four independently commanded fingers, whose asymmetry reached 0.13 rad as they let go. `tools/postprocess_rml63_mjcf.py` has the fit; `joint01` is limited to [0, 0.2], its open-to-closed range, and the followers' rate limits are raised 1.25×. Afterwards the fingers stay mirrored to within 0.001 rad.

### Mobile Base

A differential-drive base needs no planner and no MoveIt 2 code; it needs a drive controller, the right frames, and teleoperation. `lunar_sim` is the reference for a differential-drive base, and `hangar_sim` for SLAM and Nav2.

**Drive controller.** Replace Gazebo's `gz-sim-diff-drive-system` plugin with `diff_drive_controller/DiffDriveController`:

- Carry over the wheel names, radius, separation, and any separation multiplier from the Gazebo xacro, and expect to recalibrate them. Measure against simulation ground truth, not `/odom`: odometry is computed from the same parameters, so it always agrees with the command. From MoveIt Pro 10.2, the Simulator publishes `<base_link>_ground_truth` under `mj_world`; use that frame, because `mj_world → base_footprint` resolves through the controller's odometry. Recalibrating stops being optional once the application closes a loop on odometry.
- Exactly one node owns `odom` → base frame. With wheel odometry alone, that is the controller (`enable_odom_tf: true`). With a SLAM node that provides the odometry frame (Cartographer's `provide_odom_frame = true`), set `enable_odom_tf: false`.
- Name it as `lunar_sim` does (`platform_velocity_controller`) and map its topics to plain `/cmd_vel` and `/odom` with `controller_shared_topics` in `config.yaml`.
- **Commands must be `TwistStamped`, and they expire.** The ROS 2 Jazzy controller accepts only `geometry_msgs/TwistStamped` and drops any command older than `cmd_vel_timeout` (0.5 s), so a `Twist` publisher does nothing and a single command stops the base after half a second. Gazebo's plugin took `Twist` and kept driving on the last command. To keep an original `Twist` node working unchanged, relay it through a node that stamps each message and, for a press-once teleop node, re-sends the last one at a fixed rate.

> [!NOTE]
> **Worked example:** `src/qrb_amr_sim/config/control/qrb_robot_base.ros2_control.yaml` replaces the plugin, and `src/qrb_amr_sim/scripts/twist_stamper.py` lets Qualcomm's unmodified `Twist` node drive. Qualcomm's 1.1525 separation multiplier turns about 9% faster than commanded in MuJoCo, where 1.05 matches. In the remote assistant, whose scripted route turns on `/odom`, Qualcomm's 1.125 made every 90° turn about 99° and drove the route into a wall; that port uses 1.05.

**Frames.**

- **SRDF:** declare a planar virtual joint from `odom` to the base frame, so the planning scene follows the base through the controller's TF. Put the drive wheels in a planning group (for example `base`); the Runtime needs one even though nothing plans for it.
- **Planning-scene objects** for a base are expressed in `odom`. That matches the MuJoCo world only while odometry has not drifted; for a robot that spawns at the origin, `odom` and the simulation world start out coinciding.
- **Simulator:** set `odom_frame` to `odom`, `publish_mj_world_tf` to `true`, and `base_link_name` on the `MujocoSystem` hardware block, as `lunar_sim` does. With SLAM, the tree's root is `map`, so set `odom_frame` to `map`: `MujocoSystem` then publishes `mj_world → map` as identity, and the scene camera hangs off `map`.
- **3D Visualizer:** it renders relative to `world` by default, and a base's TF tree has no `world` frame, so the Desktop App shows the scene camera but not the robot. Ship `config/frontend_settings.yaml` with `referenceFrame: odom` (or `map` with SLAM, together with the SRDF virtual joint's parent). A per-user `frontend_settings.yaml` saved earlier overrides it; changing the 3D Visualizer's Fixed Frame to `odom` once fixes that case.
- **A shaking base in the 3D Visualizer means two TF parents.** When the MJCF's free joint is on a body above `base_link_name`, MoveIt Pro releases before 10.2 take the base for a scene object and publish `mj_world → <base>` next to the odometry's `odom → <base>`. Check with `ros2 topic echo /tf | grep -B1 -A1 "child_frame_id: <base>"`: more than one parent `frame_id` is the bug. Set `<param name="publish_object_tf">false</param>` in the hardware block (it removes only free scene objects' TF); MoveIt Pro 10.2 and newer exclude the base's free body themselves, so only earlier releases need the parameter.

### Make the Scene Visible in the Desktop App

The 3D view shows the URDF and the planning scene, not the MuJoCo scene. A port that skips this shows the robot floating alone.

- **Scene camera:** add a `<camera>` and a `<site name="<camera>_optical_frame">` to the MJCF. The Simulator publishes `<camera>/color`, `/depth`, `/points`, and `/camera_info`, and the Desktop App shows them. The optical-frame site is the camera orientation rotated 180° about its x-axis: multiply the camera's `xyaxes` by `1 1 1 -1 -1 -1`.
- **Planning scene:** add the static furniture as collision objects in an Objective, so it is drawn and the planner avoids it. Generate the Objective from the compiled MJCF, not from the source XML or the Gazebo world: `tools/gen_scene_objective.py` loads the model with MuJoCo, reads each furniture geom's world pose and size, and writes `AddCollisionBox` and `AddCollisionCylinder` calls. Hand-transcribed poses drift from the compiled model (rotated tables that do not match the camera). Set `overwrite="true"` on those Behaviors, so the Objective can run more than once without "CollisionObject … already exists". Leave grasped objects out; they move.

> [!WARNING]
> **Rebuild after adding an Objective file:** The Runtime reads Objectives from the colcon install space, which links the files that existed at build time. A new `.xml` stays invisible ("Can't find a tree with name …") until you run `moveit_pro build user_workspace` again. Edits to existing files show up without a rebuild.

> [!NOTE]
> **Worked example:** `src/rml63_sim/objectives/qrb_load_scene.xml` and `src/qrb_amr_sim/objectives/qrb_amr_load_scene.xml` are generated this way.

### Teleoperation

A keyboard or joystick node that publishes velocity commands has no Behavior Tree counterpart to write: the Desktop App's teleoperation does its job. An arm teleoperates through MoveIt Pro's jog controllers, configured as `lab_sim` does (`pose_jog.yaml`, `joint_jog.yaml`, and the controllers they name). A differential-drive base today needs one custom Behavior, `TeleoperateBase`, from `lunar_sim_behaviors` in `moveit_pro_example_ws`:

1. Vendor `lunar_sim_behaviors` into the workspace unchanged (BSD-3-Clause; record the source commit) and load `lunar_sim_behaviors::LunarSimBehaviorsLoader` in `config.yaml`.
2. Override `Request Teleoperation` with `lunar_sim`'s version, which runs `TeleoperateBase` in Pose mode. Set its velocity limits to the original node's speeds.
3. Point `pose_jog.yaml` at the base group and the drive controller.

In the **Pose** tab, **X ±** drives and **Roll ±** turns: `TeleoperateBase` keeps only the jog command's `linear.x` and `angular.z`, and the Pose pad labels its `angular.z` buttons "Roll" because they are meant for an arm's end effector. The other buttons do nothing. Tell users this; the labels do not. If the installed release provides a core base-teleoperation mode (`moveit_params.base_jog`), use it instead of the vendored Behavior.

> [!WARNING]
> **Teleoperation needs a connected Desktop App:** `Request Teleoperation` cannot run without one; `DoTeleoperateAction`'s goal is rejected ("no UI tab is subscribed to /moveit_pro_ui/do_teleoperate/goal"). To check teleoperation from a script, run `TeleoperateBase` on its own with the same ports and publish `moveit_pro_controllers_msgs/VelocityForceCommand` on `/pose_jog/<group>`.

> [!NOTE]
> **Worked example:** `src/qrb_amr_sim/objectives/request_teleoperation.xml`.

### Checkpoint

Start MoveIt Pro:

```bash
moveit_pro run
```

You should see the robot in the Desktop App's 3D view and the scene camera's image. After you run the planning-scene Objective, you should see the furniture drawn around the robot.

In a MoveIt Pro shell (`moveit_pro shell`), list the controllers:

```bash
ros2 control list_controllers
```

You should see every controller listed as `active`. Send the original's commands straight to the controllers from the same shell, and the robot should move as it did in Gazebo. Teleoperation should move the robot too, and a base should drive at the original's speeds. The drivers' log should have no `[ERROR]` lines.

---

## Stage 5: Port the Behavior

In this stage, we port what the application does. Use the subsections your triage pointed to; each one ends with its own checkpoint.

### Sequencing Code Becomes an Objective

> [!NOTE]
> **What is an Objective?** An Objective is a Behavior Tree that MoveIt Pro runs: a tree of Behaviors (single actions such as "move to a waypoint" or "open the gripper") and Subtrees (other Objectives called by name). You can run, edit, and watch Objectives in the Desktop App.

A hand-written sequence of motions becomes an Objective. For a sequence of joint targets, no C++ Behavior is needed.

1. **Joint targets become waypoints.** Each hard-coded joint map, and each SRDF named state the code used, becomes an entry in `waypoints/<robot>_waypoints.yaml`. Generate the file from the original values (`tools/gen_waypoints.py`), so nothing is retyped.
2. **Each move becomes a Subtree call.** Wrap `Move to Named Waypoint` once with the robot's group, controller, and the original speed scaling, then call it with only `waypoint_name`. Override the core `Open Gripper` and `Close Gripper` placeholder Objectives for the robot's gripper controller; the core versions log an error and fail.
3. **Group the steps** into named `Sequence`s that match the original's phases (Prepare, Pick, Place), so the tree reads like the original code.
4. **Load the scene first:** call the planning-scene Objective at the start of the tree.

Before you write a Behavior, search the installed Behavior library for one that already does the job (the core library's waits, for example, include `WaitForDuration` and `WaitForUserDecision`). Where the original has a node that waits for an event, run that node ([Running the original's nodes unchanged](#running-the-originals-nodes-unchanged)) rather than writing a Behavior that waits.

> [!NOTE]
> **Worked example:** `src/rml63_sim/objectives/qrb_pick_and_place.xml`, with `qrb_move_arm.xml`, `open_gripper.xml`, and `close_gripper.xml` beside it:
>
> | `qrb_ros_arm_pick_place.cpp` step | `QRB Pick and Place` |
> |---|---|
> | — | `QRB Load Scene` |
> | 1. `moveToNamedTarget(arm, "ready")` | `QRB Move Arm` `ready` |
> | 2. `moveToNamedTarget(gripper, "open")` | `Open Gripper` |
> | 3. `moveToJointAngles(getPickJointAngles())` | `QRB Move Arm` `pick` |
> | 4. `moveToNamedTarget(gripper, "close")` | `Close Gripper` |
> | 5. `moveToJointAngles(getRaiseJointAngles())` | `QRB Move Arm` `raise` |
> | 6. `moveToJointAngles(getPlaceJointAngles())` | `QRB Move Arm` `place` |
> | 7. `moveToNamedTarget(gripper, "open")` | `Open Gripper` |
> | 8. `moveToJointAngles(getReRaiseJointAngles())` | `QRB Move Arm` `re_raise` |
> | 9. `moveToNamedTarget(arm, "ready")` | `QRB Move Arm` `ready` |

#### Checkpoint

Run the Objective from the Desktop App. You should see it finish with `SUCCEEDED`, and the robot should do what the original did. To run it without the Desktop App:

```bash
ros2 action send_goal /do_objective moveit_studio_sdk_msgs/action/DoObjectiveSequence "{objective_name: '<Objective name>'}"
```

### MoveIt 2 Code Plans In-Process

Keep the original node when the application's own code is part of what you are porting. MoveIt Pro ships the MoveIt libraries under its own names and replaces the `move_group` node and its planners; code that called `MoveGroupInterface` plans in-process instead. The public guide **Migrate from MoveIt 2 to MoveIt Pro Core** covers package and namespace renames and the OMPL → ProRRT and Pilz → Cartesian-planning replacements; this section covers what it does not: `MoveGroupInterface` and `move_group`.

| MoveIt 2 | MoveIt Pro |
|---|---|
| `MoveGroupInterface(node, group)` | `moveit_pro::base::RobotModel` built from the URDF and SRDF strings, plus `moveit_pro::planning_scene_monitor::CurrentStateMonitor` on `/joint_states` for the start state |
| `setNamedTarget("ready")` | `RobotState::setToDefaultValues(group, "ready")` reads the SRDF `group_state` |
| `setJointValueTarget(map)` | `RobotState::setVariablePositions(map)` on a copy of the start state |
| `setPlannerId("RRTConnect…")`, `setPlanningTime` | `pro_rrt::RRTParams` |
| `setMaxVelocity/AccelerationScalingFactor` | `pro_rrt::TrajectoryParams::velocity_scale_factor` / `acceleration_scale_factor` |
| `plan()` | `pro_rrt::planTrajectoryToJointGoal(group, start, goal, rrt_params, constraints, trajectory_params, scene)`, returning a `trajectory_msgs/JointTrajectory` or a `PlanningError` with a readable `message` |
| `execute()` (through `move_group`) | An action client on the controller: `FollowJointTrajectoryWithAdmittance` for the JTAC ([Arm](#arm-jointtrajectorycontroller--jointtrajectoryadmittancecontroller-jtac)), `control_msgs/GripperCommand` for a gripper |
| `move_group` node, `moveit_controllers.yaml`, `MoveItConfigsBuilder` launch | Removed |

What `MoveGroupInterface` did for you, and you now do yourself:

- **Apply acceleration limits.** `move_group` loaded `joint_limits.yaml` and applied its acceleration limits to the robot model. Nothing does that for in-process planning, and URDF has no acceleration limits, so time parameterization fails with "No acceleration limit was defined for joint …". Set `acceleration_bounded`, `min_acceleration`, and `max_acceleration` on each joint's `VariableBounds` from the file's values.
- **Give the state monitor its own executor.** `MoveGroupInterface` spun a private executor to receive `/joint_states`. If the main node sits in `rclcpp::spin_until_future_complete` waiting on an action, a `CurrentStateMonitor` on that node never receives a message, and `waitForCompleteState` times out. Give the monitor a second node spun on a background thread. Own that thread with `std::jthread` and stop the executor in an object destroyed before it: a plain `std::thread` member still joinable when the constructor throws calls `std::terminate`, and the node dies with SIGABRT instead of printing its error.
- **Cancel what you stop waiting for.** `MoveGroupInterface::execute` cancelled its goal when interrupted. A hand-written action client that times out waiting for a result must call `async_cancel_goal` on the accepted goal, or the controller finishes the motion with nothing supervising it.
- **Validate the robot model.** Use `RobotModel::create(urdf, srdf)` rather than the `RobotModel` constructor (`create` requires MoveIt Pro 10.2 or newer). `create` validates the model, including mimic joint limits, and returns the reason it was rejected; the constructor skips that check, so a model the Runtime would refuse loads silently in the application.
- **Mimic joints in named states.** With a coupled gripper, keep only the driving joint in its SRDF `group_state`s; `setToDefaultValues` sets the followers through the URDF `<mimic>` multipliers, and the node sends the single driving value to the gripper controller.

Building:

- Add `moveit_pro_base`, `moveit_pro_planning_scene_monitor`, and `pro_rrt` as dependencies (plus `moveit_pro_controllers_msgs` for the JTAC). Remove `moveit_ros_planning_interface` and `moveit_msgs` if nothing else uses them.
- `rosdep` has no keys for MoveIt Pro packages; it finds them only with the MoveIt Pro environment sourced, as `moveit_pro build user_image` does. Outside it, `rosdep install` fails with "Cannot locate rosdep definition for [pro_rrt]" unless they are listed in `--skip-keys`.

> [!NOTE]
> **Worked example:** `src/qrb_pick_place_core` wraps planning in one small class, `MotionPlanner` (`src/qrb_pick_place_core/include/qrb_pick_place_core/motion_planner.hpp`), and keeps Qualcomm's step structure, joint maps, and log text in `src/qrb_pick_place_core/src/qrb_ros_arm_pick_place.cpp`, so the diff against the original reads one-for-one. Its launch file passes `joint_limits.yaml` in as parameters; the state monitor runs on `state_node_` / `state_executor_` / `state_thread_`; both the arm and gripper goals are cancelled after `result_timeout_s`. Qualcomm's QIRP SDK cross-compile path, which hard-codes sysroot include directories for each MoveIt 2 package, was not ported; a QIRP build needs the MoveIt Pro packages in the Yocto sysroot first.

#### Checkpoint

With MoveIt Pro running, start the ported node from a Runtime shell:

```bash
docker exec -it -u $USER <Runtime container name from docker ps> bash -c \
  'source /etc/skel/.moveit-pro-bashrc && ros2 launch <package> <launch file>'
```

You should see the node run its sequence, and its launch log should end with "process has finished cleanly". Judge the run by that line, not by the launch command's exit code, which is 0 even when the node dies.

### Navigation: SLAM and Nav2

A navigation sample is often a set of commands run next to Gazebo: a SLAM node, then `nav2_bringup`'s `navigation_launch.py`, then RViz's "2D Goal Pose". All three move into the robot configuration package:

- **Launch them as drivers.** `hardware.additional_driver_launch_file` in `config.yaml` names a launch file that starts with the Simulator. Declare `cartographer_ros` (or your SLAM package) and `nav2_bringup` as `exec_depend`s: the MoveIt Pro images do not ship Nav2, SLAM Toolbox, AMCL, or Cartographer, and `rosdep` installs them when the user image is rebuilt (`moveit_pro build user_image`).
- **Keep the sample's parameter files.** Copy its SLAM configuration and `nav2_params.yaml` unchanged, and make every needed change in the launch file.
- **`use_sim_time` is false.** MoveIt Pro runs on wall time and the Simulator publishes no `/clock`; a sample's `use_sim_time:=true` was for Gazebo.
- **Nav2 must publish `TwistStamped`.** Jazzy's Nav2 publishes `Twist` unless each node that sends velocity sets `enable_stamped_cmd_vel` (`controller_server`, `velocity_smoother`, `collision_monitor`, `behavior_server`, `docking_server`). Nav2's `RewrittenYaml` adds a missing key only when given its full path, `<node>.ros__parameters.enable_stamped_cmd_vel`; a bare key changes only keys already in the file. The collision monitor's output, `/cmd_vel`, then reaches the drive controller with no relay.
- **The goal comes from an Objective.** `GetPoseFromUser` and `NavigateToPoseAction` on `/navigate_to_pose` replace RViz's "2D Goal Pose". Load `moveit_pro::behaviors::NavBehaviorsLoader`. Mapping teleoperation is the Desktop App's teleoperation.
- **Watch for velocity publishers that share `/cmd_vel`.** Before adding a node that publishes `Twist` on `/cmd_vel` next to Nav2 and the drive controller, list the topic's publishers (`ros2 topic info -v /cmd_vel`). Two message types on one topic is a type clash, and an extra publisher of zeros (Nav2's idle `collision_monitor`, for example) can halve the commanded speed. Remap each `Twist` node to its own topic and stamp it on the way in. A relay that re-sends the last command is wrong for a node that publishes every control tick itself (`twist_stamper.py` takes `republish 0` for that case).

**Cartographer: upstream or a vendor fork.** Upstream Cartographer (`ros-jazzy-cartographer-ros`) builds for every architecture and installs through `rosdep`. A vendor fork's files usually run on it with small changes in a wrapper rather than in the vendor's files: pass `-configuration_directory` and `-configuration_basename` (a fork may build its default in), and remove fork-only Lua options, which upstream aborts on ("Key '…' was used the wrong number of times") with a two-line wrapper that `include`s the vendor's file and sets the option to `nil`.

> [!NOTE]
> **Worked example:** `src/qrb_amr_nav_sim` starts Cartographer and Nav2 from `launch/navigation_drivers.launch.py`, keeps the sample's Lua and Nav2's default parameters in `config/cartographer/` and `config/nav2/`, and replaces "2D Goal Pose" with `objectives/navigate_to_clicked_pose.xml`. Qualcomm's samples use their Cartographer fork (`ros-jazzy-qcom-cartographer*` from `ppa:ubuntu-qcom-iot/qirp`), which publishes arm64 packages only and adds a `SlamCommand` map save/load service. The fork's `cartographer_node` builds its Lua file's path in, so Qualcomm's launch file passes no `-configuration_directory` or `-configuration_basename`. Its Lua sets the fork-only `save_pose_to_scv`, so `config/cartographer/qrb_amr_mapping_2d_upstream.lua` is `include "qrb_amr_mapping_2d.lua"` plus `options.save_pose_to_scv = nil`. `src/qrb_amr_nav_sim/README.md` describes switching to the fork, for example to build on the IQ-9.

#### Checkpoint

Drive the base around with teleoperation. You should see the map grow in the Desktop App. Then run the goal Objective and click a goal: the base should drive there.

### Running the Original's Nodes Unchanged

Sometimes the right port runs the original's nodes byte-identical, so the same nodes run against the simulation now and against the vendor's own stack on its hardware later. Everything the port needs then goes at the nodes' interfaces.

- **Vendor the packages and record where they came from.** Copy them unchanged and write a `VENDORED.md` with the source repository, commit, and license, and every change made *around* them (`COLCON_IGNORE`, `colcon.pkg`, skipped `rosdep` keys); none go inside their files. A message package published only as a binary for one architecture builds from its interface files with a new `CMakeLists.txt`.
- **Shim a vendor-only service.** When the nodes call a service only the vendor's stack serves, write a node that serves the same interface over the upstream equivalents, and give it a timeout, so a caller that polls it cannot spin forever. On the vendor's hardware, a launch argument switches the shim off.
- **Stand in for a model that needs the vendor's NPU.** Keep the model's output topic as the seam, and publish the same message from a MoveIt Pro Objective (SAM3 segmentation, for example) through a small bridge node. Check the stand-in's latency against the original's timeouts.
- **Re-run a launcher script's sequence** from the robot configuration package's driver launch, in the same order and with the same commands to the original nodes, when the original's launcher cannot run as it is (it hard-codes `use_sim_time:=true`, or a launch file only the vendor ships).
- **Intercept a subprocess a node starts itself** with a wrapper ahead of the real `ros2` on that node's `PATH` only, which runs the port's launch for the one command it matches and passes every other command through.
- **Keep the original's start order.** Start each subsystem where the original does, not at launch. Starting a second instance next to a running one is not a fallback: duplicate-named lifecycle nodes sit unconfigured, and their publishers clash.
- **Use the original's own end-to-end check.** If the original has a node that decides "done", run it: when it reports success, the migration works as the original defined it, with no new Behavior and no reimplemented success criterion.

> [!NOTE]
> **Worked example:** `src/qrb_remote_assistant_sim` runs Qualcomm's `build_map_node`, `nav_preparation_node`, and `task_manager_node` from a vendored copy (`src/simulation_remote_assistant`). `scripts/slam_command_shim.py` serves `/qrb_slam_command` (`SlamCommand` 0–5: start, finish, save, load, start localization, validate relocalization) over upstream Cartographer's `finish_trajectory`, `write_state`, `-load_state_filename` with a localization `start_trajectory`, and, for "relocalized", the first `/constraint_list` constraint between different trajectories; it fails with `DEADLINE_EXCEEDED` after a timeout, and `slam_backend:=qualcomm` switches it off. `src/qrb_ros_slam_msgs` builds Qualcomm's arm64-only `ros-jazzy-qrb-ros-slam-msgs` from source. The person check is YOLOv8 compiled for the IQ-9's HTP (`yolov8_det_qcs9075.bin`), publishing `vision_msgs/Detection2DArray` on `/yolo_detect_result`; the `QRB Detect Person` Objective runs SAM3 (`GetMasks2DFromExemplar`) with the prompt "person" and `scripts/detection_bridge.py` publishes the count as that message (`detector:=qualcomm` switches back). SAM3 on the CPU takes about 30 s per frame and `task_manager_node` waits only 10 s after arrival, so the Objective loops from before the robot arrives. `launch/remote_assistant_pipeline.launch.py` re-runs Qualcomm's `map_nav_setup.py` sequence on wall time, and `scripts/nav2_on_wall_time/ros2` catches `nav_preparation_node`'s own `ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true`. Nav2 starts after relocalization, as in the original, so nothing but Qualcomm's nodes publishes velocity during the scripted mapping drive.

#### Checkpoint

Run the original's own end-to-end check. You should see it report success in the Simulator.

> [!NOTE]
> **Worked example:** `task_manager_node`, given its own prompt `go to office to check person`, reports "Navigation target reached" and receives its detection.

---

## Stage 6: Hand It Over

In this final stage, we make sure the port can be run, and judged, by someone else.

1. **Write down the known issues.** List every defect of the original that shows up in MuJoCo, with what it does, how you measured it, and the fix you did not apply. List every value you changed and why. [Appendix A](#appendix-a-the-worked-examples) shows the form.
2. **Document the run steps** in the package README, including how to switch to the vendor's own stack on its hardware.
3. **Hand someone a running session.** Start the Objective or a teleoperation session in the Desktop App and have someone else watch it, or drive it. They will see problems a command-line check cannot: a missing scene, a robot not drawn in the 3D view, wrong geometry, unclear controls.

### Checkpoint

Ask someone who did not write the port to start it from the README. They should see the original behavior run, with the scene visible.

---

## Summary

In this guide, we moved a ROS 2 application onto MoveIt Pro in six stages, each ending in a checkpoint you could see:

- A triage of what the application uses, and the MoveIt Pro reference for each part.
- A workspace with a robot configuration package that builds.
- The robot and its world rebuilt in the MoveIt Pro Simulator, checked offline.
- The robot running in the Desktop App on MoveIt Pro's controllers, visible in its scene and moving under teleoperation.
- The original's behavior running as an Objective, as a ported node, or as the original's own nodes.
- A port that someone else can run, with its known issues written down.

Congratulations, your application now runs on MoveIt Pro! To see these stages applied end to end, continue to [Appendix A: The Worked Examples](#appendix-a-the-worked-examples).

---

## Troubleshooting

### Symptom Index

| Symptom | Where |
|---|---|
| `number of faces should be between 1 and 200000` | [Robot model](#robot-model) |
| `free joint can only be used on top level` | [Robot model](#robot-model) (mobile base) |
| MuJoCo bodies "do not sit where the URDF places" links | [Robot model](#robot-model) (world pose) |
| `Non-PNG texture`, or `no decoder found for mesh file` (`.glb`) | [Scene](#scene) |
| Every lidar beam reads a few centimeters | [Planar lidar](#planar-lidar) |
| Simulation runs slower than real time with a lidar or cameras | [Planar lidar](#planar-lidar), [Cameras](#cameras) |
| `Field required`, or `got '()' of type tuple`, in `config.yaml` | [The configuration package](#the-configuration-package) |
| JTAC `sensor_frames must be specified` | [Arm](#arm-jointtrajectorycontroller--jointtrajectoryadmittancecontroller-jtac) |
| `Mimic joint '…' can violate its position limits` | [Gripper](#gripper-couple-the-fingers) |
| Base ignores `cmd_vel`, or stops after half a second | [Mobile base](#mobile-base) (drive controller) |
| Turns overshoot or a scripted route drifts, while odometry agrees with the command | [Mobile base](#mobile-base) (drive controller); [simulation speed](#environment-and-tooling) |
| Base visible in the scene camera but not the 3D Visualizer | [Mobile base](#mobile-base) (frames) |
| Base shakes or flickers in the 3D Visualizer | [Mobile base](#mobile-base) (two TF parents) |
| Desktop App shows only the robot | [Make the scene visible](#make-the-scene-visible-in-the-desktop-app) |
| `CollisionObject … already exists` | [Make the scene visible](#make-the-scene-visible-in-the-desktop-app) |
| `Can't find a tree with name` | [Make the scene visible](#make-the-scene-visible-in-the-desktop-app) (rebuild) |
| `Rejecting /do_teleoperate goal: no UI tab is subscribed` | [Teleoperation](#teleoperation) |
| `No acceleration limit was defined for joint` | [MoveIt 2 code](#moveit-2-code-plans-in-process) |
| State monitor times out; `/joint_states` never arrives | [MoveIt 2 code](#moveit-2-code-plans-in-process) (executor); [`docker exec`](#environment-and-tooling) |
| Node dies with SIGABRT on a startup error | [MoveIt 2 code](#moveit-2-code-plans-in-process) (`std::jthread`) |
| `Cannot locate rosdep definition for [pro_rrt]` | [MoveIt 2 code](#moveit-2-code-plans-in-process) (building) |
| Nav2 runs but the base never moves | [Navigation](#navigation-slam-and-nav2) (`enable_stamped_cmd_vel`) |
| `Key '…' was used the wrong number of times` (Cartographer) | [Navigation](#navigation-slam-and-nav2) |
| Two message types on `/cmd_vel`, or the base moves at half speed | [Navigation](#navigation-slam-and-nav2) |
| An original node sits at time 0 or waits forever | [Running the original's nodes unchanged](#running-the-originals-nodes-unchanged); [no `/clock`](#environment-and-tooling) |
| `x509: certificate signed by unknown authority` pulling images | [Environment and tooling](#environment-and-tooling) |
| `no rosdep rule` for a ROS package | [Environment and tooling](#environment-and-tooling) |

### Environment and Tooling

- **`docker exec` into the Runtime container needs the bashrc.** The container's own environment leaves `CYCLONEDDS_URI` empty; only `/etc/skel/.moveit-pro-bashrc` (inside the image) sets it, so a node started from a plain `docker exec` shell never discovers `/joint_states`. Run `source /etc/skel/.moveit-pro-bashrc` first, or use `moveit_pro shell`.
- **`ros2 launch` exits 0 when its node dies.** Judge a run by the node's own exit line ("process has finished cleanly" vs. "process has died").
- **There is no `/clock`.** Any `use_sim_time:=true` in the original, including inside a subprocess a node launches itself, leaves that node at time 0.
- **Simulation speed varies.** The Simulator runs at a different real-time factor with physics alone, with sensors, and with cameras rendering in the full Runtime. An application that closes loops on wall-clock timers behaves differently at each speed; measure the real-time factor in the setup you will demonstrate in.
- **SAM3:** `moveit_pro configure --sam3-license` acts only on a clone of `moveit_pro_example_ws`. Elsewhere, vendor `moveit_pro_sam3` and record the Meta SAM License acceptance in its `VENDORED.md`. On the CPU it takes about 30 s per frame, and the Objective server held 16 GB RSS once the model was loaded.
- **Moving to a newer release mid-port:** upgrade the CLI and images together, rebuild the user workspace from clean, and re-run every checkpoint. A newer release can make workarounds unnecessary and can change TF paths (see [Mobile base](#mobile-base)). Mark each workaround with the release that needs it, so it can be found and removed.
- **TLS-inspecting corporate networks** re-sign Docker Hub's certificate with the gateway's own CA, so image pulls and builds fail with "x509: certificate signed by unknown authority" (check the issuer with `openssl s_client -connect registry-1.docker.io:443`). Pull through a registry mirror: add `"registry-mirrors": ["https://mirror.gcr.io"]` to `/etc/docker/daemon.json` and restart Docker. `docker manifest inspect` ignores the mirror, so test with `docker pull`. Some tags, such as the CUDA images, can be incomplete on the mirror; `MOVEIT_TARGET= moveit_pro build user_image` (empty) builds on the CPU base image instead.
- **`rosdep` reports "no rosdep rule" when `ROS_DISTRO` is unset**, outside a sourced ROS environment. Set `ROS_DISTRO=jazzy` before concluding a package has no rule.

---

## Appendix A: The Worked Examples

This repository holds one robot configuration package per ported sample; switch between them with `moveit_pro configure -c <package>`, then `moveit_pro build user_workspace` and `moveit_pro run`. `qrb_amr_nav_sim` and `qrb_remote_assistant_sim` need Cartographer and Nav2 in the user image: run `moveit_pro build user_image` first. The `tools/` directory holds the generators and post-processing scripts cited above.

Each port keeps the original's sequence, targets, and parameters; its job is to prove the migration, not to fix the demo. Values that break the original's own sequence in MuJoCo are changed and listed as such.

### Pick and Place (`simulation_sample_pick_and_place`)

An RML-63 arm on MoveIt 2 picks a can from a table and places it on a box.

**Run it:** configure `-c rml63_sim`, then run `QRB Pick and Place` in the Desktop App; Run `Reset MuJoCo Sim` to put the can back. To run the ported node instead, start `qrb_pick_place_core`'s `pick_place_core.launch.py` from a Runtime shell ([MoveIt 2 code](#moveit-2-code-plans-in-process)).

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

**Known issues:**

- **The can is knocked over after release in roughly one run in four.** `re_raise` does not lift the gripper straight up: the tool moves 16 cm sideways while rising 20 cm, and for the first 40% of that motion the open fingers sweep through the released can's footprint (clearance −0.016 to −0.041 m, measured by `src/rml63_sim/test/release_sweep.py`). The can lands upright on the box and is then clipped. A vertical retreat pose after the release would fix it: (−31.0, −44.68, −40.52, 0, −15.8, 0)° lifts the tool 10 cm straight up. Not applied, to keep the sequence identical. Whether it also happens in Qualcomm's Gazebo setup is untested.
- **The ported node does not plan around the tables; the Objective does.** Every target is a hard-coded joint configuration that happens to clear them. `QRB Load Scene` puts the tables in the planning scene, so the Objective's plans avoid them, but the node plans in an empty scene (self-collision only), as Qualcomm's original did.

### Simple Motion (`simulation_sample_amr_simple_motion`)

A keyboard node drives the QRB robot base in a warehouse.

**Run it:** configure `-c qrb_amr_sim`, run `QRB AMR Load Scene`, and drive from teleoperation's **Pose** tab (**X ±** drives, **Roll ±** turns).

| Path | Role |
|---|---|
| `src/qrb_amr_sim/config/config.yaml` | Robot configuration package for the QRB robot base |
| `src/qrb_amr_sim/config/control/qrb_robot_base.ros2_control.yaml` | `diff_drive_controller` replacing Gazebo's diff-drive plugin |
| `src/qrb_amr_sim/description/scene.xml`, `qrb_robot_base.xml` | Base in the warehouse, with a scene camera |
| `src/qrb_amr_sim/objectives/request_teleoperation.xml` | Pose-tab teleoperation for the base |
| `src/qrb_amr_sim/objectives/qrb_amr_load_scene.xml` | Generated warehouse planning-scene objects |
| `src/qrb_amr_sim/scripts/twist_stamper.py` | Lets Qualcomm's unmodified `Twist` node drive the controller |
| `src/lunar_sim_behaviors/` | Vendored `TeleoperateBase` |

**Known issues:**

- **Turns run about 9% fast** with Qualcomm's wheel separation multiplier (1.1525) in MuJoCo; 1.05 matches the commanded rate. Kept at Qualcomm's value.
- **The base stops when commands stop**, 0.5 s after the last one, where Gazebo kept driving on the last command. This is `diff_drive_controller`'s safety behavior and is kept; the relay that lets the original node run re-sends its last command to keep press-once-to-drive for that node only.

### Navigation (`simulation_amr_navigation`, with `simulation_2d_lidar_slam`)

The QRB robot base mini maps the Ionic world with Cartographer and drives to goals with Nav2.

**Run it:** configure `-c qrb_amr_nav_sim`. Drive around in teleoperation to build the map, then run `Navigate to Clicked Pose` and click a goal on the floor.

| Path | Role |
|---|---|
| `src/qrb_amr_nav_sim/config/config.yaml` | Robot configuration package for the QRB robot base mini with SLAM and Nav2 |
| `src/qrb_amr_nav_sim/launch/navigation_drivers.launch.py` | Cartographer and Nav2, started with the drivers |
| `src/qrb_amr_nav_sim/config/cartographer/`, `config/nav2/` | The sample's Cartographer Lua and Nav2's default parameters, unchanged, plus the upstream wrapper |
| `src/qrb_amr_nav_sim/description/scene.xml`, `ionic.xml`, `qrb_robot_base_mini.xml` | Base with its 1800-beam lidar in the Ionic world |
| `src/qrb_amr_nav_sim/objectives/navigate_to_clicked_pose.xml` | Replaces RViz's "2D Goal Pose" |
| `src/qrb_amr_nav_sim/README.md` | Switching to Qualcomm's Cartographer fork |

**Known issues:**

- **Turns run about 9% fast** here too: the mini base's 1.125 separation multiplier gives 1.094 rad/s for 1.0 commanded. Kept at Qualcomm's value.
- **The room is drawn with its collision shapes**, not the Fuel model's textured mesh, which MuJoCo cannot load.

### Remote Assistant (`simulation_remote_assistant`)

The base maps Qualcomm's office world on a scripted route, saves and reloads the map, relocalizes, navigates to a named location with Nav2, and checks for a person, all with Qualcomm's three nodes unchanged.

**Run it:** configure `-c qrb_remote_assistant_sim` and run. The mapping drive starts on its own; wait for "Relocalization successful, starting navigation" in the drivers log. Run `QRB Detect Person` in the Desktop App, then in a Runtime shell run `ros2 run simulation_remote_assistant task_manager_node` and type `go to office to check person`.

| Path | Role |
|---|---|
| `src/qrb_remote_assistant_sim/config/config.yaml` | Robot configuration package for the remote assistant |
| `src/qrb_remote_assistant_sim/launch/remote_assistant_pipeline.launch.py` | `map_nav_setup.py`'s sequence around Qualcomm's unchanged nodes |
| `src/qrb_remote_assistant_sim/scripts/slam_command_shim.py` | `/qrb_slam_command` over upstream Cartographer |
| `src/qrb_remote_assistant_sim/scripts/nav2_on_wall_time/ros2` | Runs the wall-time Nav2 launch when `nav_preparation_node` starts Nav2 |
| `src/qrb_remote_assistant_sim/scripts/detection_bridge.py`, `objectives/detect_person.xml` | SAM3 person check in place of QNN YOLO |
| `src/qrb_remote_assistant_sim/description/scene.xml`, `office_walls.xml`, `office_furniture.xml` | The office world, generated from `office.sdf` |
| `src/qrb_remote_assistant_sim/README.md` | Running it, and switching to Qualcomm's stack on the IQ-9 |
| `src/simulation_remote_assistant/`, `src/qrb_ros_slam_msgs/` | Qualcomm's nodes and `SlamCommand` messages, vendored |
| `src/moveit_pro_sam3/` | Vendored SAM3 models package |

**Known issues:**

- **Changed: `wheel_separation_multiplier` 1.05**, not Qualcomm's 1.125. `build_map_node` closes each turn on odometry, and with 1.125 each 90° turn is about 99° true, which drives the route into a station wall.
- **Changed: `control_period` 0.05 s**, not 0.2. Qualcomm's README gives this remedy "if the robot overshoots turns and hits walls"; at 0.74× real time, a 0.2 s wall-clock loop overshoots each turn by about 4°, and the route clears one wall with a 4.4° heading budget.
- **The mapping route is timing-sensitive.** It depends on the Simulator's speed, so the route and saved map differ between runs in the full Runtime. In Gazebo at 1.0× real time, the same arithmetic gives about 3° of overshoot per turn.
- **The Simulator runs below real time** in the office (0.74× with physics alone, 0.34× in the full Runtime with two cameras rendering) with Qualcomm's 1800-beam lidar, kept as is; Cartographer and Nav2 run on wall time.
- **Cameras render at 2 Hz** on the CPU image (`render_publish_rate`), enough for the person check.
- **Validated setups:** `task_manager_node`'s end-to-end run ("Navigation target reached", then the detection) passed during the port in a run with only the Simulator, controllers, and pipeline (no other Runtime nodes); SAM3 detection passes on saved frames (one person at the office goal, none at an empty wall) and runs live in the Runtime.

---

## Appendix B: Survey of `qrb_ros_samples`

How every Qualcomm sample triaged (dependencies from each `package.xml`, stacks from source and launch files):

| Sample | Uses | Triage rows |
|---|---|---|
| `robotics/simulation_sample_pick_and_place` | MoveIt 2 `MoveGroupInterface`, OMPL, Gazebo | MoveIt 2. **Ported** |
| `robotics/sample_robot_arm_grasping_with_pose_estimation` | Vendor RM arm API (no MoveIt), DenseFusion + YOLO via ONNX | Arm driven another way, perception |
| `robotics/simulation_amr_navigation` | Nav2 (`NavigateToPose`), Gazebo; runs on top of `simulation_2d_lidar_slam` | Mobile base. **Ported** |
| `robotics/simulation_2d_lidar_slam` | Cartographer (Qualcomm's fork), Gazebo, keyboard teleop | Mobile base. **Ported, with `simulation_amr_navigation`** |
| `robotics/simulation_remote_assistant` | Qualcomm's Cartographer fork (`SlamCommand` service), Nav2 (`NavigateToPose`), QNN YOLO, Gazebo | Mobile base, perception, original nodes unchanged. **Ported** |
| `robotics/simulation_sample_amr_simple_motion` | Keyboard teleop publishing raw `cmd_vel`, Gazebo diff-drive plugin | Mobile base, bespoke teleoperation. **Ported** |
| `robotics/simulation_follow_me`, `sample_followme` | Person tracking to raw `cmd_vel`, Gazebo | Mobile base, perception |
| `robotics/sample_apriltag` | `apriltag_ros`, `qrb_ros_camera` | Perception |
| `robotics/sample_3d_lidar_semantic_segmentation`, `sample_fall_detection` | `qrb_ros_nn_inference`, camera | Perception |
| `ai_vision/*` | `qrb_ros_nn_inference` + `qrb_ros_camera` | Perception; no robot, so the value is Behaviors and the Desktop App |
| `ai_audio/*` | Speech recognition, TTS | No MoveIt Pro counterpart today |

---

## Appendix C: Asset Licenses

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
| Office world Fuel models (people, furniture, `Coke`, `MonitorAndKeyboard`) | CC-BY-4.0 or CC0-1.0 |
| `moveit_pro_sam3` package (`moveit_pro_example_ws`) | BSD-3-Clause |
| SAM 3 ONNX models it downloads | Meta SAM License (use restrictions; each user accepts it before building, see `src/moveit_pro_sam3/README.md`) |

Each vendored package records its source commit and license in its `VENDORED.md`, and the office world's Fuel models are listed in `src/qrb_remote_assistant_sim/description/assets/LICENSES.md`.
