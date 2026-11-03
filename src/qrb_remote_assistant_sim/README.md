# qrb_remote_assistant_sim

MoveIt Pro robot configuration package for Qualcomm's `simulation_remote_assistant` sample (`qrb_ros_samples/robotics/simulation_remote_assistant`): the QRB robot base mini maps Qualcomm's office world on a scripted route, saves and reloads the map, relocalizes, and navigates with Nav2 to a named location to check for a person, in the MoveIt Pro Simulator instead of Gazebo.

Qualcomm's three nodes (`build_map_node`, `nav_preparation_node`, `task_manager_node`) run unchanged from `src/simulation_remote_assistant`. The driver launch (`launch/remote_assistant_pipeline.launch.py`) runs the sequence their `scripts/map_nav_setup.py` runs.

| Sample step | Here |
|---|---|
| `gazebo_robot_base_mini.launch.py world_model:=office initial_x:=1.0 initial_y:=6.0` | `description/scene.xml`, started by `moveit_pro run` |
| `map_nav_setup.launch.py` (mapping, map save, relocalization, Nav2) | the driver launch, started with the Simulator |
| `yolo_detectcion.launch.py` with `yolov8_det_qcs9075.bin` | the `QRB Detect Person` Objective (SAM3) and `detection_bridge.py` |
| `ros2 run simulation_remote_assistant task_manager_node`, then "go to office to check person" | the same command, unchanged |

## Running it

```bash
moveit_pro configure -w <path to qrb_moveit_pro_ws> -c qrb_remote_assistant_sim
moveit_pro build user_workspace
moveit_pro run
```

Wait for "Relocalization successful, starting navigation" in the drivers log; the mapping drive runs first. Run **QRB Detect Person** in the Desktop App, then in a Runtime shell:

```bash
ros2 run simulation_remote_assistant task_manager_node
# What can I help you with?
go to office to check person
```

## Switching to Qualcomm's stack on the IQ-9

Two parts stand in for Qualcomm packages that are not available off their hardware. Each one is switched off by a launch argument and its Qualcomm counterpart is installed or launched in its place.

| Here | On the IQ-9 | Switch |
|---|---|---|
| `scripts/slam_command_shim.py` serves `/qrb_slam_command` over upstream Cartographer | Qualcomm's Cartographer fork serves it itself | `slam_backend:=qualcomm`, and install the fork as `qrb_amr_nav_sim/README.md` describes |
| SAM3 (`QRB Detect Person`) and `detection_bridge.py` publish `/yolo_detect_result` | Qualcomm's YOLO pipeline (`qrb_ros_nn_inference`, `yolov8_det_qcs9075.bin` from Qualcomm AI Hub) | `detector:=qualcomm`, then launch their `yolo_detectcion.launch.py` as the sample's README does |

`src/qrb_ros_slam_msgs` is Qualcomm's message package built from source, because their PPA publishes it for arm64 only; on the IQ-9 install `ros-jazzy-qrb-ros-slam-msgs` and delete that package.

## Differences from the Gazebo sample

- **Nav2 on wall time.** `nav_preparation_node` starts Nav2 itself with `use_sim_time:=true`; MoveIt Pro publishes no `/clock`, so `scripts/nav2_on_wall_time/ros2`, on that node's `PATH` only, runs `launch/nav2.launch.py` instead (wall time, stamped velocity commands, Nav2's default parameters).
- **Stamped velocity commands.** Qualcomm's nodes publish `Twist` on `/cmd_vel`; Jazzy's `diff_drive_controller` takes `TwistStamped` only, so their `/cmd_vel` is remapped and stamped by `twist_stamper.py`.
- **Turn calibration.** `wheel_separation_multiplier` is 1.025, not Qualcomm's 1.125: this base turns at 0.975 × the multiplier × the command in MuJoCo, and `build_map_node` closes each turn on odometry, so with 1.125 its route drives into a station wall (see the comment in `config/control/qrb_robot_base_mini.ros2_control.yaml`).
- **`control_period` 0.05.** Qualcomm's README lowers `build_map_node`'s control period from 0.2 to 0.05 s when turns overshoot and the robot hits walls; the Simulator runs slower than real time here, so the port uses 0.05 by default.
- **Simulation speed.** The 1800-beam lidar is cast on every 4 ms physics step and slows the Simulator to about 0.7× real time; the cameras render in software on the CPU image, so `render_publish_rate` is 2 Hz.
- **Detection.** SAM3 on the CPU takes about 30 s per frame, longer than `task_manager_node`'s 10 s wait after arrival, so `QRB Detect Person` runs continuously from before the robot arrives.
- **The office world.** Walls are `office.sdf`'s own boxes; furniture and the seated people are the Fuel models it includes, at its poses (`description/assets/LICENSES.md`).
