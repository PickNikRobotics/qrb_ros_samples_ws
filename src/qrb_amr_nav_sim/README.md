# qrb_amr_nav_sim

MoveIt Pro robot configuration package for Qualcomm's `simulation_amr_navigation` sample (`qrb_ros_samples/robotics/simulation_amr_navigation`, with its prerequisite `simulation_2d_lidar_slam`): the QRB robot base mini maps the Ionic world with Cartographer 2D lidar SLAM while Nav2 drives it to goals, in the MoveIt Pro Simulator instead of Gazebo.

| Sample step | Here |
|---|---|
| `gazebo_robot_base_mini.launch.py world_model:=ionic` | `description/scene.xml`, started by `moveit_pro run` |
| `ros2 launch cartographer_ros qrb_2d_lidar_slam.launch.py` | `launch/navigation_drivers.launch.py`, the driver launch in `config/config.yaml` |
| `ros2 launch nav2_bringup navigation_launch.py` | the same launch, with `config/nav2/nav2_params.yaml` |
| RViz "2D Goal Pose" | the `Navigate to Clicked Pose` Objective |
| `teleop_twist_keyboard` for mapping | the Desktop App's teleoperation (`Request Teleoperation`, Pose tab: X ± drives, Roll ± turns) |

`config/cartographer/qrb_amr_mapping_2d.lua` and `launch/qrb_2d_lidar_slam.launch.py` are byte-identical copies from Qualcomm's `ros-jazzy-qcom-cartographer-ros` package. `config/nav2/nav2_params.yaml` is `nav2_bringup`'s defaults file, unchanged.

## Cartographer: upstream now, Qualcomm's fork later

This package uses **upstream Cartographer** (`ros-jazzy-cartographer`, `ros-jazzy-cartographer-ros`) from the ROS 2 apt repository, which `rosdep` installs into the user image from `package.xml`'s `cartographer_ros` dependency. It builds for amd64 and arm64.

The sample runs on Qualcomm's fork, which these examples normally use on the IQ-9 (arm64). To switch to it:

- **Packages:** `ros-jazzy-qcom-cartographer`, `ros-jazzy-qcom-cartographer-ros`, and `ros-jazzy-qrb-ros-slam-msgs` from the PPA `ppa:ubuntu-qcom-iot/qirp`.
- **Architecture:** the PPA publishes **arm64 only**, so the fork cannot be installed in an amd64 image. Build the user image on the IQ-9 or another arm64 host.
- **Image:** in the workspace `Dockerfile`, before `rosdep install`, add the PPA (`add-apt-repository ppa:ubuntu-qcom-iot/qirp` with its signing key) and `apt-get install` the three packages. They provide the `cartographer_ros` package name, so `package.xml` stays as it is. Keep upstream `ros-jazzy-cartographer*` out of the image, because both install to the same paths.
- **Launch:** the fork's `cartographer_node` defaults its Lua file to `qrb_amr_mapping_2d.lua` in its own share directory. Either keep `navigation_drivers.launch.py`, which passes this package's identical copy explicitly, or include the fork's own `qrb_2d_lidar_slam.launch.py` (copied in `launch/`) in its place.
- **What the fork adds:** the `qrb_ros_slam_msgs/SlamCommand` service for saving and loading maps. The sample's navigation steps do not use it, and upstream Cartographer's mapping is the same code.

## Differences from the Gazebo sample

- `use_sim_time` is false: MoveIt Pro runs on wall time and the Simulator publishes no `/clock`.
- `diff_drive_controller` replaces Gazebo's diff-drive plugin, with Qualcomm's wheel values; like the plugin, it publishes no odometry TF, which Cartographer owns (`provide_odom_frame = true`).
- Nav2 publishes `TwistStamped` (`enable_stamped_cmd_vel`, set in the launch file), because Jazzy's `diff_drive_controller` accepts nothing else.
- The room's collision is the Fuel "Ionic interior" model's own primitive colliders (`description/ionic.xml`, CC-BY-4.0, Open Robotics), drawn as plain shapes; MuJoCo cannot load its textured GLB.
- The lidar is 1800 MuJoCo rangefinders at 15 Hz, Qualcomm's geometry; the physics step is the Ionic world's 4 ms.
