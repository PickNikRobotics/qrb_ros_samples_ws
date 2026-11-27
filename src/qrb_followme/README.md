# qrb_target_tracking

MoveIt Pro Behavior plugin derived from the Qualcomm Follow-Me tracking/control design.
It consumes the Re-ID-selected 2D target, aligned depth image, and camera intrinsics;
computes median target depth and horizontal bearing; runs linear/angular PID; publishes
`geometry_msgs/msg/TwistStamped` on `/cmd_vel`; and outputs the same command to the BT blackboard.

## Ports

Required inputs:
- `depth_image`: aligned `sensor_msgs/msg/Image` (`16UC1` millimeters or `32FC1` meters)
- `camera_info`: `sensor_msgs/msg/CameraInfo`
- `target_detection`: `vision_msgs/msg/Detection2D`

Outputs:
- `cmd_vel`: `geometry_msgs/msg/TwistStamped`
- `target_depth`, `target_angle`, `message`

## Example BT XML

```xml
<Action
  ID="QrbTargetTracking"
  depth_image="{depth_image}"
  camera_info="{camera_info}"
  target_detection="{target_detection}"
  target_distance="2.0"
  distance_tolerance="0.15"
  angle_tolerance="0.05"
  linear_kp="0.45"
  linear_ki="0.0"
  linear_kd="0.05"
  angular_kp="1.2"
  angular_ki="0.0"
  angular_kd="0.08"
  max_linear_speed="0.45"
  max_angular_speed="0.8"
  cmd_vel="{cmd_vel}"
  target_depth="{target_depth}"
  target_angle="{target_angle}"
  message="{tracking_message}"
/>
```

## Build

```bash
cd ~/user_ws
rm -rf build/qrb_target_tracking install/qrb_target_tracking
colcon build --packages-select qrb_target_tracking --symlink-install --event-handlers console_direct+
source install/setup.bash
```

## Important assumptions

1. `target_detection` uses `vision_msgs/msg/Detection2D`, matching the earlier Re-ID package draft.
2. Depth is already aligned to the RGB/detection coordinate system.
3. `/cmd_vel` uses `TwistStamped`. Change the publisher/port type to `Twist` if your base expects unstamped Twist.
4. `BehaviorContext` is expected to expose `shared_resources->node`, matching the earlier plugin structure. If your compiled YOLO plugin uses a different MoveIt Pro constructor/base class, mirror that exact API.
