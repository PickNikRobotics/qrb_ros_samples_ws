# QRB Follow-Me Simulation Demo

## 1. Overview

The QRB Follow-Me demo demonstrates how existing QRB ROS packages can be integrated into MoveIt Pro as reusable Behavior Tree plugins.

The demo implements an end-to-end Follow-Me pipeline consisting of:

1. RGB, depth, and camera-info acquisition
2. YOLO-based person detection
3. OSNet-based People Re-ID
4. RGB-D target tracking
5. AMR velocity control through `/cmd_vel`

The pipeline is implemented as a MoveIt Pro Behavior Tree so that individual QRB ROS capabilities can be reused and composed as modular MoveIt Pro behaviors.

### High-Level Pipeline

```text
                    MoveIt Pro Behavior Tree
                              |
                              v
                 +-------------------------+
                 |  Acquire Camera Inputs  |
                 | RGB + Depth + CameraInfo|
                 +------------+------------+
                              |
                              v
                 +-------------------------+
                 |   QrbYoloBehavior       |
                 |   Person Detection      |
                 +------------+------------+
                              |
                              v
                 +-------------------------+
                 |    QrbPeopleReid        |
                 |    Person Matching      |
                 +------------+------------+
                              |
                              v
                 +-------------------------+
                 |  QrbTargetTracking      |
                 | RGB-D Target Tracking   |
                 +------------+------------+
                              |
                              v
                         /cmd_vel
                              |
                              v
                             AMR
```

## 2. Running the Demo

The demo requires the MoveIt Pro runtime, YOLO object detection, and People Re-ID to run together.

### 2.1 Start MoveIt Pro on the Device

On the device:

```bash
moveit_pro run -c qrb_followme
```

Wait until the MoveIt Pro runtime has started successfully.

### 2.2 Connect from MoveIt Pro Desktop

On the host machine:

1. Open **MoveIt Pro Desktop**.
2. Connect to the device using the device IP address and Frontkey.
3. Open the `qrb_followme` configuration.
4. Load and run the Follow-Me objective:

```text
follow_me.xml
```

The objective is located at:

```text
qrb_followme/objectives/follow_me.xml
```

### 2.3 Start YOLO Object Detection

Open a terminal on the device:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/ubuntu/.ros/cyclonedds.xml

ros2 launch sample_object_detection launch_with_qrb_ros_camera.py \
  model:=/opt/model/yolov8_detection_qcs9075.bin \
  label_file:=/opt/model/coco8.yaml
```

The `QrbYoloBehavior` plugin sends the RGB image to `/cam0_stream1` and waits for detection results from `/yolo_detect_result`.

### 2.4 Start People Re-ID

Open another terminal on the device with a clean shell environment:

```bash
env -i HOME=$HOME USER=$USER TERM=$TERM PATH=/usr/bin:/bin bash --noprofile --norc
```

Then run:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/ubuntu/.ros/cyclonedds.xml

ros2 launch qrb_ros_people_reid qrb_sample_people_reid.launch.py \
  reid_model_path:=./osnet.bin
```

The People Re-ID module uses the OSNet model to identify and continuously match the target person detected by YOLO.

### 2.5 Start the Simulated Person

Open a third terminal on the device and enter the MoveIt Pro runtime container:

```bash
docker exec -it moveit_pro-runtime-1 /bin/bash
```

Set the simulated person controller to automatic mode:

```bash
ros2 topic pub --once \
  /followme_person/control_mode \
  std_msgs/msg/String \
  "{data: 'auto'}"
```

The simulated person will move automatically according to the predefined trajectory.

### 2.6 Run the Follow-Me Objective

Return to **MoveIt Pro Desktop** and run `follow_me.xml`.

The complete pipeline should execute continuously:

```text
Camera
  |
  +-- RGB
  +-- Depth
  +-- CameraInfo
        |
        v
 YOLO Detection
        |
        v
 People Re-ID
        |
        v
 Target Tracking
        |
        v
    /cmd_vel
        |
        v
       AMR
```

## 3. Expected Result

After the complete pipeline is started:

1. The simulated person moves along the predefined trajectory.
2. The camera continuously provides RGB and depth images.
3. YOLO detects people in the RGB image.
4. People Re-ID identifies the target person.
5. Target tracking estimates the target position using the YOLO bounding box, depth image, and camera intrinsics.
6. The controller calculates linear and angular velocity.
7. Velocity commands are published to `/cmd_vel`.
8. The AMR follows the selected person while maintaining the configured target distance.

The current target distance is:

```text
target_distance = 2.0 m
```

## 4. Performance

| Component | Performance |
|---|---:|
| Camera input rate | Approximately 2 Hz |
| NN inference latency | Approximately 15 ms |
| Target follow distance | 2.0 m |
| Maximum linear speed | 3.0 |
| Maximum angular speed | 0.8 rad/s |

At a camera rate of approximately 2 Hz, a new frame is available about every 500 ms. The observed NN inference latency is approximately 15 ms. Therefore, the current end-to-end pipeline is primarily constrained by the camera/input rate rather than NN inference latency.

> The performance values above represent the currently observed demo performance and should not be interpreted as guaranteed platform performance.

## 5. Module Design

The implementation separates Follow-Me functionality into reusable MoveIt Pro Behavior plugins.

### 5.1 Camera Input

The Behavior Tree retrieves three inputs:

```text
/followme_camera/color
/followme_camera/depth
/followme_camera/camera_info
```

They are acquired through two `GetImage` nodes and one `GetCameraInfo` node. A `Parallel` control node waits until all three inputs succeed before continuing.

```text
              +-- RGB ----------> image
Camera -------+-- Depth --------> depth_image
              +-- CameraInfo ---> camera_info
                         |
                  all inputs ready
                         |
                         v
                     Detection
```

### 5.2 QrbYoloBehavior

`QrbYoloBehavior` bridges the MoveIt Pro Behavior Tree and the QRB YOLO inference pipeline.

**Input:**

```text
image
```

**ROS interfaces:**

```text
Image input:      /cam0_stream1
Detection result: /yolo_detect_result
```

**Output:**

```text
yolo_detect_result
```

```text
MoveIt Pro RGB Image
        |
        v
QrbYoloBehavior
        |
        v
 /cam0_stream1
        |
        v
QRB YOLO Pipeline
        |
        v
/yolo_detect_result
        |
        v
Behavior Tree Blackboard
```

### 5.3 QrbPeopleReid

`QrbPeopleReid` determines which detected person is the current Follow-Me target. It uses the RGB image and YOLO detections, then compares each detected person's feature against the stored target feature.

**Inputs:**

```text
image
yolo_detect_result
target_feature
```

**Outputs:**

```text
similarity_score
matched
target_detection
current_feature
reid_message
```

For the OSNet model used by this demo, the similarity value represents cosine distance:

```text
smaller distance = more similar
```

### 5.4 QrbTargetTracking

`QrbTargetTracking` converts the 2D target detection into robot motion by combining:

```text
target_detection + depth_image + camera_info
```

**Outputs:**

```text
target_depth
target_angle
tracking_message
```

The calculated velocity command is published to `/cmd_vel`.

#### Depth Estimation

Configured valid depth range:

```text
min_valid_depth = 0.3 m
max_valid_depth = 10.0 m
depth_roi_radius = 5
```

The local ROI helps reduce sensitivity to individual invalid or noisy depth pixels.

#### Motion Control

Desired following distance:

```text
target_distance = 2.0 m
```

Conceptually:

```text
distance_error = target_depth - target_distance
```

Linear PID parameters:

```text
Kp = 0.45
Ki = 0.0
Kd = 0.05
```

Angular PID parameters:

```text
Kp = 1.2
Ki = 0.0
Kd = 0.08
```

Velocity limits:

```text
max_linear_speed  = 3
max_angular_speed = 0.8
```

## 6. Behavior Tree Structure

```text
FollowMe
|
+-- KeepRunningUntilFailure
    |
    +-- RetryUntilSuccessful
        |
        +-- Sequence: qrb_follow_me_pipeline
            |
            +-- Parallel: get_camera_inputs
            |   |
            |   +-- GetImage: /followme_camera/color
            |   +-- GetImage: /followme_camera/depth
            |   +-- GetCameraInfo: /followme_camera/camera_info
            |
            +-- QrbYoloBehavior
            |   +-- Input topic:  /cam0_stream1
            |   +-- Result topic: /yolo_detect_result
            |
            +-- QrbPeopleReid
            |   +-- Output: target_detection
            |
            +-- QrbTargetTracking
                +-- Output topic: /cmd_vel
```

### 6.1 Execution Logic

```text
START
  |
  v
Acquire Camera Inputs
  |
  +-- RGB
  +-- Depth
  +-- CameraInfo
  |
  v
YOLO Detection
  |
  v
People Re-ID
  |
  v
Target Tracking
  |
  v
PID Control
  |
  v
/cmd_vel
  |
  v
Next Cycle
```

`KeepRunningUntilFailure` keeps the complete Follow-Me objective active. `RetryUntilSuccessful` retries the pipeline indefinitely when a step fails.

## 7. Behavior Tree Blackboard

| Blackboard Entry | Producer | Consumer |
|---|---|---|
| `image` | `GetImage` | `QrbYoloBehavior`, `QrbPeopleReid` |
| `depth_image` | `GetImage` | `QrbTargetTracking` |
| `camera_info` | `GetCameraInfo` | `QrbTargetTracking` |
| `yolo_detect_result` | `QrbYoloBehavior` | `QrbPeopleReid` |
| `target_feature` | Re-ID state/reference | `QrbPeopleReid` |
| `similarity_score` | `QrbPeopleReid` | Debug/status |
| `matched` | `QrbPeopleReid` | Debug/status |
| `target_detection` | `QrbPeopleReid` | `QrbTargetTracking` |
| `current_feature` | `QrbPeopleReid` | Re-ID state |
| `reid_message` | `QrbPeopleReid` | Debug/status |
| `target_depth` | `QrbTargetTracking` | Debug/status |
| `target_angle` | `QrbTargetTracking` | Debug/status |
| `tracking_message` | `QrbTargetTracking` | Debug/status |

## 8. Module Architecture

```text
+----------------------------------------------------------+
|                     MoveIt Pro                           |
|                                                          |
|  follow_me.xml                                           |
|       |                                                  |
|       +-- GetImage / GetCameraInfo                       |
|       +-- QrbYoloBehavior                                |
|       +-- QrbPeopleReid                                  |
|       +-- QrbTargetTracking ----------------> /cmd_vel   |
|                                                          |
+----------------------+-----------------------------------+
                       | ROS 2
              +--------+--------+
              |                 |
              v                 v
     QRB YOLO Detection   QRB People Re-ID
              |                 |
              v                 v
         YOLOv8 Model        OSNet Model
```

The key design principle is that QRB ROS functionality is exposed to MoveIt Pro through reusable Behavior plugins rather than implementing the entire Follow-Me application in one monolithic node.

This makes the following capabilities independently reusable in other MoveIt Pro objectives:

- YOLO detection
- People Re-ID
- RGB-D target tracking

## 9. Key ROS 2 Topics

| Topic | Purpose |
|---|---|
| `/followme_camera/color` | RGB image |
| `/followme_camera/depth` | Depth image |
| `/followme_camera/camera_info` | Camera calibration and intrinsics |
| `/cam0_stream1` | Input image for the QRB YOLO pipeline |
| `/yolo_detect_result` | YOLO detection results |
| `/followme_person/control_mode` | Simulated person control mode |
| `/cmd_vel` | AMR velocity command |

## 10. Follow-Me Behavior Tree XML

File:

```text
qrb_followme/objectives/follow_me.xml
```

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<root BTCPP_format="4" main_tree_to_execute="FollowMe">

  <BehaviorTree ID="FollowMe" _description="" _favorite="false">

    <!-- Keep the whole Follow-Me pipeline running continuously -->
    <Decorator ID="KeepRunningUntilFailure">

      <!-- Retry indefinitely if any step in the pipeline fails -->
      <Decorator ID="RetryUntilSuccessful" num_attempts="-1">

        <Control ID="Sequence" name="qrb_follow_me_pipeline">

          <!-- Acquire RGB, depth, and camera-info inputs in parallel. -->
          <Control
            ID="Parallel"
            name="get_camera_inputs"
            success_count="3"
            failure_count="1">

            <Action
              ID="GetImage"
              topic_name="/followme_camera/color"
              message_out="{image}"
              message_timeout_sec="5.0"
              publisher_timeout_sec="5.0"
            />

            <Action
              ID="GetImage"
              topic_name="/followme_camera/depth"
              message_out="{depth_image}"
              message_timeout_sec="5.0"
              publisher_timeout_sec="5.0"
            />

            <Action
              ID="GetCameraInfo"
              topic_name="/followme_camera/camera_info"
              message_out="{camera_info}"
              message_timeout_sec="5.0"
              publisher_timeout_sec="5.0"
            />

          </Control>

          <Action
            ID="QrbYoloBehavior"
            image="{image}"
            image_topic="/cam0_stream1"
            result_topic="/yolo_detect_result"
            timeout_sec="5.0"
            yolo_detect_result="{yolo_detect_result}"
          />

          <Action
            ID="QrbPeopleReid"
            image="{image}"
            detections="{yolo_detect_result}"
            reference_feature="{target_feature}"
            match_threshold="1"
            template_update_threshold="0.20"
            timeout_sec="5.0"
            similarity_score="{similarity_score}"
            matched="{matched}"
            target_detection="{target_detection}"
            current_feature="{current_feature}"
            message="{reid_message}"
          />

          <Action
            ID="QrbTargetTracking"
            depth_image="{depth_image}"
            camera_info="{camera_info}"
            target_detection="{target_detection}"
            detection_image_width="640"
            detection_image_height="640"
            detection_letterboxed="true"
            cmd_vel_topic="/cmd_vel"
            target_distance="2.0"
            min_valid_depth="0.3"
            max_valid_depth="10.0"
            depth_roi_radius="5"
            linear_kp="0.45"
            linear_ki="0.0"
            linear_kd="0.05"
            angular_kp="1.2"
            angular_ki="0.0"
            angular_kd="0.08"
            max_linear_speed="3"
            max_angular_speed="0.8"
            target_depth="{target_depth}"
            target_angle="{target_angle}"
            message="{tracking_message}"
          />

        </Control>
      </Decorator>
    </Decorator>
  </BehaviorTree>

  <TreeNodesModel>
    <SubTree ID="FollowMe">
      <MetadataFields>
        <Metadata runnable="true" />
        <Metadata subcategory="" />
      </MetadataFields>
    </SubTree>
  </TreeNodesModel>

</root>
```

## 11. Summary

The QRB Follow-Me demo exposes existing QRB ROS perception and tracking capabilities as modular MoveIt Pro Behavior plugins.

The end-to-end flow is:

```text
Simulated Camera
       |
       v
RGB + Depth + CameraInfo
       |
       v
QrbYoloBehavior
       |
       v
QRB YOLO Detection
       |
       v
QrbPeopleReid
       |
       v
OSNet People Re-ID
       |
       v
QrbTargetTracking
       |
       v
Depth and Angle Estimation
       |
       v
PID Controller
       |
       v
    /cmd_vel
       |
       v
      AMR
```

The current simulation operates with a camera input rate of approximately **2 Hz** and an observed NN inference latency of approximately **15 ms**. The modular design allows the same QRB ROS capabilities to be composed into other MoveIt Pro workflows.
