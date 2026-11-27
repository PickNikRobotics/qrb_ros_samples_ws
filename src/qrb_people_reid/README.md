# qrb_people_reid

MoveIt Pro Behavior plugin that crops each `vision_msgs/Detection2DArray` candidate from an RGB image, calls `/extract_feature`, and compares it with a persistent `reference_feature` through `/compute_similarity`.

## Behavior ports

- Inputs: `image`, `detections`, `match_threshold`, `template_update_threshold`, `timeout_sec`
- Bidirectional: `reference_feature`
- Outputs: `similarity_score`, `matched`, `target_detection`, `current_feature`, `message`

An empty `reference_feature` initializes the target from the first valid detection. Later ticks compare all candidates and select the lowest distance. A high-confidence match updates the template.

## Build

```bash
cd ~/moveit_pro/qrb_ros_samples_ws
rm -rf build/qrb_people_reid install/qrb_people_reid
colcon build --packages-select qrb_people_reid --symlink-install --event-handlers console_direct+
source install/setup.bash
```

## Example XML

```xml
<QrbPeopleReId
  image="{image}"
  detections="{detections}"
  reference_feature="{target_feature}"
  match_threshold="0.35"
  template_update_threshold="0.20"
  timeout_sec="3.0"
  similarity_score="{similarity_score}"
  matched="{matched}"
  target_detection="{target_detection}"
  current_feature="{current_feature}"
  message="{reid_message}" />
```

The registered Behavior ID is `QrbPeopleReId` only if changed in `register_behaviors.cpp`; this package currently registers `QrbPeopleReid`. Use the exact current ID:

```xml
<QrbPeopleReid ... />
```

## Assumptions

- `qrb_ros_people_reid/srv/ExtractFeature` request contains `sensor_msgs/Image image`; response contains `float32[] feature`, `bool success`, `string message`.
- `ComputeSimilarity` uses `float32[] feature1`, `float32[] feature2`; response contains `float32 similarity`, `bool success`, `string message`.
- Lower similarity values mean a better match, matching the Qualcomm Follow-Me sample's distance-threshold behavior.
