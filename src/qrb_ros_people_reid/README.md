# People Re-ID ROS 2 Package

A modular ROS 2 package for person re-identification (ReID) that provides feature extraction and similarity computation services.

## Overview

This package has been refactored to focus solely on ReID operations, removing YOLO detection and other unrelated components. It provides a clean, service-based interface for:

1. **Feature Extraction**: Extract feature vectors from person images
2. **Similarity Computation**: Compare two feature vectors using cosine distance

## Architecture

The package consists of two main components:

1. **PeopleReIDNode**: Service provider that handles image preprocessing and feature management
2. **QNN Inference Node**: Neural network inference engine (OSNet model)

### Data Flow

```
Client Request (Image) 
    ↓
[extract_feature service]
    ↓
PeopleReIDNode (preprocessing)
    ↓
/reid_inference_input_tensor (TensorList)
    ↓
QNN Inference Node (OSNet model)
    ↓
/reid_tensor_output (TensorList)
    ↓
PeopleReIDNode (postprocessing)
    ↓
Service Response (Feature Vector)
```

## Services

### 1. `/extract_feature`

Extracts a feature vector from a person image.

**Service Type**: `qrb_ros_people_reid/srv/ExtractFeature`

**Request**:
- `sensor_msgs/Image image`: Input image (BGR8 format)

**Response**:
- `float32[] feature`: Extracted feature vector
- `bool success`: Operation success status
- `string message`: Status message

**Example Usage**:
```bash
ros2 service call /extract_feature qrb_ros_people_reid/srv/ExtractFeature "{image: {header: {frame_id: 'camera'}, height: 256, width: 128, encoding: 'bgr8', data: [...]}}"
```

### 2. `/compute_similarity`

Computes cosine distance between two feature vectors.

**Service Type**: `qrb_ros_people_reid/srv/ComputeSimilarity`

**Request**:
- `float32[] feature1`: First feature vector
- `float32[] feature2`: Second feature vector

**Response**:
- `float32 similarity`: Cosine distance (0.0 = identical, 2.0 = opposite)
- `bool success`: Operation success status
- `string message`: Status message

**Example Usage**:
```bash
ros2 service call /compute_similarity qrb_ros_people_reid/srv/ComputeSimilarity "{feature1: [0.1, 0.2, ...], feature2: [0.15, 0.25, ...]}"
```

## Topics

### Published
- `/reid_inference_input_tensor` (qrb_ros_tensor_list_msgs/TensorList): Preprocessed image tensor for inference

### Subscribed
- `/reid_tensor_output` (qrb_ros_tensor_list_msgs/TensorList): Feature vectors from inference engine

## Launch

### Basic Launch

```bash
ros2 launch qrb_ros_people_reid qrb_sample_people_reid.launch.py
```

### Launch with Custom Model Path

```bash
ros2 launch qrb_ros_people_reid qrb_sample_people_reid.launch.py \
    reid_model_path:=/path/to/your/osnet.bin
```

### Launch Parameters

- `reid_model_path`: Path to the OSNet model file (default: `../osnet.bin`)
- `backend_option`: QNN backend library path (default: `/usr/lib/libQnnHtp.so`)

## Model Requirements

The package expects an OSNet model in QNN format:
- **Input**: 256x128x3 RGB image (NCHW format)
- **Output**: Feature vector (typically 512 dimensions)
- **Model file**: `osnet.bin` (place in package root or specify path)

## Dependencies

- ROS 2 (Humble or later)
- rclcpp
- sensor_msgs
- qrb_ros_tensor_list_msgs
- qrb_ros_nn_inference
- cv_bridge
- OpenCV

## Building

```bash
cd /path/to/your/ros2_workspace
colcon build --packages-select qrb_ros_people_reid
source install/setup.bash
```

## Usage Example (Python)

```python
import rclpy
from rclpy.node import Node
from qrb_ros_people_reid.srv import ExtractFeature, ComputeSimilarity
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ReIDClient(Node):
    def __init__(self):
        super().__init__('reid_client')
        self.extract_client = self.create_client(ExtractFeature, 'extract_feature')
        self.similarity_client = self.create_client(ComputeSimilarity, 'compute_similarity')
        self.bridge = CvBridge()
    
    def extract_feature(self, image_path):
        # Load image
        cv_image = cv2.imread(image_path)
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        
        # Call service
        request = ExtractFeature.Request()
        request.image = ros_image
        future = self.extract_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result().feature
    
    def compute_similarity(self, feature1, feature2):
        request = ComputeSimilarity.Request()
        request.feature1 = feature1
        request.feature2 = feature2
        future = self.similarity_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result().similarity

# Usage
rclpy.init()
client = ReIDClient()
feature1 = client.extract_feature('person1.jpg')
feature2 = client.extract_feature('person2.jpg')
similarity = client.compute_similarity(feature1, feature2)
print(f"Similarity: {similarity}")
```

## Notes

- The service calls are synchronous and will block until inference completes
- Default timeout for inference is 5 seconds
- Images are automatically resized to 256x128 and normalized
- Cosine distance ranges from 0.0 (identical) to 2.0 (completely different)
- Lower similarity scores indicate more similar features

## License

BSD-3-Clause-Clear

## Maintainer

Qualcomm Innovation Center
