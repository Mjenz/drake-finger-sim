# finger_vision package
* RDS Speedster team
* Spring 2026

## Description
This package detects colored targets in 3D space using an Intel RealSense RGB-D camera and publishes their positions as TF transforms for the finger to track.

## Launchfiles
| Launch file | Description |
|---|---|
| `ros2 launch finger_vision vision.launch.xml` | RealSense camera + AprilTag detection + vision node + Rviz2. |

## Config files
| File | Description |
|---|---|
| `config/vision.yaml` | HSV filter bounds, depth clip range, blur/area thresholds for color-based object detection. |
| `config/apriltag.yaml` | AprilTag family and detection parameters for the `apriltag_ros` node. |

## Nodes
| Executable | Description |
|---|---|
| `vision` | Processes aligned RGB-D streams to detect a colored target. Applies HSV filtering, smoothing, morphological closing, and contour analysis to find the centroid; reprojects it to 3D using camera intrinsics; and broadcasts the result as a TF transform at 20 Hz. All parameters are hot-reloadable at 100 Hz. |
