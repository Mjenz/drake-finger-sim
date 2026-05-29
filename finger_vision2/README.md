# finger_vision2 package
* RDS Speedster team
* Spring 2026

## Description
This package provides multiple vision backends for the whack-a-mole and 3D waypoint tracking demos: real camera-based detection, simulation-based random goal generation, and phone pixel coordinate conversion.

## Launchfiles
| Launch file | Description |
|---|---|
| `ros2 launch finger_vision2 vision2.launch.xml` | RealSense camera + AprilTag detection + `vision` node + Rviz2. |

## Config files
| File | Description |
|---|---|
| `config/phone_tf.yaml` | Phone camera extrinsic calibration (translation, rotation, pixels-per-meter) used by `pixel_vision`. |

## Nodes
| Executable | Description |
|---|---|
| `vision` | C++ color-based 3D object detection using an Intel RealSense. Applies HSV filtering, smoothing, Canny edge detection, and contour analysis; reprojects the detected centroid to 3D and broadcasts it as a TF transform at 20 Hz. |
| `simulation_vision` | Simulates a vision system for 3D waypoint tracking. Randomly samples reachable joint configurations, computes FK with `Transformer`, and publishes the resulting Cartesian goal as a marker and TF. Generates a new goal on `/completed` or after a normally-distributed number of ticks. |
| `pixel_vision` | Converts whack-a-mole server pixel targets into 3D goal TF frames. Subscribes to `/whackamole/target`, transforms pixel coordinates to metric space via the phone camera TF, and republishes the goal as a marker and TF at 10 Hz. |
