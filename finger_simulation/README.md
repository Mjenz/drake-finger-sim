# finger_simulation package
* RDS Speedster team
* Spring 2026

## Description
This package runs a drake simulation of our robotic finger, visualized in Rviz. 

The SDF version (4bar) is for dynamic simulation of the robotic finger and models the four bar linkage such that it can be closed within the drake simulation.
The URDF version (basic) is for kinematic modeling and assumes a linearly scaled relationship between the PIP and DIP joints.

## Launchfiles
| Launch file | Description |
|---|---|
| `ros2 launch finger_simulation 4barsim.launch.xml` | Drake 4-bar simulation with Rviz2 and a fingertip static TF. |
| `ros2 launch finger_simulation basic_fingersim.launch.xml` | Drake 4-bar simulation with Rviz2 (no fingertip TF). |

## Nodes
| Executable | Description |
|---|---|
| `4barsim` | Drake dynamics simulation with full tendon-driven pipeline. Loads the finger SDF, closes the 4-bar loop constraint, and wires the complete Drake system graph. Exposes `/heartbeat` to signal initialization complete. |
| `basic` | Kinematic Drake simulation (URDF-based). Used alongside the hardware bridge to visualize actual finger pose. |
| `basic_setpoint` | Kinematic Drake simulation for visualizing the commanded (setpoint) finger pose. Launched under the `setpoint` namespace with `motor_pos_actual_feedback` remapped to `motor_pos_setpoint_feedback`. |