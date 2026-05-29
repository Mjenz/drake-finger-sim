# finger_interfaces package
* RDS Speedster team
* Spring 2026

## Description
This package defines all custom ROS2 messages, services, and actions used across the robotic finger project.

## Messages

| Message | Description |
|---|---|
| `MotorFeedback` | Motor shaft positions for splay, MCP flex, and PIP/DIP flex (rad). |
| `MotorActivity` | Activity state of the motor controller (1.0 = executing, 0.0 = idle). |

## Services

| Service | Description |
|---|---|
| `SendCommand` | Send a full motor position trajectory to the bridge. `mode` selects position/velocity/torque control. |
| `StartStopCommand` | Start or stop execution of the currently loaded trajectory. |

## Actions

| Action | Description |
|---|---|
| `Cartesian` | Move the fingertip through a list of Cartesian waypoints (m). |
| `Linear` | Linear move between two joint-space positions `[splay, mcp, pip/dip]` (rad). |
| `Sinusoidal` | Sinusoidal trajectory on one joint (rad, Hz). Set `repeat=1` to loop. |
| `Chirp` | Frequency-sweep position chirp on one joint (rad, Hz, s). |
| `ChirpVelocity` | Frequency-sweep velocity chirp on one joint starting from a specified position (rad, Hz, s). |
| `Force` | Alternating force step at the end-effector via Jacobian transpose. `q_joint` is the hold pose (rad); forces are `[Fx, Fy, Fz]` (N). |
