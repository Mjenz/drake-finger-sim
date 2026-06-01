# finger_control package
* RDS Speedster team
* Spring 2026

## Description
This packages serves as the high level controller controlling where the finger goes.

## Launchfiles
| Launch file | Description |
|---|---|
| `ros2 launch finger_control demo.launch.xml` | Full control stack running a selectable demo trajectory. Args: `bridge:=simulation\|hardware`, `demo:=force_step\|sinusoidal\|linear\|linear_step\|ik\|cartesian_ik\|chirp\|chirp_velocity\|lissajous\|none`, `use_rviz:=true\|false`, `record:=true\|false` |
| `ros2 launch finger_control 3d_waypoint_tracking.launch.xml` | Full control stack with simulation vision for 3D waypoint tracking. Args: `bridge:=simulation\|hardware`, `use_rviz:=true\|false`, `record:=true\|false` |
| `ros2 launch finger_control pixel_whackamole.launch.xml` | Whack-a-mole game using pixel-based vision and the whackamole server. Args: `bridge:=simulation\|hardware`, `use_rviz:=true\|false`, `record:=true\|false` |

## Config files
| File | Description |
|---|---|
| `config/demo.yaml` | Demo trajectory parameters (joint targets, amplitudes, frequencies, etc.) |

## Nodes
| Executable | Description |
|---|---|
| `demo` | Runs a single configurable demo trajectory selected by the `demo` parameter (linear, linear_step, sinusoidal, force_step, ik, cartesian_ik, chirp, chirp_velocity, lissajous). |
| `control` | High-level whack-a-mole control node. Polls the TF tree for a `goal` frame, sends cartesian IK moves when a new goal appears, and publishes `/completed` once the fingertip arrives. |

## Helpful commands
| Command | Description |
|---|---|
| `ros2 service call /<action_name>/_action/cancel_goal action_msgs/srv/CancelGoal` | Cancel an action call that is being repeated (sinusoidal or force_step). |


