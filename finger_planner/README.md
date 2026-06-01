# finger_planner package
* RDS Speedster team
* Spring 2026

## Description
This package contains asynchronous services to plan movements and create trajectories for the finger motors to follow.

## Config files
| File | Description |
|---|---|
| `config/movement_config.yaml` | Motion planning parameters: `max_velocity`, `max_acceleration`, `relative_gnd_height`. |

## Nodes
| Executable | Description |
|---|---|
| `finger_planner` | Action server for cartesian, linear, linear step, sinusoidal, force step, chirp, and chirp velocity movements; generates motor position commands; manages execution via `/send_command`, `/start_command`, `/stop_command`. |

