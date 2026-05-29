# finger_recorder package
* RDS Speedster team
* Spring 2026

## Description
This package records motor feedback data from the finger to MCAP bag files for offline analysis and plotting.

## Nodes
| Executable | Description |
|---|---|
| `record` | Subscribes to actual, setpoint, and activity motor feedback topics and writes them to a timestamped MCAP bag file in `finger_recorder/bags/`. |

## Visualzation
| Executable | Description |
|---|---|
| `plotter.py` | Plots the recorded data, run this file from the top level workspace and source before running. Can plot the latest recording or specify location of bag to plot. Can enable and disable plotting the torque commands from drake. |
