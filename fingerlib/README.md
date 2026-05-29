# fingerlib
* RDS Speedster team
* Spring 2026

## Description
C++23 library providing the core kinematics, trajectory generation, and motor control for the robotic finger. Used by `finger_planner`, `finger_control`, `finger_simulation`, and `finger_vision2`.

## Header files

### `Transformer` (`include/fingerlib/transformer.hpp`)
Handles all coordinate space conversions for the finger. Constructed with the motor radius matrix, tendon routing matrix, screw axes, home configuration SE(3), 4-bar linkage lengths, and joint limits.

| Method | Description |
|---|---|
| `joint_to_motor(q_joint)` | Convert joint angles (rad) → motor positions |
| `motor_to_joint(q_motor)` | Convert motor positions → joint angles (rad) |
| `joint_to_motor_torque(t_joint)` | Convert joint torques → motor torques |
| `joint_to_end_effector(q_joint)` | Forward kinematics: joint angles → fingertip SE(3) |
| `end_effector_to_joint(xyz)` | Inverse kinematics: fingertip position → joint angles |
| `get_jacobian_space(q_joint)` | Space-frame Jacobian (PIP/DIP collapsed to single DOF) |
| `get_jacobian_body(q_joint)` | Body-frame Jacobian (PIP/DIP collapsed to single DOF) |
| `calculate_4bar_ratios(pip_angle, ...)` | Compute DIP angle and speed ratio from PIP angle via 4-bar geometry |

### `JointTrajectory` (`include/fingerlib/joint_trajectory.hpp`)
Generates motor position trajectories at a configurable sampling rate. Constructed with a `Transformer`, sampling rate (Hz), and ground height.

| Method | Description |
|---|---|
| `generate_sinusoid(joint, amp, freq, v_shift)` | Sinusoidal trajectory for one joint |
| `generate_chirp(joint, amp, freq_1, freq_2, time, v_shift)` | Frequency-sweep (chirp) position trajectory |
| `generate_chirp_velocity(joint, amp, freq_1, freq_2, time, start_pos)` | Frequency-sweep velocity trajectory |
| `generate_step(joint, amp, freq, v_shift)` | Square-wave step trajectory for one joint |
| `generate_linear(start, end, v_max, a_max)` | Linear joint-space move with trapezoidal time scaling |
| `generate_cartesian(waypoints, v_max, a_max)` | Cartesian waypoint trajectory via linear IK interpolation |
| `generate_force_step(q_joint, force_low, force_high, freq)` | Alternating force step at the end-effector via Jacobian transpose |

### `PositionController` (`include/fingerlib/pos_controller.hpp`)
PID + feedforward motor position controller. Constructed with `Kp`, `Ki`, `Kd` gains.

| Method | Description |
|---|---|
| `pump_controller(setpoint, actual, next_cmd, shaft_vel)` | Compute control output (torque/PWM). `next_cmd` provides feedforward; `shaft_vel` drives the derivative term. |
| `set_ffwd_control(enable)` | Enable/disable feedforward term |
| `set_gvty_compensation(enable)` | Enable/disable gravity compensation term |
| `set_i_clamp_val(clamp_val)` | Set integral error clamp |
| `set_u_clamp_val(clamp_val)` | Set output command clamp |

## Tests
Tests use Catch2 3. Build `fingerlib` then run the test binary:

```bash
colcon build --packages-select fingerlib
./build/fingerlib/tests           # all tests
./build/fingerlib/tests "[tag]"   # filter by tag
```
