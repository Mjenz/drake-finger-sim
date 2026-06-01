/// \file
/// \brief Runs a configurable single-shot demo trajectory. Waits for the Drake
///        heartbeat, then executes one trajectory selected by the `demo` parameter:
///        linear, sinusoidal, force_step, ik, cartesian_ik, chirp, or chirp_velocity.
///
/// PARAMETERS:
///   + demo (string) - Trajectory type to run: linear | sinusoidal | force_step | ik | cartesian_ik | chirp | chirp_velocity
///   + linear.start_loc / linear.end_loc (double[3]) - Joint-space start and end positions (rad)
///   + sinusoidal.joint / .amp / .freq / .v_offset / .repeat - Sinusoidal oscillation parameters
///   + chirp.joint / .amp / .freq_init / .freq_final / .time / .v_offset - Chirp sweep parameters
///   + chirp_velocity.joint / .vel_amp / .freq_init / .freq_final / .time / .start_pos - Velocity chirp parameters
///   + force.q_state / .low / .high / .freq / .repeat - Force step parameters
///   + ik.start / ik.end (double[3]) - Cartesian start/end positions for IK demos (m)
///   + ik.repeat (int) - Number of back-and-forth repetitions for ik demos
///
/// SUBSCRIBES:
///   + /motor_pos_actual_feedback (finger_interfaces/msg/MotorFeedback) - Used to derive current joint state for relative moves
///
/// CLIENTS:
///   + /heartbeat (std_srvs/srv/Empty) - Blocks startup until the Drake simulation is ready
///   + /cartesian_move (finger_interfaces/action/Cartesian) - Sends end-effector waypoint goals
///   + /sinusoidal_move (finger_interfaces/action/Sinusoidal) - Sends sinusoidal joint trajectory goals
///   + /linear_move (finger_interfaces/action/Linear) - Sends linear joint-space trajectory goals
///   + /force_step_move (finger_interfaces/action/Force) - Sends alternating force step goals
///   + /chirp_move (finger_interfaces/action/Chirp) - Sends frequency-sweep position chirp goals
///   + /chirp_velocity_move (finger_interfaces/action/ChirpVelocity) - Sends frequency-sweep velocity chirp goals

#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <armadillo>

#include "finger_control/finger_control_base.hpp"


using namespace std::chrono_literals;

/// \brief A class that bridges commands and feedback between ros and drake
class FingerDemo : public FingerControlBase
{
public:

  /// \brief Create an instance of FingerDemo for running demos
  FingerDemo()
  : FingerControlBase("finger_demo")
  {
    declare_and_get_demo_params();

    if (demo_ == "linear") {
      RCLCPP_INFO(get_logger(), "Running linear joint movement demo...");

      std::vector<float> start_joint_loc(linear_start_loc_.begin(), linear_start_loc_.end());
      std::vector<float> end_joint_loc(linear_end_loc_.begin(), linear_end_loc_.end());

      // send_linear_goal(0, {start_joint_loc});
      send_linear_goal(1, {start_joint_loc,
                        end_joint_loc,
                        start_joint_loc});
    }

    else if (demo_ == "linear_step") {
      RCLCPP_INFO(get_logger(), "Running linear step joint movement demo...");

      send_linear_step_goal(linear_step_repeat_, linear_step_freq_, linear_step_waypoints_);
    }

    else if (demo_ == "sinusoidal") {
      RCLCPP_INFO(get_logger(), "Running sinusoidal movement demo...");

      // send_sinusoid_goal(repeat, joint, amp, freq, v_offset);
      send_sinusoid_goal(sinusoidal_repeat_, sinusoidal_joint_, sinusoidal_amp_, sinusoidal_freq_, sinusoidal_v_offset_);

    }

    else if (demo_ == "force_step") {
      RCLCPP_INFO(get_logger(), "Running force step demo...");
      std::vector<float> q_joints(force_q_state_.begin(), force_q_state_.end());
      std::vector<float> force_low(force_low_.begin(), force_low_.end());
      std::vector<float> force_high(force_high_.begin(), force_high_.end());

      send_force_step_goal(q_joints, force_low, force_high, force_freq_, force_repeat_);
    }

    else if (demo_ == "ik") {
      RCLCPP_INFO(get_logger(), "Running inverse kinematics demo...");
      std::vector<float> start(ik_start_.begin(), ik_start_.end());
      std::vector<float> end(ik_end_.begin(), ik_end_.end());

      // move to start
      send_cartesian_goal({start});

      for (auto i = 0; i < ik_repeat_; i++) {
          send_cartesian_goal({start, end});
          send_cartesian_goal({end, start});
      }
    }

    else if (demo_ == "cartesian_ik") {
      RCLCPP_INFO(get_logger(), "Running cartesian ik demo...");

      auto lerp_waypoints = [](const std::vector<float>& start,
        const std::vector<float>& end, int n = 30) {
            std::vector<std::vector<float>> points;
            for (int i = 0; i <= n; ++i) {
                float t = static_cast<float>(i) / n;
                points.push_back({
                    start[0] + t * (end[0] - start[0]),
                    start[1] + t * (end[1] - start[1]),
                    start[2] + t * (end[2] - start[2])
                });
            }
            return points;
        };

      std::vector<float> start(ik_start_.begin(), ik_start_.end());
      std::vector<float> end(ik_end_.begin(), ik_end_.end());

      for (auto i = 0; i < 20; i++) {
          send_cartesian_goal(lerp_waypoints(start, end));
          send_cartesian_goal(lerp_waypoints(end, start));
      }
    }

    else if (demo_ == "chirp") {
      RCLCPP_INFO(get_logger(), "Running chirp demo...");

      send_chirp_goal(chirp_joint_, chirp_amp_, chirp_freq_init_, chirp_freq_final_, chirp_time_, chirp_v_offset_);
    }

    else if (demo_ == "chirp_velocity") {
      RCLCPP_INFO(get_logger(), "Running chirp velocity demo...");

      std::vector<float> start_pos(chirp_velocity_start_pos_.begin(), chirp_velocity_start_pos_.end());

      send_chirp_velocity_goal(chirp_velocity_joint_, chirp_velocity_vel_amp_, chirp_velocity_freq_init_, chirp_velocity_freq_final_, chirp_velocity_time_, start_pos);
    }

    else if (demo_ == "lissajous") {
      RCLCPP_INFO(get_logger(), "Running lissajous demo...");
      auto lissajous = 
        [this](float y_offset = 0.07, float z_offset = -0.07, float f = 0.5, int hz = 800) {

            auto n = int(hz / f);
            RCLCPP_INFO_STREAM(get_logger(), n);
            std::vector<std::vector<float>> points;
            for (int i = 0; i <= n; ++i) {
                float t = static_cast<float>(i) /  n;
                points.push_back({
                    0.0f, // motion is in the x plane
                    1.5f * std::sinf(2 * M_PI * t) / 100.0f + y_offset,
                    1.5f * std::sinf(2 * M_PI* 2 * t + 3 * M_PI / 4) / 100.0f + z_offset - 0.0161f
                });
            }
            return points;
        };
      for (auto i = 0; i < 20; i++) {
        send_cartesian_goal(lissajous());
        rclcpp::sleep_for(500ms);
      }
      }
  }

private:
  std::string demo_;
  int sinusoidal_repeat_;
  int sinusoidal_joint_;
  double sinusoidal_amp_;
  double sinusoidal_freq_;
  double sinusoidal_v_offset_;
  int chirp_joint_;
  double chirp_amp_;
  double chirp_freq_init_;
  double chirp_freq_final_;
  double chirp_time_;
  double chirp_v_offset_;
  int chirp_velocity_joint_;
  double chirp_velocity_vel_amp_;
  double chirp_velocity_freq_init_;
  double chirp_velocity_freq_final_;
  double chirp_velocity_time_;
  std::vector<double> chirp_velocity_start_pos_;
  std::vector<double> linear_start_loc_;
  std::vector<double> linear_end_loc_;
  std::vector<std::vector<float>> linear_step_waypoints_;
  int linear_step_repeat_;
  double linear_step_freq_;
  double force_freq_;
  int force_repeat_;
  std::vector<double> force_q_state_;
  std::vector<double> force_low_;
  std::vector<double> force_high_;
  int ik_repeat_;
  std::vector<double> ik_start_;
  std::vector<double> ik_end_;

  void declare_and_get_demo_params()
  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};

    param_desc.description = "The demo that should be run.";
    declare_parameter("demo", "none", param_desc);

    param_desc.description = "Boolean (1 or 0) to enable repeat for sinusoidal";
    declare_parameter("sinusoidal.repeat", 0, param_desc);

    param_desc.description = "The joint to oscillate for sinusoidal demo.";
    declare_parameter("sinusoidal.joint", 0, param_desc);

    param_desc.description = "The amplitude to oscillate at for sinusoidal demo (rad).";
    declare_parameter("sinusoidal.amp", 0.0, param_desc);

    param_desc.description = "The frequency to oscillate at for sinusoidal demo (Hz).";
    declare_parameter("sinusoidal.freq", 0.0, param_desc);

    param_desc.description = "The vertical offset (in radians) for sinusoidal demo.";
    declare_parameter("sinusoidal.v_offset", 0.0, param_desc);

    param_desc.description = "The joint to oscillate for chirp demo.";
    declare_parameter("chirp.joint", 0, param_desc);

    param_desc.description = "The amplitude to oscillate at for chirp demo (rad).";
    declare_parameter("chirp.amp", 0.0, param_desc);

    param_desc.description = "The initial frequency for chirp demo (Hz).";
    declare_parameter("chirp.freq_init", 0.0, param_desc);

    param_desc.description = "The final frequency for chirp demo (Hz).";
    declare_parameter("chirp.freq_final", 0.0, param_desc);

    param_desc.description = "The length of time for the chirp demo in seconds.";
    declare_parameter("chirp.time", 0.0, param_desc);

    param_desc.description = "The vertical offset for chirp demo (in radians).";
    declare_parameter("chirp.v_offset", 0.0, param_desc);

    param_desc.description = "The joint to oscillate for velocity chirp demo.";
    declare_parameter("chirp_velocity.joint", 0, param_desc);

    param_desc.description = "The amplitude to oscillate at for velocity chirp demo (rad).";
    declare_parameter("chirp_velocity.vel_amp", 0.0, param_desc);

    param_desc.description = "The initial frequency for velocity chirp demo (Hz).";
    declare_parameter("chirp_velocity.freq_init", 0.0, param_desc);

    param_desc.description = "The final frequency for velocity chirp demo (Hz).";
    declare_parameter("chirp_velocity.freq_final", 0.0, param_desc);

    param_desc.description = "The length of time for the chirp velocity demo in seconds.";
    declare_parameter("chirp_velocity.time", 0.0, param_desc);

    param_desc.description = "The starting position for chirp velocity demo (in radians).";
    declare_parameter("chirp_velocity.start_pos", std::vector<double>{0.0, 0.0, 0.0}, param_desc);

    param_desc.description = "The start position in joint space for linear move demo <splay, mcp, pipdip> in radians.";
    declare_parameter("linear.start_loc", std::vector<double>{0.0, 0.0, 0.0}, param_desc);

    param_desc.description = "The end position in joint space for linear move demo <splay, mcp, pipdip> in radians.";
    declare_parameter("linear.end_loc", std::vector<double>{0.0, 0.0, 0.0}, param_desc);

    param_desc.description = "The list of 3 vector positions in joint space for linear step move demo in radians.";
    declare_parameter("linear_step.waypoints", std::vector<double>{0.0, 0.0, 0.0}, param_desc);

    param_desc.description = "The frequency to alterate linear step at for the linear step demo.";
    declare_parameter("linear_step.freq", 0.0, param_desc);

    param_desc.description = "Boolean (1 or 0) to indicate if linear step demo should be repeated.";
    declare_parameter("linear_step.repeat", 0, param_desc);

    param_desc.description = "The frequency to alterate force at for the force demo.";
    declare_parameter("force.freq", 0.0, param_desc);

    param_desc.description = "Boolean (1 or 0) to indicate if force demo should be repeated.";
    declare_parameter("force.repeat", 0, param_desc);

    param_desc.description = "The joint space state for force demo <splay, mcp, pipdip> in radians.";
    declare_parameter("force.q_state", std::vector<double>{0.0, 0.0, 0.0}, param_desc);

    param_desc.description = "The low force for the force demo <F_x, F_y, F_z> in newtons.";
    declare_parameter("force.low", std::vector<double>{0.0, 0.0, 0.0}, param_desc);

    param_desc.description = "The high force for the force demo <F_x, F_y, F_z> in newtons.";
    declare_parameter("force.high", std::vector<double>{0.0, 0.0, 0.0}, param_desc);

    param_desc.description = "Number of back-and-forth repetitions for the ik demo.";
    declare_parameter("ik.repeat", 0, param_desc);

    param_desc.description = "The start position in cartesian space for ik demo <X, Y, Z> in meters.";
    declare_parameter("ik.start", std::vector<double>{0.0, 0.0, 0.0}, param_desc);

    param_desc.description = "The end position in cartesian space for ik demo <X, Y, Z> in meters.";
    declare_parameter("ik.end", std::vector<double>{0.0, 0.0, 0.0}, param_desc);

    demo_ = get_parameter("demo").as_string();
    sinusoidal_repeat_ = get_parameter("sinusoidal.repeat").as_int();
    sinusoidal_joint_ = get_parameter("sinusoidal.joint").as_int();
    sinusoidal_amp_ = get_parameter("sinusoidal.amp").as_double();
    sinusoidal_freq_ = get_parameter("sinusoidal.freq").as_double();
    sinusoidal_v_offset_ = get_parameter("sinusoidal.v_offset").as_double();
    chirp_joint_ = get_parameter("chirp.joint").as_int();
    chirp_amp_ = get_parameter("chirp.amp").as_double();
    chirp_freq_init_ = get_parameter("chirp.freq_init").as_double();
    chirp_freq_final_ = get_parameter("chirp.freq_final").as_double();
    chirp_time_ = get_parameter("chirp.time").as_double();
    chirp_v_offset_ = get_parameter("chirp.v_offset").as_double();
    chirp_velocity_joint_ = get_parameter("chirp_velocity.joint").as_int();
    chirp_velocity_vel_amp_ = get_parameter("chirp_velocity.vel_amp").as_double();
    chirp_velocity_freq_init_ = get_parameter("chirp_velocity.freq_init").as_double();
    chirp_velocity_freq_final_ = get_parameter("chirp_velocity.freq_final").as_double();
    chirp_velocity_time_ = get_parameter("chirp_velocity.time").as_double();
    chirp_velocity_start_pos_ = get_parameter("chirp_velocity.start_pos").as_double_array();
    linear_start_loc_ = get_parameter("linear.start_loc").as_double_array();
    linear_end_loc_ = get_parameter("linear.end_loc").as_double_array();
    linear_step_freq_ = get_parameter("linear_step.freq").as_double();
    linear_step_repeat_ = get_parameter("linear_step.repeat").as_int();
    force_q_state_ = get_parameter("force.q_state").as_double_array();
    force_repeat_ = get_parameter("force.repeat").as_int();
    force_freq_ = get_parameter("force.freq").as_double();
    force_low_ = get_parameter("force.low").as_double_array();
    force_high_ = get_parameter("force.high").as_double_array();
    ik_repeat_ = get_parameter("ik.repeat").as_int();
    ik_start_ = get_parameter("ik.start").as_double_array();
    ik_end_ = get_parameter("ik.end").as_double_array();

    // get linear step waypoints
    std::vector<double> flat_linear_step_waypoints = get_parameter("linear_step.waypoints").as_double_array();
    for (auto i = 0; i < int(flat_linear_step_waypoints.size()); i+=3) {
      linear_step_waypoints_.push_back({float(flat_linear_step_waypoints.at(i)),
                                        float(flat_linear_step_waypoints.at(i+1)),
                                        float(flat_linear_step_waypoints.at(i+2))});
    }
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FingerDemo>());
  rclcpp::shutdown();
  return 0;
}
