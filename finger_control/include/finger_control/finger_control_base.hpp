#ifndef FINGER_CONTROL__FINGER_CONTROL_BASE_HPP_
#define FINGER_CONTROL__FINGER_CONTROL_BASE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_srvs/srv/empty.hpp"

#include "finger_interfaces/action/cartesian.hpp"
#include "finger_interfaces/action/sinusoidal.hpp"
#include "finger_interfaces/action/linear.hpp"
#include "finger_interfaces/action/force.hpp"
#include "finger_interfaces/msg/motor_feedback.hpp"

#include "fingerlib/joint_trajectory.hpp"

using namespace std::chrono_literals;

class FingerControlBase : public rclcpp::Node
{
public:
  using GoalHandleCartesian = rclcpp_action::ClientGoalHandle<finger_interfaces::action::Cartesian>;
  using GoalHandleSinusoidal = rclcpp_action::ClientGoalHandle<finger_interfaces::action::Sinusoidal>;
  using GoalHandleLinear = rclcpp_action::ClientGoalHandle<finger_interfaces::action::Linear>;
  using GoalHandleForce = rclcpp_action::ClientGoalHandle<finger_interfaces::action::Force>;
  using Cartesian = finger_interfaces::action::Cartesian;
  using Sinusoidal = finger_interfaces::action::Sinusoidal;
  using Linear = finger_interfaces::action::Linear;
  using Force = finger_interfaces::action::Force;

  explicit FingerControlBase(const std::string & node_name)
  : Node(node_name),
    ra_(0.0075), rb_(0.0025), rc_(0.0025),
    r1_(0.008), r3_(0.0045), r5_(0.008), r7_(0.0045), r9_(0.009), r11_(ra_ * 3.5),
    Ra_{{ra_, 0, 0},
        {0, rb_, 0},
        {0, 0, rc_}},
    St_{{-r11_, -r3_, r1_},
        {0, r7_, r5_},
        {0, 0, r9_}},
    slist_{arma::vec6({0, 0, 1, 0, 0, 0}),
           arma::vec6({-1, 0, 0, 0, 0, 0.01776}),
           arma::vec6({-1, 0, 0, 0, 0, 0.07776}),
           arma::vec6({-1, 0, 0, 0, 0, 0.11836})},
    joint_min_{-0.55, -0.2, -0.01},
    joint_max_{0.55, 1.572, 1.572},
    M_{{1, 0, 0, 0},
       {0, 1, 0, 0.15},
       {0, 0, 1, 0},
       {0, 0, 0, 1}},
    four_bar_lengths_{8.83765 * 0.001, 40.6 * 0.001, 8.91536 * 0.001, 37.79903 * 0.001}
  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "The relative height of the ground (should be negative).";
    declare_parameter("relative_gnd_height", -0.25, param_desc);
    gnd_height_ = get_parameter("relative_gnd_height").as_double();

    transforms_ = std::make_shared<Transformer>(Ra_, St_, slist_, M_, four_bar_lengths_,
      joint_min_, joint_max_);
    generator_ = std::make_shared<JointTrajectory>(*transforms_, 100, gnd_height_);

    wait_for_drake_heartbeat();

    cartesian_client_ = rclcpp_action::create_client<Cartesian>(this, "/cartesian_move");
    sinusoidal_client_ = rclcpp_action::create_client<Sinusoidal>(this, "/sinusoidal_move");
    linear_client_ = rclcpp_action::create_client<Linear>(this, "/linear_move");
    force_step_client_ = rclcpp_action::create_client<Force>(this, "/force_step_move");

    while (!cartesian_client_->wait_for_action_server(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "client interrupted while waiting for cartesian_move action to appear.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_logger(), "waiting for cartesian_move action to appear...");
    }
    while (!sinusoidal_client_->wait_for_action_server(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "client interrupted while waiting for sinusoidal_move action to appear.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_logger(), "waiting for sinusoidal_move action to appear...");
    }
    while (!linear_client_->wait_for_action_server(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "client interrupted while waiting for linear_move action to appear.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_logger(), "waiting for linear_move action to appear...");
    }
    while (!force_step_client_->wait_for_action_server(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "client interrupted while waiting for force_step action to appear.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_logger(), "waiting for force_step action to appear...");
    }

    motor_actual_feedback_sub_ = create_subscription<finger_interfaces::msg::MotorFeedback>(
      "/motor_pos_actual_feedback", 10,
      [this](finger_interfaces::msg::MotorFeedback::UniquePtr msg) {
        motor_actual_feedback_ = *msg;
      });

    RCLCPP_INFO(get_logger(), "Waiting for motor feedback...");
    while (motor_actual_feedback_.motor_positions.size() < 3) {
      rclcpp::spin_some(this->get_node_base_interface());
      rclcpp::sleep_for(100ms);
    }
    RCLCPP_INFO(get_logger(), "Motor feedback received, proceeding.");

    rclcpp::sleep_for(3000ms);
  }

protected:
  const double ra_, rb_, rc_;
  const double r1_, r3_, r5_, r7_, r9_, r11_;
  const arma::mat Ra_;
  const arma::mat St_;
  const std::vector<arma::vec6> slist_;
  const arma::vec joint_min_;
  const arma::vec joint_max_;
  const arma::mat44 M_;
  const std::vector<double> four_bar_lengths_;

  double gnd_height_;
  std::shared_ptr<Transformer> transforms_;
  std::shared_ptr<JointTrajectory> generator_;

  rclcpp_action::Client<Cartesian>::SharedPtr cartesian_client_;
  rclcpp_action::Client<Sinusoidal>::SharedPtr sinusoidal_client_;
  rclcpp_action::Client<Linear>::SharedPtr linear_client_;
  rclcpp_action::Client<Force>::SharedPtr force_step_client_;
  rclcpp::Subscription<finger_interfaces::msg::MotorFeedback>::SharedPtr motor_actual_feedback_sub_;
  finger_interfaces::msg::MotorFeedback motor_actual_feedback_;

  void send_cartesian_goal(std::vector<std::vector<float>> waypoints)
  {
    auto goal_msg = Cartesian::Goal();
    goal_msg.length = int(waypoints.size());
    for (auto & wp : waypoints) {
      goal_msg.x.push_back(wp.at(0));
      goal_msg.y.push_back(wp.at(1));
      goal_msg.z.push_back(wp.at(2));
    }

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Cartesian>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](const GoalHandleCartesian::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
        }
      };
    send_goal_options.feedback_callback = [this](
      GoalHandleCartesian::SharedPtr,
      const std::shared_ptr<const Cartesian::Feedback>) {
        RCLCPP_INFO(get_logger(), "feedback received...");
      };
    send_goal_options.result_callback = [this](const GoalHandleCartesian::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED: break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was aborted"); return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was canceled"); return;
          default:
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown result code"); return;
        }
        RCLCPP_INFO_STREAM(get_logger(), "result code: " << result.result.get()->success);
      };

    auto goal_handle_future = cartesian_client_->async_send_goal(goal_msg, send_goal_options);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to send goal"); return;
    }
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal rejected"); return;
    }
    auto result_future = cartesian_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to get result"); return;
    }
    RCLCPP_INFO(get_logger(), "Goal completed");
  }

  void send_sinusoid_goal(bool repeat, int joint, float amp, float freq, float v_shift)
  {
    auto goal_msg = Sinusoidal::Goal();
    goal_msg.repeat = int(repeat);
    goal_msg.joint = joint;
    goal_msg.amp = amp;
    goal_msg.freq = freq;
    goal_msg.v_shift = v_shift;

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Sinusoidal>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](const GoalHandleSinusoidal::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
        }
      };
    send_goal_options.feedback_callback = [this](
      GoalHandleSinusoidal::SharedPtr,
      const std::shared_ptr<const Sinusoidal::Feedback>) {
        RCLCPP_INFO(get_logger(), "feedback received...");
      };
    send_goal_options.result_callback = [this](const GoalHandleSinusoidal::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED: break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was aborted"); return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was canceled"); return;
          default:
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown result code"); return;
        }
        RCLCPP_INFO_STREAM(get_logger(), "result code: " << result.result.get()->success);
      };

    auto goal_handle_future = sinusoidal_client_->async_send_goal(goal_msg, send_goal_options);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to send goal"); return;
    }
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal rejected"); return;
    }
    auto result_future = sinusoidal_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to get result"); return;
    }
    RCLCPP_INFO(get_logger(), "Goal completed");
  }

  void send_linear_goal(std::vector<float> end, std::vector<float> start = {})
  {
    auto goal_msg = Linear::Goal();

    if (!start.empty()) {
      goal_msg.start = start;
    } else {
      arma::vec motor_pos = {
        motor_actual_feedback_.motor_positions.at(0),
        motor_actual_feedback_.motor_positions.at(1),
        motor_actual_feedback_.motor_positions.at(2)};
      arma::vec joint_pos = transforms_->motor_to_joint(motor_pos);
      goal_msg.start = {float(joint_pos(0)), float(joint_pos(1)), float(joint_pos(2))};
    }

    goal_msg.end = end;

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Linear>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](const GoalHandleLinear::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
        }
      };
    send_goal_options.feedback_callback = [this](
      GoalHandleLinear::SharedPtr,
      const std::shared_ptr<const Linear::Feedback>) {
        RCLCPP_INFO(get_logger(), "feedback received...");
      };
    send_goal_options.result_callback = [this](const GoalHandleLinear::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED: break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was aborted"); return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was canceled"); return;
          default:
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown result code"); return;
        }
        RCLCPP_INFO_STREAM(get_logger(), "result code: " << result.result.get()->success);
      };

    auto goal_handle_future = linear_client_->async_send_goal(goal_msg, send_goal_options);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to send goal"); return;
    }
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal rejected"); return;
    }
    auto result_future = linear_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to get result"); return;
    }
    RCLCPP_INFO(get_logger(), "Goal completed");
  }

  void send_force_step_goal(
    std::vector<float> joint_state,
    std::vector<float> force_low,
    std::vector<float> force_high,
    double frequency,
    int repeat)
  {
    auto goal_msg = Force::Goal();
    goal_msg.q_joint = joint_state;
    goal_msg.force_low = force_low;
    goal_msg.force_high = force_high;
    goal_msg.frequency = frequency;
    goal_msg.repeat = repeat;

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Force>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](const GoalHandleForce::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
        }
      };
    send_goal_options.feedback_callback = [this](
      GoalHandleForce::SharedPtr,
      const std::shared_ptr<const Force::Feedback>) {
        RCLCPP_INFO(get_logger(), "feedback received...");
      };
    send_goal_options.result_callback = [this](const GoalHandleForce::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED: break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was aborted"); return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was canceled"); return;
          default:
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown result code"); return;
        }
        RCLCPP_INFO_STREAM(get_logger(), "result code: " << result.result.get()->success);
      };

    auto goal_handle_future = force_step_client_->async_send_goal(goal_msg, send_goal_options);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to send goal"); return;
    }
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal rejected"); return;
    }
    auto result_future = force_step_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to get result"); return;
    }
    RCLCPP_INFO(get_logger(), "Goal completed");
  }

private:
  void wait_for_drake_heartbeat()
  {
    auto heartbeat_client = create_client<std_srvs::srv::Empty>("/heartbeat", 10);
    while (!heartbeat_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "client interrupted while waiting for service to appear.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_logger(), "waiting for heartbeat service to appear...");
    }
  }
};

#endif  // FINGER_CONTROL__FINGER_CONTROL_BASE_HPP_
