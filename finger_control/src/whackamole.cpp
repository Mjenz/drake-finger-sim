/// \file
/// \brief Runs high level control coordinating perception and movement commands.
///        Waits for the Drake simulation heartbeat, then dispatches cartesian,
///        sinusoidal, and linear trajectory goals to the finger planner.
///
/// CLIENTS:
///   + /heartbeat (std_srvs/srv/Empty) - Blocks startup until the Drake simulation is ready
///   + /cartesian_move (finger_interfaces/action/Cartesian) - Sends end-effector waypoint goals
///   + /sinusoidal_move (finger_interfaces/action/Sinusoidal) - Sends sinusoidal joint trajectory goals
///   + /linear_move (finger_interfaces/action/Linear) - Sends linear joint-space trajectory goals

#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <armadillo>

#include "finger_control/finger_control_base.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;


/// \brief State variable for moving to published goal points
enum FingerState
{
  MOVING,
  IDLE,
};

/// \brief A class that bridges commands and feedback between ros and drake
class FingerWhackamole : public FingerControlBase
{
public:

  /// \brief Create an instance of FingerWhackamole running the whackamole demo
  FingerWhackamole()
  : FingerControlBase("finger_whackamole"),
    finger_state_ (FingerState::IDLE),
    goal_count_ (0),
    thresh_ (0.001)
  {
    // init tf2
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // init tfs
    prev_tf_.transform.translation.x = 0.0;
    prev_tf_.transform.translation.y = 0.0;
    prev_tf_.transform.translation.z = 0.0;
    goal_tf_.transform.translation.x = 0.0;
    goal_tf_.transform.translation.y = 0.0;
    goal_tf_.transform.translation.z = 0.0;

    send_linear_goal({0.0, 0.1, 0.1});

    // init frame names
    fromFrameRel_ = "base_frame";
    toFrameRel_ = "goal";
    // toFrameRel_ = "phone_frame";

    // define timer callback and init
    auto hyper_alg_timer_cb =
      [this]() -> void {

        switch (finger_state_) {

          case FingerState::IDLE:

            // listen for new goal positions
            try {
              goal_tf_ = tf_buffer_->lookupTransform(
                fromFrameRel_, toFrameRel_,
                tf2::TimePointZero);
              
              if (!similar_tfs(goal_tf_, prev_tf_)) {
                // if successful switch state and move finger to this location
                finger_state_ = FingerState::MOVING;
              }

            } catch (const tf2::TransformException & ex) {
              RCLCPP_INFO_ONCE(get_logger(), "Could not transform %s to %s: %s",
                fromFrameRel_.c_str(), toFrameRel_.c_str(), ex.what());
              return;
            }
            break;
            
          case FingerState::MOVING:
            // convert to vector
            std::vector<float> above_goal = {float(goal_tf_.transform.translation.x),
                                               float(goal_tf_.transform.translation.y),
                                               float(goal_tf_.transform.translation.z + 0.02f)};
            std::vector<float> goal = {float(goal_tf_.transform.translation.x),
                                       float(goal_tf_.transform.translation.y),
                                       float(goal_tf_.transform.translation.z)};

            // just the goal
            send_cartesian_goal({goal});
            send_linear_goal({0.0, 0.025, 1.25});
            // send_cartesian_goal({above_goal});
            // send_cartesian_goal({above_goal, goal});
            // send_cartesian_goal({goal, above_goal});
            // send_linear_goal({0.0, 0.025, 1.25});

            // update prev_tf
            prev_tf_ = goal_tf_;

            // update state
            finger_state_ = FingerState::IDLE;

            break;
          }        
      };
      
    timer_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = create_wall_timer(100ms, hyper_alg_timer_cb, timer_cb_group_);

  
  }

private:
  FingerState finger_state_;
  int goal_count_;
  float thresh_;

  std::string fromFrameRel_;
  std::string toFrameRel_;
  geometry_msgs::msg::TransformStamped goal_tf_;
  geometry_msgs::msg::TransformStamped prev_tf_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

  bool similar_tfs(geometry_msgs::msg::TransformStamped tf1, geometry_msgs::msg::TransformStamped tf2) {

    auto x_diff = fabs(tf1.transform.translation.x - tf2.transform.translation.x);
    auto y_diff = fabs(tf1.transform.translation.y - tf2.transform.translation.y);
    auto z_diff = fabs(tf1.transform.translation.z - tf2.transform.translation.z);

    if ((x_diff > thresh_) || (y_diff > thresh_) || (z_diff > thresh_)) {
      return false;
    } else {
      return true;
    }
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FingerWhackamole>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
