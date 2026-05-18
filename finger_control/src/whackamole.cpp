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

using namespace std::chrono_literals;

/// \brief A class that bridges commands and feedback between ros and drake
class FingerControl : public FingerControlBase
{
public:

  /// \brief Create an instance of FingerControl running the whackamole demo
  FingerControl()
  : FingerControlBase("finger_control")
  {
    // define timer callback and init
    auto hyper_alg_timer_cb =
      [this]() -> void {

        RCLCPP_INFO_ONCE(get_logger(), "idle ...");
        // get tfs

        // check if there is a target tf


        
      };
    timer_ = this->create_wall_timer(100ms, hyper_alg_timer_cb);
  
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FingerControl>());
  rclcpp::shutdown();
  return 0;
}
