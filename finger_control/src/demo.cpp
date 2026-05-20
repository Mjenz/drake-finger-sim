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
class FingerDemo : public FingerControlBase
{
public:

  /// \brief Create an instance of FingerDemo for running demos
  FingerDemo()
  : FingerControlBase("finger_demo")
  {
    auto param_desc1 = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc1.description = "The demo that should be run.";
    declare_parameter("demo", "none", param_desc1);
    std::string demo = get_parameter("demo").as_string();

    if (demo =="linear") {
      RCLCPP_INFO(get_logger(), "Running linear joint movement demo...");

      std::vector<float> start_joint_loc = {0.0, 0.0, 0.0};
      std::vector<float> end_joint_loc = {0.0, 0.0, 1.256};
      for (auto i = 0; i < 10; i++) {
        send_linear_goal(end_joint_loc, start_joint_loc);
        send_linear_goal(start_joint_loc, end_joint_loc);
      }
    }

    else if (demo =="sinusoidal") {
      RCLCPP_INFO(get_logger(), "Running sinusoidal movement demo...");
      send_sinusoid_goal(1, 0, 0.1, 1.0, 0.0);
    }

    else if (demo =="ik") { 
      RCLCPP_INFO(get_logger(), "Running inverse kinematics demo...");   
      std::vector<float> start = {0.05f, 0.08f, -0.1f};            
      std::vector<float> end   = {-0.05f, 0.08f, -0.1f};

      // move out of singularity
      send_linear_goal({0.1, 0.1, 0.1});

      // move to start
      send_cartesian_goal({start});        
                                           
      for (auto i = 0; i < 20; i++) {
          send_cartesian_goal({start, end});        
          send_cartesian_goal({end, start});
      }
    }

    else if (demo =="force_step") {   
      RCLCPP_INFO(get_logger(), "Running force step demo...");   
      std::vector<float> q_joints = {0.0f, 0.0f, 0.0f};
      std::vector<float> force_low = {0.0f, 0.0f, -10.0f};
      std::vector<float> force_high = {0.0f, 0.0f, -10.0f};

      send_force_step_goal(q_joints, force_low, force_high, 1.0, 1);
    }

    else if (demo =="cartesian_ik") {   
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

        std::vector<float> start = {0.05f, 0.1f, -0.1f};            
        std::vector<float> end   = {-0.05f, 0.1f, -0.1f};
                                                                    
        for (auto i = 0; i < 20; i++) {
            send_cartesian_goal(lerp_waypoints(start, end));        
            send_cartesian_goal(lerp_waypoints(end, start));
        }    
      }
  }

private:

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FingerDemo>());
  rclcpp::shutdown();
  return 0;
}
