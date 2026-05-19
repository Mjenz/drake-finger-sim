/// \file
/// \brief Simulates face detections by publishing random bounding boxes and visualization markers at randomized intervals.
///
/// Face detections are generated with uniformly distributed image coordinates and
/// normally distributed depth. Markers are published in base_link frame at the
/// projected 3D position of the detection.
///
/// PUBLISHERS:
///   + ~/detections (ryan_interfaces/msg/Detection) - Publishes bounding boxes, image size, and depth of detected faces
///   + ~/bbox_marker (visualization_msgs/msg/Marker) - Publishes a 3D sphere marker for the current detection in base_link
///   + /tf (tf2_msgs/msg/TFMessage) - Broadcasts goal frame relative to speedster_finger/base_link

#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <armadillo>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"

#include "fingerlib/transformer.hpp"

using namespace std::chrono_literals;

std::mt19937 & get_random()
{
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{}; 
    static std::mt19937 mt{rd()};

    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
    return mt;
}

/// \brief Publishes random points for the finger to go towards
class FakeVision : public rclcpp::Node
{
public:
  FakeVision()
  : Node("fake_vision"),
    id_(0),
    has_marker_(false),
    count_(0),
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
    // create transformer
    transforms_ = std::make_shared<Transformer>(Ra_, St_, slist_, M_, four_bar_lengths_, joint_min_, joint_max_);

    // create publishers
    auto qos = rclcpp::QoS(10).transient_local();
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/goals", qos);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // init uniform distributions
    splay_d_ = std::uniform_real_distribution<>(joint_min_.at(0), joint_max_.at(0));
    mcpflex_d_ = std::uniform_real_distribution<>(joint_min_.at(1), joint_max_.at(1));
    pipflex_d_ = std::uniform_real_distribution<>(joint_min_.at(2), joint_max_.at(2));
    norm_count_d_ = std::normal_distribution<>(50.0, 10.0);

    // init max count
    max_count_ =  std::max(1, static_cast<int>(norm_count_d_(get_random())));

    auto timer_callback =
      [this]()-> void {

        if (count_ % max_count_ == 0)
        {
            // update stored marker from new detection
            updateMarker();

            // reset timer max
            max_count_ = std::max(1, static_cast<int>(norm_count_d_(get_random())));
            count_ = 0;
        }

        // republish current marker and goal tf until the next detection arrives
        if (has_marker_) {
            marker_.header.stamp = now();
            marker_pub_->publish(marker_);
            goal_tf_.header.stamp = now();
            tf_broadcaster_->sendTransform(goal_tf_);
        }

        count_++;
      };

    timer_ = create_wall_timer(100ms, timer_callback);

    RCLCPP_INFO(get_logger(), "fake_vision node started");
  }

private:
  void updateMarker()
{
    arma::vec rand_joint_q = {splay_d_(get_random()), mcpflex_d_(get_random()), pipflex_d_(get_random())};

    arma::mat44 rand_cartesian = transforms_->joint_to_end_effector(rand_joint_q);
        
    goal_tf_.header.frame_id = "base_frame";
    goal_tf_.child_frame_id = "goal"; //_" + std::to_string(id_++);
    goal_tf_.transform.translation.x = rand_cartesian(0,3);
    goal_tf_.transform.translation.y = rand_cartesian(1,3);
    goal_tf_.transform.translation.z = rand_cartesian(2,3);
    goal_tf_.transform.rotation.w = 1.0;

    marker_.header.frame_id = "base_frame";
    marker_.id = 0;
    marker_.type = visualization_msgs::msg::Marker::SPHERE;
    marker_.action = visualization_msgs::msg::Marker::ADD;
    marker_.pose.position.x = rand_cartesian(0,3);
    marker_.pose.position.y = rand_cartesian(1,3);
    marker_.pose.position.z = rand_cartesian(2,3);
    // marker_.pose.position.x = 1.0;
    // marker_.pose.position.y = 1.0;
    // marker_.pose.position.z = 1.0;
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;
    marker_.scale.x = 0.02;
    marker_.scale.y = 0.02;
    marker_.scale.z = 0.02;
    marker_.color.r = 0.0f;
    marker_.color.g = 1.0f;
    marker_.color.b = 0.0f;
    marker_.color.a = 0.8f;
    has_marker_ = true;
}
  int id_;
  bool has_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::uniform_real_distribution<> splay_d_;
  std::uniform_real_distribution<> mcpflex_d_;
  std::uniform_real_distribution<> pipflex_d_;
  std::normal_distribution<> norm_count_d_;
  visualization_msgs::msg::Marker marker_;
  geometry_msgs::msg::TransformStamped goal_tf_;
  int count_;
  int max_count_;

  const double ra_, rb_, rc_;
  const double r1_, r3_, r5_, r7_, r9_, r11_;
  const arma::mat Ra_;
  const arma::mat St_;
  const std::vector<arma::vec6> slist_;
  const arma::vec joint_min_;
  const arma::vec joint_max_;
  const arma::mat44 M_;
  const std::vector<double> four_bar_lengths_;
  std::shared_ptr<Transformer> transforms_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeVision>());
  rclcpp::shutdown();
  return 0;
};
