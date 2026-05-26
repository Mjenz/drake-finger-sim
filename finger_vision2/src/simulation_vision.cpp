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
class SimulationVision : public rclcpp::Node
{
public:
  SimulationVision()
  : Node("simulation_vision"),
    id_(0),
    has_marker_(false),
    count_(0)
  {
    declare_and_get_finger_params();
    
    // create transformer
    transforms_ = std::make_shared<Transformer>(Ra_, St_, slist_, M_, four_bar_lengths_, joint_min_, joint_max_);

    // create publishers
    auto qos = rclcpp::QoS(10).transient_local();
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/goals", qos);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // init uniform distributions
    splay_d_ = std::uniform_real_distribution<>(joint_min_.at(0) + 0.1, joint_max_.at(0) - 0.1);
    mcpflex_d_ = std::uniform_real_distribution<>(joint_min_.at(1) + 0.1, joint_max_.at(1) - 0.1);
    pipflex_d_ = std::uniform_real_distribution<>(joint_min_.at(2) + 0.1, joint_max_.at(2) - 0.1);
    norm_count_d_ = std::normal_distribution<>(10.0, 5.0);

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

    RCLCPP_INFO(get_logger(), "simulation_vision node started");
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

  double ra_, rb_, rc_;
  double r1_, r3_, r5_, r7_, r9_, r11_;
  arma::mat Ra_;
  arma::mat St_;
  std::vector<arma::vec6> slist_;
  arma::vec joint_min_;
  arma::vec joint_max_;
  arma::mat44 M_;
  std::vector<double> four_bar_lengths_;
  std::shared_ptr<Transformer> transforms_;


  void declare_and_get_finger_params()
  {
    declare_parameter("ra", 0.0);
    declare_parameter("rb", 0.0);
    declare_parameter("rc", 0.0);
    declare_parameter("r1", 0.0);
    declare_parameter("r3", 0.0);
    declare_parameter("r5", 0.0);
    declare_parameter("r7", 0.0);
    declare_parameter("r9", 0.0);
    declare_parameter("r11", 0.0);
    declare_parameter("slist", std::vector<double>{
        0, 0, 1, 0, 0, 0,
        -1, 0, 0, 0, 0, 0.0178,
        -1, 0, 0, 0, 0, 0.079,
        -1, 0, 0, 0, 0, 0.1195});
    declare_parameter("joint_min", std::vector<double>{-0.55, -0.2, -0.01});
    declare_parameter("joint_max", std::vector<double>{0.55, 1.572, 1.572});
    declare_parameter("M", std::vector<double>{
        1, 0, 0, 0,
        0, 1, 0, 0.16,
        0, 0, 1, 0,
        0, 0, 0, 1});
    declare_parameter("four_bar_lengths", std::vector<double>{
        8.83765e-3, 40.6e-3, 8.91536e-3, 37.79903e-3});

    ra_ = get_parameter("ra").as_double();
    rb_ = get_parameter("rb").as_double();
    rc_ = get_parameter("rc").as_double();
    r1_ = get_parameter("r1").as_double();
    r3_ = get_parameter("r3").as_double();
    r5_ = get_parameter("r5").as_double();
    r7_ = get_parameter("r7").as_double();
    r9_ = get_parameter("r9").as_double();
    r11_ = get_parameter("r11").as_double();
    Ra_ = arma::diagmat(arma::vec3{ra_, rb_, rc_});
    St_ = arma::mat{{r11_, r3_, r1_}, {0.0, r7_, r5_}, {0.0, 0.0, r9_}};

    auto slist_flat = get_parameter("slist").as_double_array();
    slist_ = {
        arma::vec6(slist_flat.data()),
        arma::vec6(slist_flat.data() + 6),
        arma::vec6(slist_flat.data() + 12),
        arma::vec6(slist_flat.data() + 18)};

    auto jmin_flat = get_parameter("joint_min").as_double_array();
    joint_min_ = arma::vec(jmin_flat);

    auto jmax_flat = get_parameter("joint_max").as_double_array();
    joint_max_ = arma::vec(jmax_flat);

    auto M_flat = get_parameter("M").as_double_array();
    M_ = arma::mat44(arma::mat(M_flat.data(), 4, 4).t());

    auto fbl_flat = get_parameter("four_bar_lengths").as_double_array();
    four_bar_lengths_ = std::vector<double>(fbl_flat.begin(), fbl_flat.end());
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulationVision>());
  rclcpp::shutdown();
  return 0;
};
