#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/msg/entity.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"

class CoveragePlayback : public rclcpp::Node
{
public:
  CoveragePlayback() : Node("coverage_playback")
  {
    duration_ = std::max(1.0, declare_parameter<double>("duration", 60.0));
    model_name_ = declare_parameter<std::string>("model_name", "usv");
    service_name_ = declare_parameter<std::string>(
      "service_name", "/world/industrial_sludge_pond/set_pose");
    z_ = declare_parameter<double>("z", 0.2817);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/coverage_path", rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&CoveragePlayback::pathCallback, this, std::placeholders::_1));
    client_ = create_client<ros_gz_interfaces::srv::SetEntityPose>(service_name_);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&CoveragePlayback::tick, this));
    RCLCPP_INFO(get_logger(), "Waiting for /coverage_path; playback duration %.1f s", duration_);
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.size() < 2 || started_) return;
    path_ = msg->poses;
    cumulative_.assign(path_.size(), 0.0);
    for (std::size_t i = 1; i < path_.size(); ++i) {
      const auto & a = path_[i - 1].pose.position;
      const auto & b = path_[i].pose.position;
      cumulative_[i] = cumulative_[i - 1] + std::hypot(b.x - a.x, b.y - a.y);
    }
    total_length_ = cumulative_.back();
    start_ = std::chrono::steady_clock::now();
    started_ = total_length_ > 0.0;
    RCLCPP_INFO(
      get_logger(), "Playback started: %zu poses, %.1f m compressed into %.1f s",
      path_.size(), total_length_, duration_);
  }

  void tick()
  {
    if (!started_ || finished_ || request_pending_) return;
    if (!client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for %s", service_name_.c_str());
      return;
    }
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_).count();
    const double ratio = std::clamp(elapsed / duration_, 0.0, 1.0);
    const double target = ratio * total_length_;
    auto upper = std::lower_bound(cumulative_.begin(), cumulative_.end(), target);
    std::size_t i = static_cast<std::size_t>(std::distance(cumulative_.begin(), upper));
    i = std::clamp<std::size_t>(i, 1, path_.size() - 1);
    const auto & a = path_[i - 1].pose.position;
    const auto & b = path_[i].pose.position;
    const double segment = cumulative_[i] - cumulative_[i - 1];
    const double t = segment > 1e-9 ? (target - cumulative_[i - 1]) / segment : 0.0;
    const double x = a.x + t * (b.x - a.x);
    const double y = a.y + t * (b.y - a.y);
    const double bow_yaw = std::atan2(b.y - a.y, b.x - a.x);
    const double model_yaw = bow_yaw - M_PI_2;

    auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
    request->entity.name = model_name_;
    request->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
    request->pose.position.x = x;
    request->pose.position.y = y;
    request->pose.position.z = z_;
    request->pose.orientation.z = std::sin(model_yaw * 0.5);
    request->pose.orientation.w = std::cos(model_yaw * 0.5);
    request_pending_ = true;
    client_->async_send_request(
      request, [this, ratio](rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedFuture future) {
        request_pending_ = false;
        if (!future.get()->success) RCLCPP_ERROR(get_logger(), "Gazebo rejected pose update");
        if (ratio >= 1.0) {
          finished_ = true;
          RCLCPP_INFO(get_logger(), "Coverage playback completed in %.1f seconds", duration_);
        }
      });
  }

  double duration_{60.0};
  double z_{0.2817};
  double total_length_{0.0};
  std::string model_name_;
  std::string service_name_;
  bool started_{false};
  bool finished_{false};
  bool request_pending_{false};
  std::chrono::steady_clock::time_point start_;
  std::vector<geometry_msgs::msg::PoseStamped> path_;
  std::vector<double> cumulative_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoveragePlayback>());
  rclcpp::shutdown();
  return 0;
}
