#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class MappingRoute : public rclcpp::Node
{
public:
  MappingRoute()
  : Node("mapping_route")
  {
    state_topic_ = this->declare_parameter<std::string>("state_topic", "/state_value");
    command_topic_ = this->declare_parameter<std::string>("command_topic", "/trajectory_cmd");
    speed_ = this->declare_parameter<double>("speed", 0.8);
    arrival_radius_ = this->declare_parameter<double>("arrival_radius", 2.0);
    const double route_limit = this->declare_parameter<double>("route_limit", 45.0);

    // A clockwise lap 5 m inside the 100 x 100 m pond boundary.
    // This keeps the banks clearly visible while leaving room to turn safely.
    waypoints_ = {
      {route_limit, -route_limit},
      {-route_limit, -route_limit},
      {-route_limit, route_limit},
      {route_limit, route_limit},
    };

    command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, 10);
    state_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      state_topic_, 10,
      std::bind(&MappingRoute::stateCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&MappingRoute::controlLoop, this));
  }

private:
  static double wrapAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void stateCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) {
      return;
    }
    x_ = msg->data[0];
    y_ = msg->data[1];
    psi_ = msg->data[2];
    state_received_ = true;

    if (!route_initialized_) {
      double best_score = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < waypoints_.size(); ++i) {
        const double dx = waypoints_[i].first - x_;
        const double dy = waypoints_[i].second - y_;
        const double distance = std::hypot(dx, dy);
        const double heading_error = wrapAngle(std::atan2(dy, dx) - psi_);
        // Prefer a waypoint ahead of the vessel so restarting the route does
        // not make the USV turn around and retrace the segment it just mapped.
        const double score = distance + 50.0 * (1.0 - std::cos(heading_error));
        if (score < best_score) {
          best_score = score;
          waypoint_index_ = i;
        }
      }
      route_initialized_ = true;
      RCLCPP_INFO(
        this->get_logger(), "Starting mapping lap at waypoint %zu: (%.1f, %.1f)",
        waypoint_index_ + 1, waypoints_[waypoint_index_].first, waypoints_[waypoint_index_].second);
    }
  }

  void publishCommand(double u, double heading)
  {
    std_msgs::msg::Float64MultiArray command;
    command.data = {u, 0.0, heading};
    command_pub_->publish(command);
  }

  void controlLoop()
  {
    if (!state_received_ || !route_initialized_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "Waiting for /state_value");
      return;
    }

    if (route_complete_) {
      publishCommand(0.0, psi_);
      return;
    }

    const auto & target = waypoints_[waypoint_index_];
    const double dx = target.first - x_;
    const double dy = target.second - y_;
    const double distance = std::hypot(dx, dy);

    if (distance <= arrival_radius_) {
      ++visited_waypoints_;
      // The first reached waypoint only places the USV at the start corner.
      // Return to that same corner after visiting the other three corners so
      // the mapping route contains all four sides of the pond.
      const std::size_t required_visits = waypoints_.size() + 1;
      RCLCPP_INFO(
        this->get_logger(), "Reached route point %zu/%zu at (%.2f, %.2f)",
        visited_waypoints_, required_visits, x_, y_);
      if (visited_waypoints_ >= required_visits) {
        route_complete_ = true;
        publishCommand(0.0, psi_);
        RCLCPP_INFO(this->get_logger(), "Mapping lap complete; USV stopped");
        return;
      }
      waypoint_index_ = (waypoint_index_ + 1) % waypoints_.size();
      return;
    }

    const double desired_heading = std::atan2(dy, dx);
    const double heading_error = wrapAngle(desired_heading - psi_);
    double desired_speed = std::min(speed_, 0.25 * distance);
    if (std::abs(heading_error) > 0.35) {
      desired_speed = 0.0;
    } else {
      desired_speed *= std::max(0.25, std::cos(heading_error));
    }
    publishCommand(desired_speed, desired_heading);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Waypoint %zu | pose=(%.1f, %.1f, %.2f) target=(%.1f, %.1f) distance=%.1f",
      waypoint_index_ + 1, x_, y_, psi_, target.first, target.second, distance);
  }

  std::string state_topic_;
  std::string command_topic_;
  double speed_;
  double arrival_radius_;
  double x_{0.0};
  double y_{0.0};
  double psi_{0.0};
  bool state_received_{false};
  bool route_initialized_{false};
  bool route_complete_{false};
  std::size_t waypoint_index_{0};
  std::size_t visited_waypoints_{0};
  std::vector<std::pair<double, double>> waypoints_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MappingRoute>());
  rclcpp::shutdown();
  return 0;
}
