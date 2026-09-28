#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std;
using namespace placeholders;

class CoveragePlanner : public rclcpp::Node
{
public:
  CoveragePlanner()
  : Node("coverage_planner")
  {
    declare_parameter<std::string>("sub_topic", "/map");
    declare_parameter<std::string>("pub_topic", "/coverage_path");

    declare_parameter<double>("min_x", -47.0);
    declare_parameter<double>("max_x", 47.0);
    declare_parameter<double>("min_y", -47.0);
    declare_parameter<double>("max_y", 47.0);
    declare_parameter<double>("track_spacing", 0.25);
    declare_parameter<double>("map_margin", 3.0);

    sub_topic_ = get_parameter("sub_topic").as_string();
    pub_topic_ = get_parameter("pub_topic").as_string();

    min_x_ = get_parameter("min_x").as_double();
    max_x_ = get_parameter("max_x").as_double();
    min_y_ = get_parameter("min_y").as_double();
    max_y_ = get_parameter("max_y").as_double();
    track_spacing_ = get_parameter("track_spacing").as_double();
    map_margin_ = get_parameter("map_margin").as_double();

    rclcpp::QoS map_qos(1);
    map_qos.reliable();
    map_qos.transient_local();

    rclcpp::QoS path_qos(1);
    path_qos.reliable();
    path_qos.transient_local();

    sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      sub_topic_,
      map_qos,
    bind(
        &CoveragePlanner::mapCallback,
        this,
      _1));

    pub_ = create_publisher<nav_msgs::msg::Path>(
      pub_topic_,
      path_qos);
  }

private:
    string sub_topic_;
    string pub_topic_;

    double min_x_;
    double max_x_;
    double min_y_;
    double max_y_;
    double track_spacing_;
    double map_margin_;

    bool path_published_{false};

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;
    void addPose(nav_msgs::msg::Path &path, double x, double y, double yaw ){
        geometry_msgs::msg::PoseStamped pose;

        pose.header = path.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;

        pose.pose.orientation.z = sin(yaw / 2.0);
        pose.pose.orientation.w = cos(yaw / 2.0);

        path.poses.push_back(pose);
    }

    void mapCallback(
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
    {
    RCLCPP_INFO_ONCE(
        get_logger(),
        "Map: %u x %u | resolution=%.3f | origin=(%.2f, %.2f) | cells=%zu",
        msg->info.width,
        msg->info.height,
        msg->info.resolution,
        msg->info.origin.position.x,
        msg->info.origin.position.y,
        msg->data.size());

    if (path_published_) {
        return;
    }

    if (track_spacing_ <= 0.0 || map_margin_ < 0.0) {
        RCLCPP_ERROR(
          get_logger(), "track_spacing must be > 0 and map_margin must be >= 0");
        return;
    }
    const double map_min_x = msg->info.origin.position.x + map_margin_;
    const double map_min_y = msg->info.origin.position.y + map_margin_;
    const double map_max_x = msg->info.origin.position.x +
    static_cast<double>(msg->info.width) * msg->info.resolution - map_margin_;
    const double map_max_y = msg->info.origin.position.y +
    static_cast<double>(msg->info.height) * msg->info.resolution - map_margin_;

    const double coverage_min_x = max(min_x_, map_min_x);
    const double coverage_max_x = min(max_x_, map_max_x);
    const double coverage_min_y = max(min_y_, map_min_y);
    const double coverage_max_y = min(max_y_, map_max_y);

    if (coverage_min_x >= coverage_max_x || coverage_min_y >= coverage_max_y) {
        RCLCPP_ERROR(
          get_logger(),
          "Coverage bounds are empty after clamping to the map (margin=%.2f m)",
          map_margin_);
        return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = msg->header.frame_id;

    bool move_up = true;
    size_t lane_count = 0;

    for (double x = coverage_min_x;
      x <= coverage_max_x + 1.0e-6; x += track_spacing_)
    {
    if (move_up) {
        addPose(path, x, coverage_min_y, M_PI_2);
        addPose(path, x, coverage_max_y, M_PI_2);
    } else {
        addPose(path, x, coverage_max_y, -M_PI_2);
        addPose(path, x, coverage_min_y, -M_PI_2);
    }

    move_up = !move_up;
    ++lane_count;
    }
    pub_->publish(path);
    path_published_ = true;

    RCLCPP_INFO(
    get_logger(),
    "Published coverage path: %zu lanes, %zu poses, spacing=%.2f m, "
    "bounds=[%.2f, %.2f] x [%.2f, %.2f]",
    lane_count,
    path.poses.size(),
    track_spacing_,
    coverage_min_x,
    coverage_max_x,
    coverage_min_y,
    coverage_max_y);
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoveragePlanner>());
  rclcpp::shutdown();
  return 0;
}
