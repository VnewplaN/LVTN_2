#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

class CoveragePlanner : public rclcpp::Node
{
public:
  CoveragePlanner()
  : Node("coverage_planner")
  {
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    path_topic_ = declare_parameter<std::string>("path_topic", "/coverage_path");
    path_frame_ = declare_parameter<std::string>("path_frame", "odom");
    pond_half_extent_ =
      std::clamp(declare_parameter<double>("pond_half_extent", 47.0), 1.0, 49.0);

    track_spacing_ =
      std::max(0.05, declare_parameter<double>("track_spacing", 0.25));

    waypoint_spacing_ =
      std::max(0.05, declare_parameter<double>("waypoint_spacing", 0.25));

    clearance_ =
      std::max(0.0, declare_parameter<double>("clearance", 2.0));

    occupied_threshold_ =
      declare_parameter<int>("occupied_threshold", 65);

    min_lane_length_ =
      std::max(0.5, declare_parameter<double>("min_lane_length", 2.0));

    path_pub_ =
      create_publisher<nav_msgs::msg::Path>(
        path_topic_,
        rclcpp::QoS(1).transient_local().reliable());

    map_sub_ =
      create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_,
        rclcpp::QoS(1).transient_local().reliable(),
        std::bind(
          &CoveragePlanner::mapCallback,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Coverage planner started - waiting for saved /map");
  }

private:
  bool worldToCell(
    double wx,
    double wy,
    int & mx,
    int & my) const
  {
    if (!map_) {
      return false;
    }

    // Current saved map has zero origin yaw.
    const double res = map_->info.resolution;

    mx = static_cast<int>(
      std::floor(
        (wx - map_->info.origin.position.x) / res));

    my = static_cast<int>(
      std::floor(
        (wy - map_->info.origin.position.y) / res));

    return
      mx >= 0 &&
      my >= 0 &&
      mx < static_cast<int>(map_->info.width) &&
      my < static_cast<int>(map_->info.height);
  }

  bool cellFree(int mx, int my) const
  {
    if (
      mx < 0 ||
      my < 0 ||
      mx >= static_cast<int>(map_->info.width) ||
      my >= static_cast<int>(map_->info.height))
    {
      return false;
    }

    const auto value =
      map_->data[
        static_cast<std::size_t>(my) *
        map_->info.width +
        static_cast<std::size_t>(mx)];

    // Unknown (-1) is unsafe.
    return value >= 0 && value < occupied_threshold_;
  }

  bool pointSafe(double wx, double wy) const
  {
    int mx, my;

    if (!worldToCell(wx, wy, mx, my)) {
      return false;
    }

    if (!cellFree(mx, my)) {
      return false;
    }

    if (clearance_ <= 0.0) {
      return true;
    }

    const double res = map_->info.resolution;

    const int radius =
      static_cast<int>(
        std::ceil(clearance_ / res));

    // Sample clearance area approximately every 0.10 m.
    const int step =
      std::max(
        1,
        static_cast<int>(
          std::round(0.10 / res)));

    for (int dy = -radius; dy <= radius; dy += step)
    {
      for (int dx = -radius; dx <= radius; dx += step)
      {
        const double d =
          std::hypot(
            dx * res,
            dy * res);

        if (d > clearance_) {
          continue;
        }

        if (!cellFree(mx + dx, my + dy)) {
          return false;
        }
      }
    }

    return true;
  }

  geometry_msgs::msg::PoseStamped makePose(
    double x,
    double y,
    const std::string & frame,
    const rclcpp::Time & stamp) const
  {
    geometry_msgs::msg::PoseStamped p;

    p.header.frame_id = frame;
    p.header.stamp = stamp;

    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.position.z = 0.0;

    p.pose.orientation.x = 0.0;
    p.pose.orientation.y = 0.0;
    p.pose.orientation.z = 0.0;
    p.pose.orientation.w = 1.0;

    return p;
  }

  std::vector<std::pair<double, double>> findSafeRuns(
    double x,
    double y_min,
    double y_max) const
  {
    std::vector<std::pair<double, double>> runs;
    bool in_run = false;
    double current_start = 0.0;
    double previous_y = y_min;

    for (
      double y = y_min;
      y <= y_max + 1e-9;
      y += waypoint_spacing_)
    {
      const bool safe = pointSafe(x, y);

      if (safe && !in_run)
      {
        current_start = y;
        in_run = true;
      }

      if (!safe && in_run)
      {
        const double current_end = previous_y;
        if (current_end - current_start >= min_lane_length_) {
          runs.emplace_back(current_start, current_end);
        }
        in_run = false;
      }

      previous_y = y;
    }

    if (in_run)
    {
      if (y_max - current_start >= min_lane_length_) {
        runs.emplace_back(current_start, y_max);
      }
    }
    return runs;
  }

  void generateCoverage()
  {
    // The occupancy-grid image may be rectangular because map_saver crops to
    // observed cells. It is not the physical pond boundary. The simulated pond
    // is a 100 x 100 m square centred at odom (0, 0), so keep a 3 m safety
    // margin and generate the path directly in odom coordinates.
    const double x_min = -pond_half_extent_;
    const double x_max = pond_half_extent_;
    const double y_min = -pond_half_extent_;
    const double y_max = pond_half_extent_;

    if (x_max <= x_min || y_max <= y_min)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Invalid pond working boundary");
      return;
    }

    nav_msgs::msg::Path path;

    const std::string frame = path_frame_;

    // This is a static global path. A zero timestamp tells TF/RViz to use the
    // latest map<->odom transform, including when SLAM publishes TF after the
    // path itself has already been generated.
    const rclcpp::Time stamp(0, 0, RCL_ROS_TIME);

    path.header.frame_id = frame;
    path.header.stamp = stamp;

    bool upward = true;
    int lane_count = 0;

    /*
     * Start from right side.
     *
     * Lane 1 : bottom -> top
     * Lane 2 : top -> bottom
     * Lane 3 : bottom -> top
     */
    for (
      double x = x_max;
      x >= x_min - 1e-9;
      x -= track_spacing_)
    {
      const double lane_y_min = y_min;
      const double lane_y_max = y_max;

      if (upward)
      {
        path.poses.push_back(makePose(x, lane_y_min, frame, stamp));
        path.poses.push_back(makePose(x, lane_y_max, frame, stamp));
      }
      else
      {
        path.poses.push_back(makePose(x, lane_y_max, frame, stamp));
        path.poses.push_back(makePose(x, lane_y_min, frame, stamp));
      }

      upward = !upward;
      ++lane_count;
    }

    if (path.poses.size() < 2)
    {
      RCLCPP_ERROR(
        get_logger(),
        "No valid coverage lanes found");
      return;
    }

    path_pub_->publish(path);
    generated_ = true;

    const auto & start =
      path.poses.front().pose.position;

    const auto & end =
      path.poses.back().pose.position;

    RCLCPP_INFO(
      get_logger(),
      "============================================");

    RCLCPP_INFO(
      get_logger(),
      "FINAL MAP-BASED ZIGZAG GENERATED");

    RCLCPP_INFO(
      get_logger(),
      "Map              : %u x %u",
      map_->info.width,
      map_->info.height);

    RCLCPP_INFO(
      get_logger(),
      "Resolution       : %.3f m",
      map_->info.resolution);

    RCLCPP_INFO(
      get_logger(),
      "Origin           : (%.2f, %.2f)",
      map_->info.origin.position.x,
      map_->info.origin.position.y);

    RCLCPP_INFO(
      get_logger(),
      "Working boundary : X[%.2f, %.2f] Y[%.2f, %.2f]",
      x_min,
      x_max,
      y_min,
      y_max);

    RCLCPP_INFO(
      get_logger(),
      "Pond margin      : %.2f m",
      50.0 - pond_half_extent_);

    RCLCPP_INFO(
      get_logger(),
      "Track spacing    : %.2f m",
      track_spacing_);

    RCLCPP_INFO(
      get_logger(),
      "Waypoint spacing : %.2f m",
      waypoint_spacing_);

    RCLCPP_INFO(
      get_logger(),
      "Number of lanes  : %d",
      lane_count);

    RCLCPP_INFO(
      get_logger(),
      "Number of poses  : %zu",
      path.poses.size());

    RCLCPP_INFO(
      get_logger(),
      "Start            : (%.2f, %.2f)",
      start.x,
      start.y);

    RCLCPP_INFO(
      get_logger(),
      "End              : (%.2f, %.2f)",
      end.x,
      end.y);

    RCLCPP_INFO(
      get_logger(),
      "Published        : %s",
      path_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "============================================");
  }

  void mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (generated_) {
      return;
    }

    map_ = msg;

    RCLCPP_INFO(
      get_logger(),
      "Saved map received: %u x %u | resolution=%.3f | "
      "origin=(%.2f, %.2f)",
      map_->info.width,
      map_->info.height,
      map_->info.resolution,
      map_->info.origin.position.x,
      map_->info.origin.position.y);

    generateCoverage();
  }

  std::string map_topic_;
  std::string path_topic_;
  std::string path_frame_;

  double track_spacing_;
  double waypoint_spacing_;
  double clearance_;
  double min_lane_length_;
  double pond_half_extent_;

  int occupied_threshold_;

  bool generated_{false};

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;

  rclcpp::Subscription<
    nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  rclcpp::Publisher<
    nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
  std::make_shared<CoveragePlanner>());

  rclcpp::shutdown();

  return 0;
}
