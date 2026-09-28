#include <array>
#include <fstream>
#include <regex>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

class MapTransformLoader : public rclcpp::Node
{
public:
  MapTransformLoader()
  : Node("map_transform_loader"), broadcaster_(this)
  {
    declare_parameter<std::string>("transform_file", "");
    const auto path = get_parameter("transform_file").as_string();
    if (path.empty()) {
      throw std::runtime_error("parameter 'transform_file' must not be empty");
    }

    world_to_map_ = load(path);
    tf2::Transform world_from_map;
    tf2::fromMsg(world_to_map_.transform, world_from_map);
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = world_to_map_.header.stamp;
    map_to_odom.header.frame_id = world_to_map_.child_frame_id;
    map_to_odom.child_frame_id = "odom";
    map_to_odom.transform = tf2::toMsg(world_from_map.inverse());
    broadcaster_.sendTransform({world_to_map_, map_to_odom});
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/usv_pose_in_map", 10);
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        publishMapPose(*message);
      });
    RCLCPP_INFO(
      get_logger(), "Loaded saved transform %s -> %s from %s",
      world_to_map_.header.frame_id.c_str(), world_to_map_.child_frame_id.c_str(),
      path.c_str());
  }

private:
  tf2_ros::StaticTransformBroadcaster broadcaster_;
  geometry_msgs::msg::TransformStamped world_to_map_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;

  void publishMapPose(const nav_msgs::msg::Odometry & odometry)
  {
    tf2::Transform world_from_map;
    tf2::fromMsg(world_to_map_.transform, world_from_map);

    geometry_msgs::msg::TransformStamped map_from_world;
    map_from_world.header.stamp = odometry.header.stamp;
    map_from_world.header.frame_id = world_to_map_.child_frame_id;
    map_from_world.child_frame_id = world_to_map_.header.frame_id;
    map_from_world.transform = tf2::toMsg(world_from_map.inverse());

    geometry_msgs::msg::PoseStamped world_pose;
    world_pose.header = odometry.header;
    world_pose.header.frame_id = world_to_map_.header.frame_id;
    world_pose.pose = odometry.pose.pose;

    geometry_msgs::msg::PoseStamped map_pose;
    tf2::doTransform(world_pose, map_pose, map_from_world);
    pose_publisher_->publish(map_pose);
  }

  static std::string scalar(
    const std::string & text, const std::string & key)
  {
    std::smatch match;
    const std::regex pattern("(?:^|\\n)\\s*" + key + "\\s*:\\s*([^\\r\\n#]+)");
    if (!std::regex_search(text, match, pattern)) {
      throw std::runtime_error("missing key '" + key + "'");
    }
    auto value = match[1].str();
    value.erase(0, value.find_first_not_of(" \\t\"'"));
    value.erase(value.find_last_not_of(" \\t\"'") + 1);
    return value;
  }

  template<std::size_t N>
  static std::array<double, N> vector(
    const std::string & text, const std::string & key)
  {
    const auto value = scalar(text, key);
    std::array<double, N> result{};
    static const std::regex number(
      "[-+]?(?:[0-9]*\\.[0-9]+|[0-9]+\\.?)(?:[eE][-+]?[0-9]+)?");
    auto begin = std::sregex_iterator(value.begin(), value.end(), number);
    const auto end = std::sregex_iterator();
    std::size_t index = 0;
    for (; begin != end && index < N; ++begin, ++index) {
      result[index] = std::stod(begin->str());
    }
    if (index != N || begin != end) {
      throw std::runtime_error("key '" + key + "' has the wrong vector length");
    }
    return result;
  }

  geometry_msgs::msg::TransformStamped load(const std::string & path)
  {
    std::ifstream input(path);
    if (!input) {
      throw std::runtime_error("cannot open " + path);
    }
    const std::string text(
      (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = scalar(text, "parent_frame");
    transform.child_frame_id = scalar(text, "child_frame");
    const auto t = vector<3>(text, "translation");
    const auto q = vector<4>(text, "rotation");
    transform.transform.translation.x = t[0];
    transform.transform.translation.y = t[1];
    transform.transform.translation.z = t[2];
    transform.transform.rotation.x = q[0];
    transform.transform.rotation.y = q[1];
    transform.transform.rotation.z = q[2];
    transform.transform.rotation.w = q[3];
    return transform;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MapTransformLoader>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("map_transform_loader"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
