#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class LaserScanMerger : public rclcpp::Node
{
public:
  LaserScanMerger()
  : Node("laser_scan_merger"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    target_frame_ = this->declare_parameter<std::string>("target_frame", "usv_body");
    front_topic_ = this->declare_parameter<std::string>(
      "front_topic", "/lidar/front/scan");
    back_topic_ = this->declare_parameter<std::string>(
      "back_topic", "/lidar/back/scan");
    output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "/lidar/merged_scan");
    output_samples_ = std::max(
      360, static_cast<int>(this->declare_parameter<int64_t>("output_samples", 2880)));

    auto qos = rclcpp::SensorDataQoS();
    front_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      front_topic_, qos,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        front_scan_ = msg;
      });
    back_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      back_topic_, qos,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        back_scan_ = msg;
      });

    merged_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      output_topic_, rclcpp::SensorDataQoS());
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&LaserScanMerger::mergeAndPublish, this));

    RCLCPP_INFO(
      this->get_logger(), "Merging %s and %s into %s in frame %s",
      front_topic_.c_str(), back_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str());
  }

private:
  static tf2::Transform toTf(const geometry_msgs::msg::Transform & transform)
  {
    tf2::Quaternion rotation(
      transform.rotation.x, transform.rotation.y,
      transform.rotation.z, transform.rotation.w);
    tf2::Vector3 translation(
      transform.translation.x, transform.translation.y, transform.translation.z);
    return tf2::Transform(rotation, translation);
  }

  void addScan(
    const sensor_msgs::msg::LaserScan & scan,
    const tf2::Transform & sensor_to_target,
    sensor_msgs::msg::LaserScan & output)
  {
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const float range = scan.ranges[i];
      if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
        continue;
      }

      const double sensor_angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const tf2::Vector3 sensor_point(
        static_cast<double>(range) * std::cos(sensor_angle),
        static_cast<double>(range) * std::sin(sensor_angle), 0.0);
      const tf2::Vector3 target_point = sensor_to_target * sensor_point;
      const double planar_range = std::hypot(target_point.x(), target_point.y());
      const double target_angle = std::atan2(target_point.y(), target_point.x());

      int index = static_cast<int>(std::lround(
        (target_angle - output.angle_min) / output.angle_increment));
      index = std::clamp(index, 0, output_samples_ - 1);

      if (planar_range >= output.range_min && planar_range <= output.range_max) {
        output.ranges[static_cast<std::size_t>(index)] = std::min(
          output.ranges[static_cast<std::size_t>(index)], static_cast<float>(planar_range));
      }
    }
  }

  void mergeAndPublish()
  {
    sensor_msgs::msg::LaserScan::ConstSharedPtr front;
    sensor_msgs::msg::LaserScan::ConstSharedPtr back;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      front = front_scan_;
      back = back_scan_;
    }
    if (!front || !back) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for both front and back LaserScan messages");
      return;
    }

    geometry_msgs::msg::TransformStamped front_transform;
    geometry_msgs::msg::TransformStamped back_transform;
    try {
      front_transform = tf_buffer_.lookupTransform(
        target_frame_, front->header.frame_id, tf2::TimePointZero);
      back_transform = tf_buffer_.lookupTransform(
        target_frame_, back->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Cannot transform laser scans: %s", error.what());
      return;
    }

    sensor_msgs::msg::LaserScan output;
    output.header.stamp = front->header.stamp;
    if (rclcpp::Time(back->header.stamp) > rclcpp::Time(front->header.stamp)) {
      output.header.stamp = back->header.stamp;
    }
    output.header.frame_id = target_frame_;
    output.angle_min = -static_cast<float>(M_PI);
    output.angle_max = static_cast<float>(M_PI);
    output.angle_increment = static_cast<float>(2.0 * M_PI / (output_samples_ - 1));
    output.time_increment = 0.0F;
    output.scan_time = std::max(front->scan_time, back->scan_time);
    output.range_min = std::min(front->range_min, back->range_min);
    output.range_max = std::max(front->range_max, back->range_max);
    output.ranges.assign(
      static_cast<std::size_t>(output_samples_), std::numeric_limits<float>::infinity());

    addScan(*front, toTf(front_transform.transform), output);
    addScan(*back, toTf(back_transform.transform), output);
    merged_pub_->publish(output);
  }

  std::string target_frame_;
  std::string front_topic_;
  std::string back_topic_;
  std::string output_topic_;
  int output_samples_;

  std::mutex scan_mutex_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr front_scan_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr back_scan_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr front_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr back_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr merged_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanMerger>());
  rclcpp::shutdown();
  return 0;
}
