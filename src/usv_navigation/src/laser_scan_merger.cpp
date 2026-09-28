#include "rclcpp/rclcpp.hpp"
#include "memory"
#include "string"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <functional>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"

using namespace std;
using namespace placeholders;

class LaserScanMerger : public rclcpp::Node
{
    public:
    LaserScanMerger() : Node("laser_scan_merger"){
    //default value
    this->declare_parameter<string>("sub_lidar_1", "/lidar/front/scan");
    this->declare_parameter<string>("sub_lidar_2", "/lidar/back/scan");
    this->declare_parameter<string>("pub_topic", "/lidar/merged_scan");
    this->declare_parameter<int>("output_samples", 2880);
    this->declare_parameter<int>("horizontal_virtual_frame",10);
    this->declare_parameter<int>("vertical_virtual_frame",20);
    this->declare_parameter<bool>("use_virtual_boundary", false);
    this->declare_parameter<double>("max_sync_delta", 0.04);
    this->declare_parameter<double>("self_filter_radius", 2.0);

    //take value from config
    sub_lidar_1 = this->get_parameter("sub_lidar_1").as_string();
    sub_lidar_2 = this->get_parameter("sub_lidar_2").as_string();
    pub_topic = this->get_parameter("pub_topic").as_string();
    output_samples_ = this->get_parameter("output_samples").as_int();
    hVirFrame = this->get_parameter("horizontal_virtual_frame").as_int();
    vVirFrame = this->get_parameter("vertical_virtual_frame").as_int();
    use_virtual_boundary_ =
        this->get_parameter("use_virtual_boundary").as_bool();
    max_sync_delta_ = this->get_parameter("max_sync_delta").as_double();
    self_filter_radius_ = this->get_parameter("self_filter_radius").as_double();

    //Create subscriber
    const auto sensor_qos = rclcpp::SensorDataQoS();
    subLidar1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    sub_lidar_1, sensor_qos, std::bind(&LaserScanMerger::lidar_1_callback, this, _1));
    subLidar2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    sub_lidar_2, sensor_qos, std::bind(&LaserScanMerger::lidar_2_callback, this, _1));
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
        pub_topic,
        sensor_qos);
    //Convert RPY to quaternion
    tf2::Quaternion front_rotation;
    front_rotation.setRPY(
    0.0,
    0.0,
    M_PI / 4.0);

    front_to_body_.setOrigin(
    tf2::Vector3(
        -0.735,
        0.885,
        0.04));

    front_to_body_.setRotation(front_rotation);

    tf2::Quaternion back_rotation;
    back_rotation.setRPY(
    0.0,
    0.0,
    M_PI / 4.0);

    back_to_body_.setOrigin(
    tf2::Vector3(
        0.735,
        -0.885,
        0.04));

    back_to_body_.setRotation(back_rotation);
    }
    private:
    int output_samples_{2880};
    std::string sub_lidar_1;
    std::string sub_lidar_2;
    std::string pub_topic;
    int hVirFrame;
    int vVirFrame;
    bool use_virtual_boundary_{false};
    double max_sync_delta_{0.04};
    double self_filter_radius_{2.0};

    sensor_msgs::msg::LaserScan::ConstSharedPtr front_scan_;
    sensor_msgs::msg::LaserScan::ConstSharedPtr back_scan_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subLidar1_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subLidar2_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
    tf2::Transform front_to_body_;
    tf2::Transform back_to_body_;

    void lidar_1_callback(
        const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
    {
        if (msg->ranges.empty()) {
        RCLCPP_WARN(get_logger(), "Front scan is empty");
        return;
        }
        RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Front: frame=%s, samples=%zu, angles=[%.2f, %.2f]",
        msg->header.frame_id.c_str(),
        msg->ranges.size(),
        msg->angle_min,
        msg->angle_max);
        front_scan_ = msg;
        tryMerge();
  }

  void lidar_2_callback(
        const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
    {
        if (msg->ranges.empty()) {
        RCLCPP_WARN(get_logger(), "Front scan is empty");
        return;
        }
        RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Front: frame=%s, samples=%zu, angles=[%.2f, %.2f]",
        msg->header.frame_id.c_str(),
        msg->ranges.size(),
        msg->angle_min,
        msg->angle_max);
        back_scan_ = msg;
        tryMerge();
    }

    void tryMerge()
    {
        if (!front_scan_ || !back_scan_) {
            return;
        }

        const rclcpp::Time front_stamp(front_scan_->header.stamp);
        const rclcpp::Time back_stamp(back_scan_->header.stamp);
        const double delta = (front_stamp - back_stamp).seconds();

        if (std::abs(delta) <= max_sync_delta_) {
            mergeAndPublish();
            front_scan_.reset();
            back_scan_.reset();
            return;
        }

        // Drop only the older scan and wait for a closer timestamp.
        if (delta < 0.0) {
            front_scan_.reset();
        } else {
            back_scan_.reset();
        }
    }

    void addScan(
    const sensor_msgs::msg::LaserScan & input,
    const tf2::Transform & sensor_to_body,
    sensor_msgs::msg::LaserScan & output)
    {
        const double half_x =
        static_cast<double>(hVirFrame) / 2.0;

        const double half_y =
        static_cast<double>(vVirFrame) / 2.0;
        for (std::size_t i = 0; i < input.ranges.size(); ++i)
        {
            const float range = input.ranges[i];
            if (!std::isfinite(range))
            {
            continue;
            }
            if (range < input.range_min ||range > input.range_max)
            {
            continue;
            }
            const double sensor_angle = input.angle_min + static_cast<double>(i) *input.angle_increment;
            const double sensor_x = static_cast<double>(range) * cos(sensor_angle);
            const double sensor_y = static_cast<double>(range) * sin(sensor_angle);
            const tf2::Vector3 point_in_sensor(sensor_x,
                                                sensor_y,
                                                0.0);
            tf2::Vector3 point_in_body = sensor_to_body * point_in_sensor;
            //Virtual boundary
            const double body_x = point_in_body.x();

            const double body_y = point_in_body.y();

            if (use_virtual_boundary_ &&
                (std::abs(body_x) > half_x || std::abs(body_y) > half_y))
            {
            continue;
            }

            const double body_range = hypot(
                                        body_x,
                                        body_y);

            // Remove returns from the USV hull, pontoons and sensor mounts.
            if (body_range < self_filter_radius_) {
                continue;
            }

            if (body_range < output.range_min) {
            continue;
            }
            double body_angle = atan2( point_in_body.y(), point_in_body.x());
            if (body_angle >= M_PI) {
                body_angle -= 2.0 * M_PI;
            }
            const int output_index = static_cast<int>(
                floor(
                (body_angle - output.angle_min) /
                output.angle_increment));

            if (output_index < 0 ||
                output_index >=
                static_cast<int>(output.ranges.size()))
            {
            continue;
            }

            if (body_range < output.range_min ||
                body_range > output.range_max)
            {
            continue;
            }

            const size_t index =
            static_cast<size_t>(output_index);

            output.ranges[index] =
            min(
                output.ranges[index],
                static_cast<float>(body_range));
        }
    }

    void mergeAndPublish()
    {
        if (!front_scan_ || !back_scan_) {
            return;
        }
        const double half_x =
        static_cast<double>(hVirFrame) / 2.0;

        const double half_y =
        static_cast<double>(vVirFrame) / 2.0;
        sensor_msgs::msg::LaserScan output;

        output.header.stamp =
            rclcpp::Time(front_scan_->header.stamp) >=
            rclcpp::Time(back_scan_->header.stamp) ?
            front_scan_->header.stamp : back_scan_->header.stamp;

        output.header.frame_id = "usv_body";

        output.angle_min = -static_cast<float>(M_PI);

        output.angle_increment =
            static_cast<float>(
            2.0 * M_PI /
            static_cast<double>(output_samples_));

        output.angle_max =
            output.angle_min +
            static_cast<float>(output_samples_ - 1) *
            output.angle_increment;

        output.time_increment = 0.0F;

        output.scan_time = std::max(
            front_scan_->scan_time,
            back_scan_->scan_time);

        output.range_min = std::min(
            front_scan_->range_min,
            back_scan_->range_min);
        output.range_max = std::max(
            front_scan_->range_max,
            back_scan_->range_max);

        output.ranges.assign(
            static_cast<std::size_t>(output_samples_),
            std::numeric_limits<float>::infinity());

        if (use_virtual_boundary_) {
            output.range_max = static_cast<float>(hypot(half_x, half_y));

            // Fill empty directions with a rectangular virtual boundary.
            for (int i = 0; i < output_samples_; i++) {
                const double angle =
                    output.angle_min +
                    static_cast<double>(i) * output.angle_increment;
                const double direction_x = cos(angle);
                const double direction_y = sin(angle);
                const double x_distance = abs(direction_x) > 1e-9 ?
                    abs(half_x / direction_x) :
                    numeric_limits<double>::infinity();
                const double y_distance = abs(direction_y) > 1e-9 ?
                    abs(half_y / direction_y) :
                    numeric_limits<double>::infinity();
                output.ranges[static_cast<std::size_t>(i)] =
                    static_cast<float>(min(x_distance, y_distance));
            }
        }
        addScan(
            *front_scan_,
            front_to_body_,
            output);

        addScan(
            *back_scan_,
            back_to_body_,
            output);

        pub_->publish(output);

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Merged scan: %zu samples, time delta=%.4f s",
            output.ranges.size(),
            std::abs(
                (rclcpp::Time(front_scan_->header.stamp) -
                 rclcpp::Time(back_scan_->header.stamp)).seconds()));
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserScanMerger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
