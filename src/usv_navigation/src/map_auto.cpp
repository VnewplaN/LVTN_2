//Automatically generate map by publish to usv controller
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "slam_toolbox/srv/save_map.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


using namespace std;
using namespace placeholders;
using namespace std::chrono_literals;

class MapAutoMapping : public rclcpp::Node{
    public:
    MapAutoMapping() : Node("map_auto_mapping"){
    //default value
    this->declare_parameter<string>("state_sub_topic", "/state_value");
    this->declare_parameter<string>("lidar_sub_topic", "/lidar/merged_scan");
    this->declare_parameter<string>("pub_topic", "/trajectory_cmd");
    this->declare_parameter<double>("lidar_front_angle", 1.5707963267948966);
    this->declare_parameter<double>("sector_half_width", 0.2617993877991494);
    this->declare_parameter<double>("desired_edge_distance", 3.0);
    this->declare_parameter<double>("edge_detect_distance", 8.0);
    this->declare_parameter<double>("safe_front_distance", 4.0);
    this->declare_parameter<double>("search_speed", 0.5);
    this->declare_parameter<double>("mapping_speed", 0.8);
    this->declare_parameter<double>("turn_speed", 0.15);
    this->declare_parameter<double>("distance_gain", 0.25);
    this->declare_parameter<double>("parallel_gain", 0.35);
    this->declare_parameter<double>("max_heading_correction", 0.5);
    this->declare_parameter<double>("turn_angle", 1.5707963267948966);
    this->declare_parameter<double>("minimum_loop_distance", 30.0);
    this->declare_parameter<double>("loop_close_radius", 2.0);
    this->declare_parameter<bool>("stop_when_loop_closed", true);
    this->declare_parameter<bool>("auto_save_map", true);
    this->declare_parameter<string>(
        "map_save_path",
        "/home/olive/LVTN_2/src/usv_navigation/maps/usv_map");
    this->declare_parameter<string>("world_frame", "world");
    this->declare_parameter<string>("map_frame", "map");
    this->declare_parameter<string>("odom_frame", "odom");
    this->declare_parameter<bool>("world_equals_odom", true);
    
    //take value from config
    state_topic_ = this->get_parameter("state_sub_topic").as_string();
    scan_topic_ = this->get_parameter("lidar_sub_topic").as_string();
    pub_topic_ = this->get_parameter("pub_topic").as_string();
    lidar_front_angle_ = this->get_parameter("lidar_front_angle").as_double();
    sector_half_width_ = this->get_parameter("sector_half_width").as_double();
    desired_edge_distance_ = this->get_parameter("desired_edge_distance").as_double();
    edge_detect_distance_ = this->get_parameter("edge_detect_distance").as_double();
    safe_front_distance_ = this->get_parameter("safe_front_distance").as_double();
    search_speed_ = this->get_parameter("search_speed").as_double();
    mapping_speed_ = this->get_parameter("mapping_speed").as_double();
    turn_speed_ = this->get_parameter("turn_speed").as_double();
    distance_gain_ = this->get_parameter("distance_gain").as_double();
    parallel_gain_ = this->get_parameter("parallel_gain").as_double();
    max_heading_correction_ = this->get_parameter("max_heading_correction").as_double();
    turn_angle_ = this->get_parameter("turn_angle").as_double();
    minimum_loop_distance_ = this->get_parameter("minimum_loop_distance").as_double();
    loop_close_radius_ = this->get_parameter("loop_close_radius").as_double();
    stop_when_loop_closed_ = this->get_parameter("stop_when_loop_closed").as_bool();
    auto_save_map_ = this->get_parameter("auto_save_map").as_bool();
    map_save_path_ = this->get_parameter("map_save_path").as_string();
    world_frame_ = this->get_parameter("world_frame").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    world_equals_odom_ = this->get_parameter("world_equals_odom").as_bool();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    //Create subscription
    state_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(state_topic_, 10, std::bind(
                                                                &MapAutoMapping::stateCallback,
                                                                this,
                                                                std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
                                                                scan_topic_, rclcpp::SensorDataQoS(), std::bind(
                                                                &MapAutoMapping::scanCallback,
                                                                this,
                                                                std::placeholders::_1));

   
    //Create publisher
    pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(pub_topic_, 10);
    save_map_client_ = create_client<slam_toolbox::srv::SaveMap>(
        "/slam_toolbox/save_map");
    control_timer_ = create_wall_timer(
        100ms, std::bind(&MapAutoMapping::controlLoop, this));
    }
    private:
    double x_ = 0.0;
    double y_ = 0.0;
    double psi_ = 0.0;

    double u_ = 0.0;
    double v_ = 0.0;
    double r_ = 0.0;

    double front_distance_ = std::numeric_limits<double>::infinity();
    double right_front_distance_ = std::numeric_limits<double>::infinity();
    double right_distance_ = std::numeric_limits<double>::infinity();
    double right_back_distance_ = std::numeric_limits<double>::infinity();
    double left_distance_ = std::numeric_limits<double>::infinity();

    double lidar_front_angle_ = 0.0;
    double sector_half_width_ = 0.0;
    double desired_edge_distance_ = 3.0;
    double edge_detect_distance_ = 8.0;
    double safe_front_distance_ = 4.0;
    double search_speed_ = 0.5;
    double mapping_speed_ = 0.8;
    double turn_speed_ = 0.15;
    double distance_gain_ = 0.25;
    double parallel_gain_ = 0.35;
    double max_heading_correction_ = 0.5;
    double turn_angle_ = 1.5707963267948966;
    double minimum_loop_distance_ = 30.0;
    double loop_close_radius_ = 2.0;
    bool stop_when_loop_closed_ = true;
    bool auto_save_map_ = true;
    bool save_requested_ = false;
    bool state_received_ = false;
    bool scan_received_ = false;

    enum class Mode {SEARCH_EDGE, FOLLOW_EDGE, AVOID_FRONT, FINISHED};
    Mode mode_ = Mode::SEARCH_EDGE;
    bool search_heading_initialized_ = false;
    bool edge_seen_once_ = false;
    bool loop_start_initialized_ = false;
    double search_heading_ = 0.0;
    double avoid_heading_ = 0.0;
    double loop_start_x_ = 0.0;
    double loop_start_y_ = 0.0;
    double last_path_x_ = 0.0;
    double last_path_y_ = 0.0;
    double travelled_distance_ = 0.0;

    string state_topic_;
    string scan_topic_;
    string pub_topic_;
    string map_save_path_;
    string world_frame_;
    string map_frame_;
    string odom_frame_;
    bool world_equals_odom_ = true;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::Client<slam_toolbox::srv::SaveMap>::SharedPtr save_map_client_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool saveTransformMetadata(
        const geometry_msgs::msg::TransformStamped & transform) const
    {
        const std::filesystem::path output_path(map_save_path_ + ".transform.yaml");
        const std::filesystem::path temporary_path(
            map_save_path_ + ".transform.yaml.tmp");

        std::ofstream output(temporary_path, std::ios::trunc);
        if (!output) {
            RCLCPP_ERROR(
                get_logger(), "Cannot open transform metadata: %s",
                temporary_path.c_str());
            return false;
        }

        const auto & t = transform.transform.translation;
        const auto & q = transform.transform.rotation;
        output << std::setprecision(17)
               << "parent_frame: " << transform.header.frame_id << '\n'
               << "child_frame: " << transform.child_frame_id << '\n'
               << "translation: [" << t.x << ", " << t.y << ", " << t.z << "]\n"
               << "rotation: [" << q.x << ", " << q.y << ", "
               << q.z << ", " << q.w << "]\n";
        output.close();

        if (!output) {
            RCLCPP_ERROR(get_logger(), "Failed writing %s", temporary_path.c_str());
            return false;
        }

        std::error_code error;
        std::filesystem::rename(temporary_path, output_path, error);
        if (error) {
            // std::filesystem::rename cannot replace an existing file on every platform.
            std::filesystem::remove(output_path, error);
            error.clear();
            std::filesystem::rename(temporary_path, output_path, error);
        }
        if (error) {
            RCLCPP_ERROR(
                get_logger(), "Cannot install transform metadata %s: %s",
                output_path.c_str(), error.message().c_str());
            return false;
        }
        return true;
    }

    bool snapshotWorldToMap(
        geometry_msgs::msg::TransformStamped & world_to_map)
    {
        try {
            world_to_map = tf_buffer_->lookupTransform(
                world_frame_, map_frame_, tf2::TimePointZero);
            makePlanar(world_to_map);
            return true;
        } catch (const tf2::TransformException & direct_exception) {
            if (!world_equals_odom_) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "TF %s -> %s unavailable: %s",
                    world_frame_.c_str(), map_frame_.c_str(), direct_exception.what());
                return false;
            }

            try {
                // Gazebo odometry uses world coordinates in this project.  If
                // no explicit world frame exists, T_world_map is therefore the
                // inverse of the SLAM transform T_map_odom.
                const auto map_to_odom = tf_buffer_->lookupTransform(
                    map_frame_, odom_frame_, tf2::TimePointZero);
                tf2::Transform transform;
                tf2::fromMsg(map_to_odom.transform, transform);
                world_to_map.header = map_to_odom.header;
                world_to_map.header.frame_id = world_frame_;
                world_to_map.child_frame_id = map_frame_;
                world_to_map.transform = tf2::toMsg(transform.inverse());
                makePlanar(world_to_map);
                RCLCPP_WARN(
                    get_logger(),
                    "No direct %s -> %s TF; using %s == %s to compute it",
                    world_frame_.c_str(), map_frame_.c_str(),
                    world_frame_.c_str(), odom_frame_.c_str());
                return true;
            } catch (const tf2::TransformException & fallback_exception) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Cannot snapshot map transform: direct TF failed (%s); "
                    "fallback TF %s -> %s failed (%s)",
                    direct_exception.what(), map_frame_.c_str(), odom_frame_.c_str(),
                    fallback_exception.what());
                return false;
            }
        }
    }

    static void makePlanar(
        geometry_msgs::msg::TransformStamped & transform)
    {
        // OccupancyGrid is a 2-D map. Boat roll, pitch and heave must not leak
        // into the persistent world/map calibration.
        const double yaw = tf2::getYaw(transform.transform.rotation);
        tf2::Quaternion rotation;
        rotation.setRPY(0.0, 0.0, yaw);
        transform.transform.translation.z = 0.0;
        transform.transform.rotation = tf2::toMsg(rotation);
    }

    void stateCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
        if (msg->data.size() < 6) {
            RCLCPP_WARN(get_logger(), "state_value must contain 6 values");
            return;
        }

        x_ = msg->data[0];
        y_ = msg->data[1];
        psi_ = msg->data[2];
        u_ = msg->data[3];
        v_ = msg->data[4];
        r_ = msg->data[5];

        if (loop_start_initialized_) {
            travelled_distance_ += std::hypot(
                x_ - last_path_x_, y_ - last_path_y_);
            last_path_x_ = x_;
            last_path_y_ = y_;
        }

        if (!search_heading_initialized_) {
            search_heading_ = psi_;
            search_heading_initialized_ = true;
        }
        state_received_ = true;
    }

    double minimumRangeAround(
        const sensor_msgs::msg::LaserScan & scan,
        const double center_angle) const
    {
        double minimum_range = std::numeric_limits<double>::infinity();

        for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
            const double range = scan.ranges[i];

            if (!std::isfinite(range) ||
                range < scan.range_min ||
                range > scan.range_max)
            {
                continue;
            }

            const double angle =
                scan.angle_min + static_cast<double>(i) * scan.angle_increment;

            const double angle_error = std::atan2(
                std::sin(angle - center_angle),
                std::cos(angle - center_angle));

            if (std::abs(angle_error) <= sector_half_width_) {
                minimum_range = std::min(minimum_range, range);
            }
        }

        return minimum_range;
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        constexpr double kHalfPi = 1.5707963267948966;
        constexpr double kQuarterPi = 0.7853981633974483;

        front_distance_ = minimumRangeAround(*msg, lidar_front_angle_);
        right_front_distance_ = minimumRangeAround(
            *msg, lidar_front_angle_ - kQuarterPi);
        right_distance_ = minimumRangeAround(
            *msg, lidar_front_angle_ - kHalfPi);
        right_back_distance_ = minimumRangeAround(
            *msg, lidar_front_angle_ - kHalfPi - kQuarterPi);
        left_distance_ = minimumRangeAround(
            *msg, lidar_front_angle_ + kHalfPi);

        scan_received_ = true;

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "front=%.2f | right-front=%.2f | right=%.2f | right-back=%.2f | left=%.2f",
            front_distance_,
            right_front_distance_,
            right_distance_,
            right_back_distance_,
            left_distance_);
    }

    double normalizeAngle(const double angle) const {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    bool validEdge(const double range) const {
        return std::isfinite(range) && range <= edge_detect_distance_;
    }

    void publishCommand(
        const double desired_u,
        const double desired_v,
        const double desired_psi)
    {
        std_msgs::msg::Float64MultiArray command;
        command.data = {
            desired_u,
            desired_v,
            normalizeAngle(desired_psi)
        };
        pub_->publish(command);
    }

    void enterAvoidMode() {
        if (mode_ != Mode::AVOID_FRONT) {
            // Following the right edge means a blocked front is avoided to the left.
            avoid_heading_ = normalizeAngle(psi_ + turn_angle_);
            mode_ = Mode::AVOID_FRONT;
            RCLCPP_INFO(get_logger(), "Front blocked: turning left");
        }
    }

    void enterFollowMode() {
        mode_ = Mode::FOLLOW_EDGE;
        edge_seen_once_ = true;

        if (!loop_start_initialized_) {
            loop_start_initialized_ = true;
            loop_start_x_ = x_;
            loop_start_y_ = y_;
            last_path_x_ = x_;
            last_path_y_ = y_;
            travelled_distance_ = 0.0;
        }

        RCLCPP_INFO(get_logger(), "Right edge detected: following edge");
    }

    bool loopClosed() const {
        if (!stop_when_loop_closed_ || !loop_start_initialized_) {
            return false;
        }

        const double distance_to_start = std::hypot(
            x_ - loop_start_x_, y_ - loop_start_y_);

        return travelled_distance_ >= minimum_loop_distance_ &&
               distance_to_start <= loop_close_radius_;
    }

    void saveMap() {
        if (!auto_save_map_ || save_requested_) {
            return;
        }

        if (!save_map_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for /slam_toolbox/save_map service");
            return;
        }

        geometry_msgs::msg::TransformStamped world_to_map;
        // Snapshot first so the metadata and occupancy map belong to the same
        // mapping session, even if SLAM changes its correction afterwards.
        if (!snapshotWorldToMap(world_to_map)) {
            return;
        }

        auto request = std::make_shared<slam_toolbox::srv::SaveMap::Request>();
        request->name.data = map_save_path_;
        save_requested_ = true;

        save_map_client_->async_send_request(
            request,
            [this, world_to_map](
                rclcpp::Client<slam_toolbox::srv::SaveMap>::SharedFuture future) {
                const auto response = future.get();
                if (response->result ==
                    slam_toolbox::srv::SaveMap::Response::RESULT_SUCCESS)
                {
                    if (!saveTransformMetadata(world_to_map)) {
                        RCLCPP_ERROR(
                            get_logger(),
                            "Map image was saved, but its world transform was not");
                        save_requested_ = false;
                        return;
                    }
                    RCLCPP_INFO(
                        get_logger(),
                        "Map saved with world transform: %s.{pgm,yaml,transform.yaml}",
                        map_save_path_.c_str());
                } else {
                    RCLCPP_ERROR(
                        get_logger(),
                        "SLAM Toolbox failed to save map, result=%u",
                        static_cast<unsigned int>(response->result));
                    save_requested_ = false;
                }
            });
    }

    void controlLoop() {
        if (!state_received_ || !scan_received_) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for /state_value and lidar scan");
            return;
        }

        if (mode_ == Mode::FINISHED) {
            publishCommand(0.0, 0.0, psi_);
            saveMap();
            return;
        }

        if (loopClosed()) {
            mode_ = Mode::FINISHED;
            publishCommand(0.0, 0.0, psi_);
            RCLCPP_INFO(
                get_logger(),
                "Mapping loop completed after %.1f m",
                travelled_distance_);
            saveMap();
            return;
        }

        if (front_distance_ <= safe_front_distance_) {
            enterAvoidMode();
        }

        if (mode_ == Mode::AVOID_FRONT) {
            const double heading_error = normalizeAngle(avoid_heading_ - psi_);

            if (std::abs(heading_error) < 0.15 &&
                front_distance_ > safe_front_distance_)
            {
                if (validEdge(right_distance_)) {
                    enterFollowMode();
                } else {
                    mode_ = Mode::SEARCH_EDGE;
                    search_heading_ = psi_;
                }
            } else {
                publishCommand(turn_speed_, 0.0, avoid_heading_);
            }
            return;
        }

        if (mode_ == Mode::SEARCH_EDGE) {
            if (validEdge(right_distance_)) {
                enterFollowMode();
            } else {
                // Initially go straight. After losing an edge, curve right to find it again.
                const double target_heading = edge_seen_once_
                    ? normalizeAngle(psi_ - 0.25)
                    : search_heading_;
                publishCommand(search_speed_, 0.0, target_heading);
                return;
            }
        }

        if (mode_ == Mode::FOLLOW_EDGE) {
            if (!validEdge(right_distance_)) {
                mode_ = Mode::SEARCH_EDGE;
                publishCommand(search_speed_, 0.0, normalizeAngle(psi_ - 0.25));
                return;
            }

            const double distance_error =
                right_distance_ - desired_edge_distance_;

            double parallel_error = 0.0;
            if (std::isfinite(right_front_distance_) &&
                std::isfinite(right_back_distance_))
            {
                parallel_error =
                    right_front_distance_ - right_back_distance_;
            }

            const double correction = std::clamp(
                -distance_gain_ * distance_error -
                parallel_gain_ * parallel_error,
                -max_heading_correction_,
                max_heading_correction_);

            publishCommand(
                mapping_speed_, 0.0,
                normalizeAngle(psi_ + correction));
        }
    }
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapAutoMapping>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
