#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class TrajectoryNode : public rclcpp::Node
{
public:
  TrajectoryNode()
  : Node("trajectory_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    path_topic_ =
      declare_parameter<std::string>(
        "path_topic",
        "/coverage_path");

    state_topic_ =
      declare_parameter<std::string>(
        "state_topic",
        "/state_value");

    cmd_topic_ =
      declare_parameter<std::string>(
        "cmd_topic",
        "/trajectory_cmd");

    control_period_ =
      declare_parameter<double>(
        "control_period",
        0.05);

    U_follow_ =
      declare_parameter<double>(
        "speed",
        0.30);

    U_go_start_ =
      declare_parameter<double>(
        "go_start_speed",
        U_follow_);

    U_shift_ =
      declare_parameter<double>(
        "shift_speed",
        U_follow_);

    Delta_ =
      declare_parameter<double>(
        "alos_lookahead",
        3.0);

    gamma_ =
      declare_parameter<double>(
        "alos_gamma",
        0.015);

    beta_max_ =
      deg2rad(
        declare_parameter<double>(
          "beta_max_deg",
          35.0));

    start_acceptance_ =
      declare_parameter<double>(
        "start_acceptance",
        0.30);

    lane_acceptance_ =
      declare_parameter<double>(
        "lane_acceptance",
        0.30);

    shift_acceptance_ =
      declare_parameter<double>(
        "shift_acceptance",
        0.08);

    heading_acceptance_ =
      deg2rad(
        declare_parameter<double>(
          "heading_acceptance_deg",
          5.0));

    cmd_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
        cmd_topic_,
        10);

    path_sub_ =
      create_subscription<nav_msgs::msg::Path>(
        path_topic_,
        rclcpp::QoS(1).transient_local().reliable(),
        std::bind(
          &TrajectoryNode::pathCallback,
          this,
          std::placeholders::_1));

    state_sub_ =
      create_subscription<std_msgs::msg::Float64MultiArray>(
        state_topic_,
        10,
        std::bind(
          &TrajectoryNode::stateCallback,
          this,
          std::placeholders::_1));

    timer_ =
      create_wall_timer(
        std::chrono::duration<double>(
          control_period_),
        std::bind(
          &TrajectoryNode::controlLoop,
          this));

    RCLCPP_INFO(
      get_logger(),
      "Trajectory node started");
  }

private:
  enum class Mode
  {
    WAIT_PATH,
    GO_TO_START,
    ALIGN,
    FOLLOW_LANE,
    SHIFT_LANE,
    FINISHED
  };

  struct Point
  {
    double x;
    double y;
  };

  struct Lane
  {
    Point start;
    Point end;
  };

  static double deg2rad(double d)
  {
    return d * M_PI / 180.0;
  }

  static double rad2deg(double r)
  {
    return r * 180.0 / M_PI;
  }

  static double wrapPi(double a)
  {
    while (a > M_PI) {
      a -= 2.0 * M_PI;
    }

    while (a < -M_PI) {
      a += 2.0 * M_PI;
    }

    return a;
  }

  static double distance(
    double x1,
    double y1,
    double x2,
    double y2)
  {
    return std::hypot(
      x2 - x1,
      y2 - y1);
  }

  void stateCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 6) {
      return;
    }

    x_ = msg->data[0];
    y_ = msg->data[1];
    psi_ = msg->data[2];

    have_state_ = true;
  }

  void pathCallback(
    const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.size() < 2) {
      return;
    }

    /*
     * Extract lanes from the dense coverage path.
     *
     * Coverage planner generates vertical lanes:
     * points inside one lane have approximately
     * constant X.
     */
    tf2::Transform path_to_odom;
    const std::string path_frame = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
    if (path_frame == "odom") {
      path_to_odom.setIdentity();
    } else {
      try {
        const auto tf = tf_buffer_.lookupTransform("odom", path_frame, tf2::TimePointZero);
        path_to_odom.setOrigin(tf2::Vector3(
          tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z));
        path_to_odom.setRotation(tf2::Quaternion(
          tf.transform.rotation.x, tf.transform.rotation.y,
          tf.transform.rotation.z, tf.transform.rotation.w));
      } catch (const tf2::TransformException & error) {
        RCLCPP_ERROR(
          get_logger(), "Cannot transform coverage path %s -> odom: %s",
          path_frame.c_str(), error.what());
        return;
      }
    }

    const auto pointInOdom = [&path_to_odom](const geometry_msgs::msg::Point & p) {
      const tf2::Vector3 q = path_to_odom * tf2::Vector3(p.x, p.y, p.z);
      return Point{q.x(), q.y()};
    };

    std::vector<Lane> extracted;

    std::size_t begin = 0;

    constexpr double X_TOL = 0.05;

    while (begin < msg->poses.size())
    {
      const double lane_x =
        msg->poses[begin].pose.position.x;

      std::size_t end = begin;

      while (
        end + 1 < msg->poses.size() &&
        std::abs(
          msg->poses[end + 1].pose.position.x -
          lane_x) < X_TOL)
      {
        ++end;
      }

      if (end > begin)
      {
        Lane lane;

        lane.start = pointInOdom(msg->poses[begin].pose.position);
        lane.end = pointInOdom(msg->poses[end].pose.position);

        if (
          distance(
            lane.start.x,
            lane.start.y,
            lane.end.x,
            lane.end.y) > 1.0)
        {
          extracted.push_back(lane);
        }
      }

      begin = end + 1;
    }

    if (extracted.empty())
    {
      RCLCPP_ERROR(
        get_logger(),
        "Could not extract lanes from /coverage_path");
      return;
    }

    lanes_ = extracted;

    lane_index_ = 0;

    /*
     * Hull reference heading is the direction
     * of the FIRST coverage lane.
     */
    const auto & first = lanes_.front();

    psi_ref_ =
      std::atan2(
        first.end.y - first.start.y,
        first.end.x - first.start.x);

    beta_hat_ = 0.0;

    have_path_ = true;

    mode_ = Mode::GO_TO_START;

    RCLCPP_INFO(
      get_logger(),
      "Coverage path received");

    RCLCPP_INFO(
      get_logger(),
      "Extracted lanes: %zu",
      lanes_.size());

    RCLCPP_INFO(
      get_logger(),
      "Coverage start: (%.2f, %.2f)",
      first.start.x,
      first.start.y);

    RCLCPP_INFO(
      get_logger(),
      "Fixed hull heading: %.1f deg",
      rad2deg(psi_ref_));
  }

  void publishCommand(
    double u_d,
    double v_d,
    double psi_d)
  {
    std_msgs::msg::Float64MultiArray cmd;

    cmd.data = {
      u_d,
      v_d,
      wrapPi(psi_d)
    };

    cmd_pub_->publish(cmd);
  }

  void stop()
  {
    publishCommand(
      0.0,
      0.0,
      psi_);
  }

  /*
   * Convert desired WORLD velocity direction
   * into body-frame surge/sway command.
   */
  void velocityCommand(
    double U,
    double chi_d,
    double psi_d)
  {
    const double relative =
      wrapPi(chi_d - psi_);

    const double u_d =
      U * std::cos(relative);

    const double v_d =
      U * std::sin(relative);

    publishCommand(
      u_d,
      v_d,
      psi_d);
  }

  void goToStart()
  {
    const auto & target =
      lanes_.front().start;

    const double dx =
      target.x - x_;

    const double dy =
      target.y - y_;

    const double d =
      std::hypot(dx, dy);

    if (d < start_acceptance_)
    {
      publishCommand(
        0.0,
        0.0,
        psi_ref_);

      mode_ = Mode::ALIGN;

      RCLCPP_INFO(
        get_logger(),
        "Reached coverage start -> ALIGN");

      return;
    }

    /*
     * While approaching the coverage start,
     * bow follows direction of travel.
     */
    const double chi =
      std::atan2(dy, dx);

    velocityCommand(
      U_go_start_,
      chi,
      chi);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "GO_TO_START | distance=%.2f | chi=%.1f deg",
      d,
      rad2deg(chi));
  }

  void align()
  {
    const double e =
      wrapPi(
        psi_ref_ - psi_);

    publishCommand(
      0.0,
      0.0,
      psi_ref_);

    if (
      std::abs(e) <
      heading_acceptance_)
    {
      beta_hat_ = 0.0;

      mode_ =
        Mode::FOLLOW_LANE;

      RCLCPP_INFO(
        get_logger(),
        "Heading aligned -> FOLLOW_LANE");
    }
  }

  void followLane()
  {
    if (lane_index_ >= lanes_.size())
    {
      mode_ = Mode::FINISHED;
      return;
    }

    const Lane & lane =
      lanes_[lane_index_];

    const double dx =
      lane.end.x -
      lane.start.x;

    const double dy =
      lane.end.y -
      lane.start.y;

    const double L =
      std::hypot(dx, dy);

    if (L < 1e-6)
    {
      ++lane_index_;
      return;
    }

    const double tx = dx / L;
    const double ty = dy / L;

    /*
     * Left-hand normal of lane.
     */
    const double nx = -ty;
    const double ny =  tx;

    const double rx =
      x_ - lane.start.x;

    const double ry =
      y_ - lane.start.y;

    /*
     * Along-track position.
     */
    const double s =
      rx * tx +
      ry * ty;

    /*
     * Signed cross-track error.
     */
    const double ey =
      rx * nx +
      ry * ny;

    const double distance_to_end =
      distance(
        x_,
        y_,
        lane.end.x,
        lane.end.y);

    /*
     * End condition.
     *
     * Either close enough to endpoint or already
     * projected to the end of lane.
     */
    if (
      distance_to_end <
        lane_acceptance_ ||
      s >= L)
    {
      beta_hat_ = 0.0;

      if (
        lane_index_ + 1 >=
        lanes_.size())
      {
        mode_ = Mode::FINISHED;

        publishCommand(
          0.0,
          0.0,
          psi_ref_);

        RCLCPP_INFO(
          get_logger(),
          "Coverage completed");

        return;
      }

      mode_ =
        Mode::SHIFT_LANE;

      RCLCPP_INFO(
        get_logger(),
        "Lane %zu/%zu completed -> SHIFT",
        lane_index_ + 1,
        lanes_.size());

      return;
    }

    /*
     * ALOS adaptive sideslip estimate.
     *
     * Keep adaptation small and bounded.
     */
    beta_hat_ +=
      gamma_ *
      ey *
      control_period_;

    beta_hat_ =
      std::clamp(
        beta_hat_,
        -beta_max_,
        beta_max_);

    const double chi_path =
      std::atan2(
        dy,
        dx);

    /*
     * ALOS:
     *
     * chi_d =
     * chi_path
     * - atan(ey / Delta)
     * - beta_hat
     */
    const double chi_d =
      wrapPi(
        chi_path -
        std::atan2(
          ey,
          Delta_) -
        beta_hat_);

    /*
     * IMPORTANT:
     *
     * psi_ref_ remains fixed.
     *
     * For reverse lanes, chi_path differs from
     * psi_ref by approximately pi, therefore
     * velocityCommand() naturally produces
     * negative surge instead of rotating the hull.
     */
    velocityCommand(
      U_follow_,
      chi_d,
      psi_ref_);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "LANE %zu/%zu | ey=%+.3f | beta=%+.1f deg | "
      "chi=%+.1f deg | psi=%+.1f/%.1f deg",
      lane_index_ + 1,
      lanes_.size(),
      ey,
      rad2deg(beta_hat_),
      rad2deg(chi_d),
      rad2deg(psi_),
      rad2deg(psi_ref_));
  }

  void shiftLane()
  {
    if (
      lane_index_ + 1 >=
      lanes_.size())
    {
      mode_ = Mode::FINISHED;
      return;
    }

    const auto & target =
      lanes_[lane_index_ + 1].start;

    const double dx =
      target.x - x_;

    const double dy =
      target.y - y_;

    const double d =
      std::hypot(dx, dy);

    if (d < shift_acceptance_)
    {
      ++lane_index_;

      beta_hat_ = 0.0;

      mode_ =
        Mode::FOLLOW_LANE;

      RCLCPP_INFO(
        get_logger(),
        "Shift completed -> lane %zu",
        lane_index_ + 1);

      return;
    }

    /*
     * Move toward next lane start while maintaining
     * fixed hull heading.
     *
     * For the normal lawnmower path this is almost
     * pure sway.
     */
    const double chi_shift =
      std::atan2(
        dy,
        dx);

    velocityCommand(
      U_shift_,
      chi_shift,
      psi_ref_);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "SHIFT | next lane=%zu | distance=%.3f",
      lane_index_ + 2,
      d);
  }

  void controlLoop()
  {
    if (!have_state_) {
      return;
    }

    if (!have_path_)
    {
      stop();
      return;
    }

    switch (mode_)
    {
      case Mode::WAIT_PATH:
        stop();
        break;

      case Mode::GO_TO_START:
        goToStart();
        break;

      case Mode::ALIGN:
        align();
        break;

      case Mode::FOLLOW_LANE:
        followLane();
        break;

      case Mode::SHIFT_LANE:
        shiftLane();
        break;

      case Mode::FINISHED:
        publishCommand(
          0.0,
          0.0,
          psi_ref_);
        break;
    }
  }

  std::string path_topic_;
  std::string state_topic_;
  std::string cmd_topic_;

  double control_period_;

  double U_follow_;
  double U_go_start_;
  double U_shift_;

  double Delta_;
  double gamma_;
  double beta_max_;

  double start_acceptance_;
  double lane_acceptance_;
  double shift_acceptance_;
  double heading_acceptance_;

  double x_{0.0};
  double y_{0.0};
  double psi_{0.0};

  double psi_ref_{0.0};
  double beta_hat_{0.0};

  bool have_state_{false};
  bool have_path_{false};

  std::size_t lane_index_{0};

  Mode mode_{Mode::WAIT_PATH};

  std::vector<Lane> lanes_;

  rclcpp::Subscription<
    nav_msgs::msg::Path>::SharedPtr path_sub_;

  rclcpp::Subscription<
    std_msgs::msg::Float64MultiArray>::SharedPtr state_sub_;

  rclcpp::Publisher<
    std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<TrajectoryNode>());

  rclcpp::shutdown();

  return 0;
}
