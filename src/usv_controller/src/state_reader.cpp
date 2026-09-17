//Receive data from /odom
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "chrono"
#include <cmath>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
using namespace std;
using namespace placeholders;
class StateReader : public rclcpp::Node{
  public: 
  StateReader() : Node("state_reader")
  {
    //default value
    this->declare_parameter<string>("sub_topic", "/odom");
    this->declare_parameter<string>("pub_topic", "/state_value");
    this->declare_parameter<double>("body_yaw_offset", M_PI / 2.0);
    //take value from config
    sub_topic_ = this->get_parameter("sub_topic").as_string();
    pub_topic_ = this->get_parameter("pub_topic").as_string();
    body_yaw_offset_ = this->get_parameter("body_yaw_offset").as_double();
    RCLCPP_INFO(this->get_logger(), "Sub topic is: %s", sub_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Pub topic is: %s", pub_topic_.c_str());
    //Create subscription
    sub = this->create_subscription<nav_msgs::msg::Odometry>(sub_topic_, 10, 
                                            bind(&StateReader::subCallback, this, _1));
    //Create publisher
    pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(pub_topic_, 10);                            
  }
  private: 
  rclcpp::Subscription<nav_msgs::msg::Odometry>:: SharedPtr sub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub;
  string sub_topic_;
  string pub_topic_;
  double body_yaw_offset_;
  void subCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    //Update position state
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    const double link_yaw = tf2::getYaw(msg->pose.pose.orientation);
    const double psi = atan2(
        sin(link_yaw + body_yaw_offset_),
        cos(link_yaw + body_yaw_offset_));

    // Odometry twist is in usv_body. Rotate it into the controller frame,
    // whose +x axis points toward the bow.
    const double vx_link = msg->twist.twist.linear.x;
    const double vy_link = msg->twist.twist.linear.y;
    const double c = cos(body_yaw_offset_);
    const double s = sin(body_yaw_offset_);
    double u = c * vx_link + s * vy_link;
    double v = -s * vx_link + c * vy_link;
    double r = msg->twist.twist.angular.z;

    RCLCPP_INFO(
        this->get_logger(),
        "x: %.2f | y: %.2f | psi: %.2f | u: %.2f | v: %.2f | r: %.2f",
        x, y, psi, u, v, r
    );
    //Publish data to pub topic
    std_msgs::msg::Float64MultiArray state_msg;

    state_msg.data = {
        x,
        y,
        psi,
        u,
        v,
        r
    };

    pub->publish(state_msg);
  }
};
int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StateReader>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
