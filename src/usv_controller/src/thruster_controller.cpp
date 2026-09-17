#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "ros_gz_interfaces/msg/entity_wrench.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
using namespace std;
using namespace placeholders;
class ThrusterController : public rclcpp::Node
{
    public:
    ThrusterController() : Node("thruster_controller"){ 
    //default value
    this->declare_parameter<string>("state_sub_topic", "/state_value");
    this->declare_parameter<string>(
        "pub_topic", "/world/industrial_sludge_pond/wrench");
    this->declare_parameter<string>("traj_sub_topic", "/trajectory_cmd");
    this->declare_parameter<string>("link_name", "usv::usv_body");
    this->declare_parameter<int>("freq", 20);
    this->declare_parameter<bool>("keyboard_control_flag", false);
    this->declare_parameter<double>("wrench_step_scale", 50.0);
    //take value from config
    state_sub_topic_ = this->get_parameter("state_sub_topic").as_string();
    traj_sub_topic_ = this->get_parameter("traj_sub_topic").as_string();
    pub_topic_ = this->get_parameter("pub_topic").as_string();
    link_name_ = this->get_parameter("link_name").as_string();
    freq_ = std::max(1, static_cast<int>(this->get_parameter("freq").as_int()));
    keyboard_control_flag_ = this->get_parameter("keyboard_control_flag").as_bool();
    wrench_step_scale_ = this->get_parameter("wrench_step_scale").as_double();
    //create subscription
    state_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(state_sub_topic_, 10, 
                                            bind(&ThrusterController::StateCallback, this, _1));
    traj_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(traj_sub_topic_, 10, 
                                            bind(&ThrusterController::TrajCallback, this, _1));
    pub = this->create_publisher<ros_gz_interfaces::msg::EntityWrench>(pub_topic_, 10);

    if (keyboard_control_flag_)
    {
    // Save current terminal configuration
    tcgetattr(STDIN_FILENO, &old_terminal_);

    // Copy old configuration
    new_terminal_ = old_terminal_;

    // Disable Enter requirement and echo
    new_terminal_.c_lflag &= ~(ICANON | ECHO);

    // Apply new configuration
    tcsetattr(
        STDIN_FILENO,
        TCSANOW,
        &new_terminal_);

    // Get current input flags
    old_flags_ =
        fcntl(STDIN_FILENO, F_GETFL, 0);

    // Set keyboard input to non-blocking
    fcntl(
        STDIN_FILENO,
        F_SETFL,
        old_flags_ | O_NONBLOCK);
    }

    //Timer for keyboard control
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / freq_),
                                    std::bind(
                                    &ThrusterController::ControlLoop,
                                    this));
    
    }

    ~ThrusterController()
    {
        if (keyboard_control_flag_)
        {
            // Restore terminal configuration
            tcsetattr(
                STDIN_FILENO,
                TCSANOW,
                &old_terminal_);

            // Restore input flags
            fcntl(
                STDIN_FILENO,
                F_SETFL,
                old_flags_);
        }
    }

    private: 
    //Declaration
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr traj_sub_;
    rclcpp::Publisher<ros_gz_interfaces::msg::EntityWrench>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer_;
    string state_sub_topic_;
    string traj_sub_topic_;
    string pub_topic_;
    string link_name_;

    int freq_;
    bool keyboard_control_flag_;
    double wrench_step_scale_;
    //Terminal
    struct termios old_terminal_;
    struct termios new_terminal_;

    int old_flags_;

    double X_keyboard_ = 0.0;
    double Y_keyboard_ = 0.0;
    double N_keyboard_ = 0.0;

    std::chrono::steady_clock::time_point last_x_key_time_;
    std::chrono::steady_clock::time_point last_y_key_time_;
    std::chrono::steady_clock::time_point last_n_key_time_;

    bool x_key_active_ = false;
    bool y_key_active_ = false;
    bool n_key_active_ = false;

    double key_timeout_ = 0.20;
    // Actual state
    double x_ = 0.0;
    double y_ = 0.0;
    double psi_ = 0.0;

    double u_ = 0.0;
    double v_ = 0.0;
    double r_ = 0.0;

    // Desired state
    double u_d_ = 0.0;
    double v_d_ = 0.0;
    double psi_d_ = 0.0;
    bool state_received_ = false;
    bool trajectory_received_ = false;

    // Controller memory (must persist between timer callbacks)
    double integral_error_u_ = 0.0;
    double integral_error_v_ = 0.0;
    double integral_error_psi_ = 0.0;
    double previous_error_u_ = 0.0;
    double previous_error_v_ = 0.0;
    double previous_error_psi_ = 0.0;
    bool error_history_initialized_ = false;
    //Functions
    void StateCallback(
        const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 6)
        {
            return;
        }
        x_   = msg->data[0];
        y_   = msg->data[1];
        psi_ = msg->data[2];

        u_ = msg->data[3];
        v_ = msg->data[4];
        r_ = msg->data[5];
        state_received_ = true;
    }
    void TrajCallback(
        const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {   
        if(keyboard_control_flag_ == 1){
            return;
        }
        if (msg->data.empty())
        {
            RCLCPP_WARN(this->get_logger(),
                "Trajectory command must contain at least u_d");
            return;
        }

        // Supported command forms:
        // [u_d]                 -> surge, zero sway, hold current heading
        // [u_d, v_d]            -> surge/sway, hold current heading
        // [u_d, v_d, psi_d]     -> surge/sway and explicit heading
        u_d_ = msg->data[0];
        v_d_ = msg->data.size() >= 2 ? msg->data[1] : 0.0;
        psi_d_ = msg->data.size() >= 3 ? msg->data[2] : psi_;
        // Do not reset PID memory here. trajectory_node publishes continuously
        // (20 Hz); resetting on every message prevents the integral terms from
        // accumulating and leaves a permanent velocity error.
        trajectory_received_ = true;
    }

    void KeyboardControl()
    {
        char key;

        int result = read(
            STDIN_FILENO,
            &key,
            1);
        auto current_time = std::chrono::steady_clock::now();
        if (result > 0)
        {   
            switch (key)
            {
                // Forward
                case 'w':
                case 'W':
                    X_keyboard_ = 1000.0;
                    x_key_active_ = true;
                    last_x_key_time_ = current_time;
                    break;

                // Backward
                case 's':
                case 'S':
                    X_keyboard_ = -1000.0;
                    x_key_active_ = true;
                    last_x_key_time_ = current_time;
                    break;

                // Left
                case 'a':
                case 'A':
                    Y_keyboard_ = 1000.0;
                    y_key_active_ = true;
                    last_y_key_time_ = current_time;                   
                    break;

                // Right
                case 'd':
                case 'D':
                    Y_keyboard_ = -1000.0;
                    y_key_active_ = true;
                    last_y_key_time_ = current_time;      
                    break;

                // Counter-clockwise
                case 'q':
                case 'Q':
                    N_keyboard_ = 500.0;
                    n_key_active_ = true;
                    last_n_key_time_ = current_time;    
                    break;

                // Clockwise
                case 'e':
                case 'E':
                    N_keyboard_ = -500.0;
                    n_key_active_ = true;
                    last_n_key_time_ = current_time; 
                    break;

                default:
                    break;
            }  
            RCLCPP_INFO(
                this->get_logger(),
                "Key=%c | X=%.1f Y=%.1f N=%.1f",
                key, X_keyboard_, Y_keyboard_, N_keyboard_);
        }
     if (x_key_active_)
            {
                auto current_time =
                    std::chrono::steady_clock::now();

                double elapsed_time =
                    std::chrono::duration<double>(
                        current_time - last_x_key_time_
                    ).count();

                if (elapsed_time > key_timeout_)
                {
                    X_keyboard_ = 0.0;
                    x_key_active_ = false;
                }
            }
    if (y_key_active_)
    {
        auto current_time =
            std::chrono::steady_clock::now();

        double elapsed_time =
            std::chrono::duration<double>(
                current_time - last_y_key_time_
            ).count();

        if (elapsed_time > key_timeout_)
        {
            Y_keyboard_ = 0.0;
            y_key_active_ = false;
        }
    }

    if (n_key_active_)
    {
        auto current_time =
            std::chrono::steady_clock::now();

        double elapsed_time =
            std::chrono::duration<double>(
                current_time - last_n_key_time_
            ).count();

        if (elapsed_time > key_timeout_)
        {
            N_keyboard_ = 0.0;
            n_key_active_ = false;
        }
    }

    }

    void PublishWrench(double X_body, double Y_body, double N_body){
        tf2::Quaternion q;
        // psi_ is already the bow heading in world coordinates.
        q.setRPY(0.0, 0.0, psi_);
        q.normalize();
        //Create force body object
        tf2::Vector3 force_body(
        X_body,
        Y_body,
        0.0);
        //Create torque body object
        tf2::Vector3 torque_body(
        0.0,
        0.0,
        N_body);
        
        tf2::Vector3 force_world = tf2::quatRotate(q, force_body);

        tf2::Vector3 torque_world = tf2::quatRotate(q, torque_body);
        
        ros_gz_interfaces::msg::EntityWrench msg;

        msg.header.stamp = this->now();
        msg.header.frame_id = "world";

        msg.entity.name = link_name_;
        msg.entity.type = ros_gz_interfaces::msg::Entity::LINK;

        // /wrench acts for one 1 ms physics step. At a 20 Hz controller rate,
        // scale by 1000 / 20 = 50 to preserve the intended average impulse.
        msg.wrench.force.x = wrench_step_scale_ * force_world.x();
        msg.wrench.force.y = wrench_step_scale_ * force_world.y();
        msg.wrench.force.z = wrench_step_scale_ * force_world.z();

        msg.wrench.torque.x = wrench_step_scale_ * torque_world.x();
        msg.wrench.torque.y = wrench_step_scale_ * torque_world.y();
        msg.wrench.torque.z = wrench_step_scale_ * torque_world.z();

        pub->publish(msg);
    }

    void ControlLoop()
{
    // ==========================================
    // MANUAL CONTROL
    // ==========================================
    if (keyboard_control_flag_)
    {
        KeyboardControl();

        PublishWrench(
            X_keyboard_,
            Y_keyboard_,
            N_keyboard_);

        return;
    }


    // ==========================================
    // AUTOMATIC CONTROL
    // ==========================================

    if (!state_received_ || !trajectory_received_)
    {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Waiting for state and /trajectory_cmd");
        return;
    }

    const double dt =
        1.0 / static_cast<double>(freq_);


    double error_u = u_d_ - u_;
    double error_v = v_d_ - v_;

    double error_psi = psi_d_ - psi_;

    // Wrap heading error to [-pi, pi]
    error_psi = atan2(
        sin(error_psi),
        cos(error_psi));


    // Surge PI
    const double Kp_u = 441.348;
    const double Ki_u = 117.693;

    // Sway PI
    const double Kp_v = 500.000;
    const double Ki_v = 612.500;

    // Heading PID
    const double Kp_psi = 523.152;
    const double Ki_psi = 74.736;
    const double Kd_psi = 280.260;


    const double max_force = 10000.0;
    const double max_yaw_torque = 5000.0;

    if (error_history_initialized_)
    {
        if (error_u * previous_error_u_ < 0.0)
        {
            integral_error_u_ = 0.0;
        }
        if (error_v * previous_error_v_ < 0.0)
        {
            integral_error_v_ = 0.0;
        }
        if (error_psi * previous_error_psi_ < 0.0)
        {
            integral_error_psi_ = 0.0;
        }
    }
    previous_error_u_ = error_u;
    previous_error_v_ = error_v;
    previous_error_psi_ = error_psi;
    error_history_initialized_ = true;

    const auto update_integral = [dt](
        double error, double kp, double ki, double limit, double & integral)
    {
        const double candidate = integral + error * dt;
        const double raw_output = kp * error + ki * candidate;
        const bool inside_limit = std::abs(raw_output) <= limit;
        const bool drives_back =
            (raw_output > limit && error < 0.0) ||
            (raw_output < -limit && error > 0.0);
        if (inside_limit || drives_back)
        {
            integral = candidate;
        }
    };

    update_integral(
        error_u, Kp_u, Ki_u, max_force, integral_error_u_);
    update_integral(
        error_v, Kp_v, Ki_v, max_force, integral_error_v_);
    update_integral(
        error_psi, Kp_psi, Ki_psi,
        max_yaw_torque, integral_error_psi_);

    double X_control =
        Kp_u * error_u
        +
        Ki_u * integral_error_u_;


    double Y_control =
        Kp_v * error_v
        +
        Ki_v * integral_error_v_;


    double N_control =
        Kp_psi * error_psi
        +
        Ki_psi * integral_error_psi_
        -
        Kd_psi * r_;

    X_control = std::clamp(X_control, -max_force, max_force);
    Y_control = std::clamp(Y_control, -max_force, max_force);
    N_control = std::clamp(
        N_control, -max_yaw_torque, max_yaw_torque);

    PublishWrench(
        X_control,
        Y_control,
        N_control);

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "u=%.3f/%.3f | v=%.3f/%.3f | psi=%.3f/%.3f | X=%.2f Y=%.2f N=%.2f",
        u_,
        u_d_,
        v_,
        v_d_,
        psi_,
        psi_d_,
        X_control,
        Y_control,
        N_control);
}
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ThrusterController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
