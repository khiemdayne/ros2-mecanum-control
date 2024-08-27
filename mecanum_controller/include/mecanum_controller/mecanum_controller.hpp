#ifndef MECANUM_CONTROLLER__MECANUM_CONTROLLER_HPP_
#define MECANUM_CONTROLLER__MECANUM_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <iostream>


#include "controller_interface/controller_interface.hpp"
#include "mecanum_controller/odometry.hpp"
#include "mecanum_controller/speed_limiter.hpp"
#include "mecanum_controller/visibility_control.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include "mecanum_controller_parameters.hpp"


namespace mecanum_controller
{

class MecanumController : public controller_interface::ControllerInterface
{
    using Twist = geometry_msgs::msg::TwistStamped;

public:
    MECANUM_CONTROLLER_PUBLIC
    MecanumController();

    MECANUM_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    MECANUM_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    MECANUM_CONTROLLER_PUBLIC
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    MECANUM_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    MECANUM_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    MECANUM_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    MECANUM_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    MECANUM_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

    MECANUM_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

    MECANUM_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

protected:
    struct WheelHandle
    {
        std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
        std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
    };
    const char *feedback_type() const;
    controller_interface::CallbackReturn configure_wheel(const std::string &wheel_name, std::vector<WheelHandle> &registered_handle);

    std::vector<WheelHandle> _registered_front_left_wheel_handle;
    std::vector<WheelHandle> _registered_front_right_wheel_handle;
    std::vector<WheelHandle> _registered_rear_left_wheel_handle;
    std::vector<WheelHandle> _registered_rear_right_wheel_handle;

    // Parameters from ROS for mecanum_controller
    std::shared_ptr<ParamListener> _param_listener;
    Params _params;

    Odometry _odometry;

    // Timeout to consider cmd_vel commands old
    std::chrono::milliseconds _cmd_vel_timeout{500};

    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> _odometry_publisher = nullptr;
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> _realtime_odometry_publisher = nullptr;

    std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> _odometry_transform_publisher = nullptr;
    std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> _realtime_odometry_transform_publisher = nullptr;

    bool _subscriber_is_active = false;
    rclcpp::Subscription<Twist>::SharedPtr _velocity_command_subscriber = nullptr;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _velocity_command_unstamped_subscriber = nullptr;

    realtime_tools::RealtimeBox<std::shared_ptr<Twist>> _received_velocity_msg_ptr{nullptr};

    std::queue<Twist> _previous_commands;  // last two commands

    // speed limiters
    SpeedLimiter _limiter_linear_x;
    SpeedLimiter _limiter_linear_y;
    SpeedLimiter _limiter_angular;

    bool _publish_limited_velocity = false;
    std::shared_ptr<rclcpp::Publisher<Twist>> _limited_velocity_publisher = nullptr;
    std::shared_ptr<realtime_tools::RealtimePublisher<Twist>> _realtime_limited_velocity_publisher = nullptr;

    rclcpp::Time _previous_update_timestamp{0};

    // publish rate limiter
    double _publish_rate = 50.0;
    rclcpp::Duration _publish_period = rclcpp::Duration::from_nanoseconds(0);
    rclcpp::Time _previous_publish_timestamp{0, 0, RCL_CLOCK_UNINITIALIZED};

    bool _is_halted = false;
    bool _use_stamped_vel = true;

    bool reset();
    void halt();
};

} // namespace mecanum_controller

#endif  // MECANUM_CONTROLLER__MECANUM_CONTROLLER_HPP_
