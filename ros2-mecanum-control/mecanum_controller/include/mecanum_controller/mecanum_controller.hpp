#ifndef MECANUM_CONTROLLER__MECANUM_CONTROLLER_HPP_
#define MECANUM_CONTROLLER__MECANUM_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <string>

#include "mecanum_controller/mecanum_wheel.hpp"
#include "mecanum_controller/visibility_control.h"

namespace mecanum_controller
{

  using Twist = geometry_msgs::msg::TwistStamped;
class MecanumController : public controller_interface::ControllerInterface
{
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
    // std::vector<std::string> interface_names_;
    // std::vector<std::string> joint_names_;
    std::shared_ptr<MecanumWheel> get_wheel(const std::string& wheel_joint_name);
    bool reset();

    rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscription_;
    realtime_tools::RealtimeBuffer<std::shared_ptr<Twist>> velocity_command_ptr_;
    std::shared_ptr<MecanumWheel> fl_wheel_;
    std::shared_ptr<MecanumWheel> fr_wheel_;
    std::shared_ptr<MecanumWheel> rl_wheel_;
    std::shared_ptr<MecanumWheel> rr_wheel_;

    std::string fl_wheel_joint_name_;
    std::string fr_wheel_joint_name_;
    std::string rl_wheel_joint_name_;
    std::string rr_wheel_joint_name_;
    
    double linear_scale_;
    double radial_scale_;
    double wheel_radius_;
    double wheel_distance_width_;
    double wheel_distance_length_;
    double wheel_separation_width_;
    double wheel_separation_length_;

    bool subscriber_is_active_;
};

}  // namespace mecanum_controller

#endif  // MECANUM_CONTROLLER__MECANUM_CONTROLLER_HPP_
