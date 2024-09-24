#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "mecanum_controller/mecanum_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"


namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC           = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC       = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC          = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC         = "/tf";
}  // namespace


namespace mecanum_controller
{

using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

MecanumController::MecanumController() : controller_interface::ControllerInterface()
{

}


const char * MecanumController::feedback_type() const
{
    return _params.position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
}


controller_interface::CallbackReturn MecanumController::on_init()
{
    try
    {
        // Create the parameter listener and get the parameters
        _param_listener = std::make_shared<ParamListener>(get_node());
        _params         = _param_listener->get_params();
    }
    catch (const std::exception & e)
    {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}


InterfaceConfiguration MecanumController::command_interface_configuration() const
{
    // TODO: List of wheel link names and angles
    std::vector<std::string> conf_names;
    conf_names.push_back(_params.front_left_wheel_name  + "/" + HW_IF_VELOCITY);
    conf_names.push_back(_params.front_right_wheel_name + "/" + HW_IF_VELOCITY);
    conf_names.push_back(_params.rear_left_wheel_name   + "/" + HW_IF_VELOCITY);
    conf_names.push_back(_params.rear_right_wheel_name  + "/" + HW_IF_VELOCITY);

    return {interface_configuration_type::INDIVIDUAL, conf_names};
}


InterfaceConfiguration MecanumController::state_interface_configuration() const
{
    // TODO: List of wheel link names and angles
    std::vector<std::string> conf_names;
    conf_names.push_back(_params.front_left_wheel_name  + "/" + feedback_type());
    conf_names.push_back(_params.front_right_wheel_name + "/" + feedback_type());
    conf_names.push_back(_params.rear_left_wheel_name   + "/" + feedback_type());
    conf_names.push_back(_params.rear_right_wheel_name  + "/" + feedback_type());
    
    return {interface_configuration_type::INDIVIDUAL, conf_names};
}


controller_interface::return_type MecanumController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    auto logger = get_node()->get_logger();
    if (get_state().id() == State::PRIMARY_STATE_INACTIVE)
    {
        if (!_is_halted)
        {
            halt();
            _is_halted = true;
        }
        return controller_interface::return_type::OK;
    }

    std::shared_ptr<Twist> last_command_msg;
    _received_velocity_msg_ptr.get(last_command_msg);

    if (last_command_msg == nullptr)
    {
        RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
        return controller_interface::return_type::ERROR;
    }

    const auto age_of_last_command = time - last_command_msg->header.stamp;
    // Brake if cmd_vel has timeout, override the stored command
    if (age_of_last_command > _cmd_vel_timeout)
    {
        last_command_msg->twist.linear.x  = 0.0;
        last_command_msg->twist.linear.y  = 0.0;
        last_command_msg->twist.angular.z = 0.0;
    }

    // Command may be limited further by SpeedLimit, without affecting the stored twist command
    Twist command            = *last_command_msg;
    double &linear_command_x = command.twist.linear.x;
    double &linear_command_y = command.twist.linear.y;
    double &angular_command  = command.twist.angular.z;

    _previous_update_timestamp = time;

    // Apply (possibly new) multipliers:
    const double wheel_separation_x       = _params.wheel_separation_x_multiplier       * _params.wheel_separation_x;
    const double wheel_separation_y       = _params.wheel_separation_y_multiplier       * _params.wheel_separation_y;
    const double front_left_wheel_radius  = _params.front_left_wheel_radius_multiplier  * _params.wheel_radius;
    const double front_right_wheel_radius = _params.front_right_wheel_radius_multiplier * _params.wheel_radius;
    const double rear_left_wheel_radius   = _params.rear_left_wheel_radius_multiplier   * _params.wheel_radius;
    const double rear_right_wheel_radius  = _params.rear_right_wheel_radius_multiplier  * _params.wheel_radius;

    auto &last_command           = _previous_commands.back().twist;
    auto &second_to_last_command = _previous_commands.front().twist;

    _limiter_linear_x.limit(linear_command_x, last_command.linear.x, second_to_last_command.linear.x, period.seconds());
    _limiter_linear_y.limit(linear_command_y, last_command.linear.y, second_to_last_command.linear.y, period.seconds());
    _limiter_angular.limit(angular_command, last_command.angular.z, second_to_last_command.angular.z, period.seconds());

    _previous_commands.pop();
    _previous_commands.emplace(command);

    // Publish limited velocity
    if (_publish_limited_velocity && _realtime_limited_velocity_publisher->trylock())
    {
        auto &limited_velocity_command        = _realtime_limited_velocity_publisher->msg_;
        limited_velocity_command.header.stamp = time;
        limited_velocity_command.twist        = command.twist;
        _realtime_limited_velocity_publisher->unlockAndPublish();
    }

    // Compute wheels velocities:
    // TODO: List of wheel link names and angles
    const double lxly = (wheel_separation_x / 2.0) + (wheel_separation_y / 2.0);
    double front_left_velocity  = (linear_command_x - linear_command_y - lxly * angular_command) / front_left_wheel_radius;
    double front_right_velocity = (linear_command_x + linear_command_y + lxly * angular_command) / front_right_wheel_radius;
    double rear_left_velocity   = (linear_command_x + linear_command_y - lxly * angular_command) / rear_left_wheel_radius;
    double rear_right_velocity  = (linear_command_x - linear_command_y + lxly * angular_command) / rear_right_wheel_radius;

    // Check if wheel motor is able to reach the calculated wheel rotational speed
    // TODO: Trim the output cmd_vel to the reduced wheel rotational speed
    if (_params.wheel_max_rotational_speed != 0.0 &&
        (abs(front_left_velocity) > _params.wheel_max_rotational_speed || abs(front_right_velocity) > _params.wheel_max_rotational_speed ||
        abs(rear_left_velocity) > _params.wheel_max_rotational_speed || abs(rear_right_velocity) > _params.wheel_max_rotational_speed))
    {
        double reduce_factor = _params.wheel_max_rotational_speed / abs(front_left_velocity);
        if (_params.wheel_max_rotational_speed / abs(front_right_velocity) < reduce_factor)
        {
            reduce_factor = _params.wheel_max_rotational_speed / abs(front_right_velocity);
        }
        if (_params.wheel_max_rotational_speed / abs(rear_left_velocity) < reduce_factor)
        {
            reduce_factor = _params.wheel_max_rotational_speed / abs(rear_left_velocity);
        }
        if (_params.wheel_max_rotational_speed / abs(rear_right_velocity) < reduce_factor)
        {
            reduce_factor = _params.wheel_max_rotational_speed / abs(rear_right_velocity);
        }
        front_left_velocity  *= reduce_factor;
        front_right_velocity *= reduce_factor;
        rear_left_velocity   *= reduce_factor;
        rear_right_velocity  *= reduce_factor;
    }

    // Set wheels velocities:
    // TODO: List of wheel link names and angles
    _registered_front_left_wheel_handle[0].velocity.get().set_value(front_left_velocity);
    _registered_front_right_wheel_handle[0].velocity.get().set_value(front_right_velocity);
    _registered_rear_left_wheel_handle[0].velocity.get().set_value(rear_left_velocity);
    _registered_rear_right_wheel_handle[0].velocity.get().set_value(rear_right_velocity);

    // Odometry
    if (_params.open_loop)
    {
        // RCLCPP_ERROR(
        //   logger, "time is %2f .", _previous_update_timestamp);
        //   RCLCPP_ERROR(
        //   logger, "fl is %2f .", front_left_velocity);
        //   RCLCPP_ERROR(
        //   logger, "fr is %2f .", front_right_velocity);
        //   RCLCPP_ERROR(
        //   logger, "rl is %2f .", rear_left_velocity);
        //   RCLCPP_ERROR(
        //   logger, "rr is %2f .", rear_right_velocity);
        _odometry.updateFromVelocity(front_left_velocity, front_right_velocity, rear_left_velocity, rear_right_velocity, time);
    }
    else
    {
        // RCLCPP_ERROR(logger, "time is %2f .", _previous_update_timestamp);
        // RCLCPP_ERROR(
        //   logger, "fl is %2f .", front_left_velocity);
        //   RCLCPP_ERROR(
        //   logger, "fr is %2f .", front_right_velocity);
        //   RCLCPP_ERROR(
        //   logger, "rl is %2f .", rear_left_velocity);
        //   RCLCPP_ERROR(
        //   logger, "rr is %2f .", rear_right_velocity);
        const double front_left_feedback  = _registered_front_left_wheel_handle[0].feedback.get().get_value();
        const double front_right_feedback = _registered_front_right_wheel_handle[0].feedback.get().get_value();
        const double rear_left_feedback   = _registered_rear_left_wheel_handle[0].feedback.get().get_value();
        const double rear_right_feedback  = _registered_rear_right_wheel_handle[0].feedback.get().get_value();

        if (std::isnan(front_left_feedback) || std::isnan(front_right_feedback) ||
            std::isnan(rear_left_feedback)  || std::isnan(rear_right_feedback))
        {
            RCLCPP_ERROR(logger, "Either of the wheels %s is invalid.", feedback_type());
            return controller_interface::return_type::ERROR;
        }

        if (_params.position_feedback)
        {
            _odometry.update(front_left_feedback, front_right_feedback, rear_left_feedback, rear_right_feedback, time);
        }
        else
        {
            _odometry.updateFromVelocity(
                front_left_feedback  * front_left_wheel_radius  * period.seconds(),
                front_right_feedback * front_right_wheel_radius * period.seconds(),
                rear_left_feedback   * rear_left_wheel_radius   * period.seconds(),
                rear_right_feedback  * rear_right_wheel_radius  * period.seconds(),time);
        }
    }

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, _odometry.getHeading());

    bool should_publish = false;
    try
    {
        if (_previous_publish_timestamp + _publish_period < time)
        {
            _previous_publish_timestamp += _publish_period;
            should_publish              = true;
        }
    }
    catch (const std::runtime_error &)
    {
        // Handle exceptions when the time source changes and initialize publish timestamp
        _previous_publish_timestamp = time;
        should_publish              = true;
    }

    if (should_publish)
    {
        _previous_publish_timestamp += _publish_period;

        if (_realtime_odometry_publisher->trylock())
        {
            auto &odometry_message                   = _realtime_odometry_publisher->msg_;
            odometry_message.header.stamp            = time;
            odometry_message.pose.pose.position.x    = _odometry.getX();
            odometry_message.pose.pose.position.y    = _odometry.getY();
            odometry_message.pose.pose.orientation.x = orientation.x();
            odometry_message.pose.pose.orientation.y = orientation.y();
            odometry_message.pose.pose.orientation.z = orientation.z();
            odometry_message.pose.pose.orientation.w = orientation.w();
            odometry_message.twist.twist.linear.x    = _odometry.getLinearX();
            odometry_message.twist.twist.linear.y    = _odometry.getLinearY();
            odometry_message.twist.twist.angular.z   = _odometry.getAngular();
            _realtime_odometry_publisher->unlockAndPublish();
        }

        if (_params.enable_odom_tf && _realtime_odometry_transform_publisher->trylock())
        {
            auto &transform                   = _realtime_odometry_transform_publisher->msg_.transforms.front();
            transform.header.stamp            = time;
            transform.transform.translation.x = _odometry.getX();
            transform.transform.translation.y = _odometry.getY();
            transform.transform.rotation.x    = orientation.x();
            transform.transform.rotation.y    = orientation.y();
            transform.transform.rotation.z    = orientation.z();
            transform.transform.rotation.w    = orientation.w();
            _realtime_odometry_transform_publisher->unlockAndPublish();
        }
    }

    return controller_interface::return_type::OK;
}


controller_interface::CallbackReturn MecanumController::on_configure(const rclcpp_lifecycle::State &)
{
    auto logger = get_node()->get_logger();
    // update parameters if they have changed
    if (_param_listener->is_old(_params))
    {
        _params = _param_listener->get_params();
        RCLCPP_INFO(logger, "Parameters were updated");
    }

    // TODO: List of wheel link names and angles
    // if (_params.left_wheel_names.size() != _params.right_wheel_names.size())
    // {
    //     RCLCPP_ERROR(
    //     logger, "The number of left wheels [%zu] and the number of right wheels [%zu] are different",
    //     _params.left_wheel_names.size(), _params.right_wheel_names.size());
    //     return controller_interface::CallbackReturn::ERROR;
    // }

    // TODO: List of wheel link names and angles
    if (_params.front_left_wheel_name.empty() || _params.front_right_wheel_name.empty() ||
        _params.rear_left_wheel_name.empty()  || _params.rear_right_wheel_name.empty())
    {
        RCLCPP_ERROR(logger, "Wheel name parameters are empty!");
        return controller_interface::CallbackReturn::ERROR;
    }

    // TODO: List of wheel link names and angles
    const double wheel_separation_x       = _params.wheel_separation_x_multiplier       * _params.wheel_separation_x;
    const double wheel_separation_y       = _params.wheel_separation_y_multiplier       * _params.wheel_separation_y;
    const double front_left_wheel_radius  = _params.front_left_wheel_radius_multiplier  * _params.wheel_radius;
    const double front_right_wheel_radius = _params.front_right_wheel_radius_multiplier * _params.wheel_radius;
    const double rear_left_wheel_radius   = _params.rear_left_wheel_radius_multiplier   * _params.wheel_radius;
    const double rear_right_wheel_radius  = _params.rear_right_wheel_radius_multiplier  * _params.wheel_radius;
 
    _odometry.setWheelParams(wheel_separation_x, wheel_separation_y, front_left_wheel_radius, front_right_wheel_radius, rear_left_wheel_radius, rear_right_wheel_radius);
    _odometry.setVelocityRollingWindowSize(_params.velocity_rolling_window_size);

    _cmd_vel_timeout          = std::chrono::milliseconds{static_cast<int>(_params.cmd_vel_timeout * 1000.0)};
    _publish_limited_velocity = _params.publish_limited_velocity;
    _use_stamped_vel          = _params.use_stamped_vel;

    _limiter_linear_x = SpeedLimiter(
        _params.linear.x.has_velocity_limits, _params.linear.x.has_acceleration_limits, _params.linear.x.has_jerk_limits,
        _params.linear.x.min_velocity, _params.linear.x.max_velocity,
        _params.linear.x.min_acceleration, _params.linear.x.max_acceleration,
        _params.linear.x.min_jerk, _params.linear.x.max_jerk);
    
    _limiter_linear_y = SpeedLimiter(
        _params.linear.y.has_velocity_limits, _params.linear.y.has_acceleration_limits, _params.linear.y.has_jerk_limits,
        _params.linear.y.min_velocity, _params.linear.y.max_velocity,
        _params.linear.y.min_acceleration, _params.linear.y.max_acceleration,
        _params.linear.y.min_jerk, _params.linear.y.max_jerk);

    _limiter_angular = SpeedLimiter(
        _params.angular.z.has_velocity_limits, _params.angular.z.has_acceleration_limits, _params.angular.z.has_jerk_limits,
        _params.angular.z.min_velocity, _params.angular.z.max_velocity,
        _params.angular.z.min_acceleration, _params.angular.z.max_acceleration,
        _params.angular.z.min_jerk, _params.angular.z.max_jerk);

    if (!reset())
    {
        return controller_interface::CallbackReturn::ERROR;
    }

    if (_publish_limited_velocity)
    {
        _limited_velocity_publisher          = get_node()->create_publisher<Twist>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
        _realtime_limited_velocity_publisher = std::make_shared<realtime_tools::RealtimePublisher<Twist>>(_limited_velocity_publisher);
    }

    const Twist empty_twist;
    _received_velocity_msg_ptr.set(std::make_shared<Twist>(empty_twist));

    // Fill last two commands with default constructed commands
    _previous_commands.emplace(empty_twist);
    _previous_commands.emplace(empty_twist);

    // initialize command subscriber
    if (_use_stamped_vel)
    {
        _velocity_command_subscriber = get_node()->create_subscription<Twist>(
            DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
            [this](const std::shared_ptr<Twist> msg) -> void
            {
                if (!_subscriber_is_active)
                {
                    RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
                    return;
                }
                if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
                {
                    RCLCPP_WARN_ONCE(get_node()->get_logger(),
                        "Received TwistStamped with zero timestamp, setting it to current time, this message will only be shown once");
                    msg->header.stamp = get_node()->get_clock()->now();
                }
                _received_velocity_msg_ptr.set(std::move(msg));
            });
    }
    else
    {
        _velocity_command_unstamped_subscriber = get_node()->create_subscription<geometry_msgs::msg::Twist>(
            DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
            [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void
            {
                if (!_subscriber_is_active)
                {
                    RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
                    return;
                }
                // Write fake header in the stored stamped command
                std::shared_ptr<Twist> twist_stamped;
                _received_velocity_msg_ptr.get(twist_stamped);
                twist_stamped->twist = *msg;
                twist_stamped->header.stamp = get_node()->get_clock()->now();
            });
    }

    // initialize odometry publisher and messasge
    _odometry_publisher = get_node()->create_publisher<nav_msgs::msg::Odometry>(DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
    _realtime_odometry_publisher = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(_odometry_publisher);

    std::string controller_namespace = std::string(get_node()->get_namespace());

    if (controller_namespace == "/")
    {
        controller_namespace = "";
    }
    else
    {
        controller_namespace = controller_namespace.erase(0, 1) + "/";
    }

    const auto odom_frame_id = controller_namespace + _params.odom_frame_id;
    const auto base_frame_id = controller_namespace + _params.base_frame_id;

    auto &odometry_message           = _realtime_odometry_publisher->msg_;
    odometry_message.header.frame_id = odom_frame_id;
    odometry_message.child_frame_id  = base_frame_id;

    // limit the publication on the topics /odom and /tf
    _publish_rate   = _params.publish_rate;
    _publish_period = rclcpp::Duration::from_seconds(1.0 / _publish_rate);

    // initialize odom values zeros
    odometry_message.twist = geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

    constexpr size_t NUM_DIMENSIONS = 6;
    for (size_t index = 0; index < 6; ++index)
    {
        // 0, 7, 14, 21, 28, 35
        const size_t diagonal_index                       = NUM_DIMENSIONS * index + index;
        odometry_message.pose.covariance[diagonal_index]  = _params.pose_covariance_diagonal[index];
        odometry_message.twist.covariance[diagonal_index] = _params.twist_covariance_diagonal[index];
    }

    // initialize transform publisher and message
    _odometry_transform_publisher = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
    _realtime_odometry_transform_publisher = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(_odometry_transform_publisher);

    // keeping track of odom and base_link transforms only
    auto &odometry_transform_message = _realtime_odometry_transform_publisher->msg_;
    odometry_transform_message.transforms.resize(1);
    odometry_transform_message.transforms.front().header.frame_id = odom_frame_id;
    odometry_transform_message.transforms.front().child_frame_id  = base_frame_id;

    _previous_update_timestamp = get_node()->get_clock()->now();
    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn MecanumController::on_activate(const rclcpp_lifecycle::State &)
{
    const auto front_left_result  = configure_wheel(_params.front_left_wheel_name,  _registered_front_left_wheel_handle);
    const auto front_right_result = configure_wheel(_params.front_right_wheel_name, _registered_front_right_wheel_handle);
    const auto rear_left_result   = configure_wheel(_params.rear_left_wheel_name,   _registered_rear_left_wheel_handle);
    const auto rear_right_result  = configure_wheel(_params.rear_right_wheel_name,  _registered_rear_right_wheel_handle);

    if (front_left_result  == controller_interface::CallbackReturn::ERROR ||
        front_right_result == controller_interface::CallbackReturn::ERROR ||
        rear_left_result   == controller_interface::CallbackReturn::ERROR ||
        rear_right_result  == controller_interface::CallbackReturn::ERROR)
    {
        return controller_interface::CallbackReturn::ERROR;
    }

    if (_registered_front_left_wheel_handle.empty() || _registered_front_right_wheel_handle.empty() ||
        _registered_rear_left_wheel_handle.empty()  || _registered_rear_right_wheel_handle.empty())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Either of the wheel interfaces are non existent");
        return controller_interface::CallbackReturn::ERROR;
    }

    _is_halted = false;
    _subscriber_is_active = true;

    RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn MecanumController::on_deactivate(const rclcpp_lifecycle::State &)
{
    _subscriber_is_active = false;
    if (!_is_halted)
    {
        halt();
        _is_halted = true;
    }
    _registered_front_left_wheel_handle.clear();
    _registered_front_right_wheel_handle.clear();
    _registered_rear_left_wheel_handle.clear();
    _registered_rear_right_wheel_handle.clear();
    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn MecanumController::on_cleanup(const rclcpp_lifecycle::State &)
{
    if (!reset())
    {
        return controller_interface::CallbackReturn::ERROR;
    }
    _received_velocity_msg_ptr.set(std::make_shared<Twist>());
    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn MecanumController::on_error(const rclcpp_lifecycle::State &)
{
    if (!reset())
    {
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}


bool MecanumController::reset()
{
    _odometry.resetOdometry();
    // release the old queue
    std::queue<Twist> empty;
    std::swap(_previous_commands, empty);
    _registered_front_left_wheel_handle.clear();
    _registered_front_right_wheel_handle.clear();
    _registered_rear_left_wheel_handle.clear();
    _registered_rear_right_wheel_handle.clear();
    _subscriber_is_active = false;
    _velocity_command_subscriber.reset();
    _velocity_command_unstamped_subscriber.reset();
    _received_velocity_msg_ptr.set(nullptr);
    _is_halted = false;
    return true;
}


controller_interface::CallbackReturn MecanumController::on_shutdown(const rclcpp_lifecycle::State &)
{
    return controller_interface::CallbackReturn::SUCCESS;
}


void MecanumController::halt()
{
    const auto halt_wheels = [](auto & wheel_handles)
    {
        for (const auto &wheel_handle : wheel_handles)
        {
            wheel_handle.velocity.get().set_value(0.0);
        }
    };
    halt_wheels(_registered_front_left_wheel_handle);
    halt_wheels(_registered_front_right_wheel_handle);
    halt_wheels(_registered_rear_left_wheel_handle);
    halt_wheels(_registered_rear_right_wheel_handle);
}


controller_interface::CallbackReturn MecanumController::configure_wheel(
    const std::string &wheel_name, std::vector<WheelHandle> &registered_handles)
{
    auto logger = get_node()->get_logger();

    if (wheel_name.empty())
    {
        RCLCPP_ERROR(logger, "No wheel name specified");
        return controller_interface::CallbackReturn::ERROR;
    }

    // register handles
    registered_handles.reserve(1);
    const auto interface_name = feedback_type();
    const auto state_handle   = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(),
        [&wheel_name, &interface_name] (const auto &interface)
        {
            return interface.get_prefix_name() == wheel_name && interface.get_interface_name() == interface_name;
        });

    if (state_handle == state_interfaces_.cend())
    {
        RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }

    const auto command_handle = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
        [&wheel_name](const auto & interface)
        {
            return interface.get_prefix_name() == wheel_name && interface.get_interface_name() == HW_IF_VELOCITY;
        });

    if (command_handle == command_interfaces_.end())
    {
        RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }

    registered_handles.emplace_back(WheelHandle{std::ref(*state_handle), std::ref(*command_handle)});

    return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace mecanum_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(mecanum_controller::MecanumController, controller_interface::ControllerInterface)
