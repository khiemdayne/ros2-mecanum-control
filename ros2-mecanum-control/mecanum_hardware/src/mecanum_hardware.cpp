
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>
#include <string>

#include "mecanum_hardware/mecanum_hardware.hpp"

PLUGINLIB_EXPORT_CLASS(
    mecanum_hardware::MecanumHardware,
    hardware_interface::SystemInterface
)

using namespace mecanum_hardware;

hardware_interface::CallbackReturn MecanumHardware::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
    hardware_interface::CallbackReturn baseResult = hardware_interface::SystemInterface::on_init(hardware_info);
    if (baseResult != hardware_interface::CallbackReturn::SUCCESS) {
        return baseResult;
    }

    serial_port_name_ = info_.hardware_parameters["serial_port"];
    motor_ids_.resize(info_.joints.size());
    position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_commands_saved_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    
    for (hardware_interface::ComponentInfo & joint : info_.joints)
    {
        if (joint.parameters["motor_id"].empty()) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumHardware"), "Motor id not defined for join %s", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumHardware"), "Invalid number of command interfaces (expected: 1)");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumHardware"), "Invalid joint command interface 0 type (expected: velocity)");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumHardware"), "Invalid number of state interfaces (expected: 2)");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumHardware"), "Invalid joint state interface 0 type (expected: position)");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumHardware"), "Invalid joint state interface 1 type (expected: velocity)");
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    for (size_t i = 0; i < info_.joints.size(); i++) {
        motor_ids_[i] = (uint8_t)std::stoi(info_.joints[i].parameters["motor_id"]);
        RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "%s mapped to motor %d", info_.joints[i].name.c_str(), motor_ids_[i]);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MecanumHardware::export_state_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "export_state_interfaces");

    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "Adding position state interface: %s", info_.joints[i].name.c_str());
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]
            )
        );
    }
    for (size_t i = 0; i < info_.joints.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "Adding velocity state interface: %s", info_.joints[i].name.c_str());
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]
            )
        );
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MecanumHardware::export_command_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "export_command_interfaces");

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "Adding velocity command interface: %s", info_.joints[i].name.c_str());
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]
            )
        );
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn MecanumHardware::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "Mecanum hardware starting ...");

    for (size_t i = 0; i < info_.joints.size(); i++) {
        if (std::isnan(position_states_[i])) {
            position_states_[i] = 0.0f;
        }
        if (std::isnan(velocity_states_[i])) {
            velocity_states_[i] = 0.0f;
        }
        if (std::isnan(velocity_commands_[i])) {
            velocity_commands_[i] = 0.0f;
        }
        velocity_commands_saved_[i] = velocity_commands_[i];
    }

    serial_port_ = std::make_shared<MecanumSerialPort>();
    if (serial_port_->open(serial_port_name_) != return_type::SUCCESS) {
        RCLCPP_WARN(rclcpp::get_logger("MecanumHardware"), "Mecanum hardware failed to open serial port");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "Mecanum hardware started");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "Mecanum hardware stopping ...");

    if (serial_port_->is_open()) {
        serial_port_->close();
        serial_port_.reset();
    }

    RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "Mecanum hardware stopped");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MecanumHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // Make sure we are connected to the serial port
    if (!serial_port_->is_open()) {
        RCLCPP_WARN(rclcpp::get_logger("MecanumHardware"), "Mecanum hardware not connected to serial port");
        return hardware_interface::return_type::ERROR;
    }

    // We currently have an ack response, so read the frames
    std::vector<SerialHdlcFrame> frames;
    serial_port_->read_frames(frames);

    /*
    for (size_t i = 0; i < frames.size(); i++) {
        char buff[100];
        int offset = 0;
        for (size_t l = 0; l < frames[i].length; l++) {
            sprintf(&buff[offset], "%02X ", frames[i].data[l]);
            offset += 3;
        }
        RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "Frame received: %s", buff);
    }
    */

    for (size_t i = 0; i < info_.joints.size(); i++) {
        //RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "Got position %.5f, velocity %.5f for joint %d!", position_states_[i], velocity_states_[i], i);
    }
    position_states_[0] = 1.1f;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MecanumHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    for (size_t i = 0; i < info_.joints.size(); i++) {
        // Only send motor commands if the velocity changed
        if (velocity_commands_[i] != velocity_commands_saved_[i]) {

            RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "Motor velocity changed: %.5f", velocity_commands_[i]);

            // Generate the motor command message
            uint16_t duty = 0;
            uint8_t message[6];
            message[0] = (uint8_t)DeviceCommand::MotorSetDuty;
            message[1] = 4; // Payload len
            message[2] = motor_ids_[i];
            if (velocity_commands_[i] >= 0.0) {
                duty = (uint16_t)(velocity_commands_[i]);
                message[3] = (uint8_t)DeviceMotorDirection::Forward;
            } else {
                duty = (uint16_t)(-velocity_commands_[i]);
                message[3] = (uint8_t)DeviceMotorDirection::Reverse;
            }
            message[4] = (uint8_t)(duty & 0xFF);
            message[5] = (uint8_t)((duty >> 8) & 0xFF);

            // Send the motor command
            if (serial_port_->is_open()) {
                serial_port_->write_frame(message, 6);
            }

            // Store the current velocity
            velocity_commands_saved_[i] = velocity_commands_[i];
        }
    }
    return hardware_interface::return_type::OK;
}