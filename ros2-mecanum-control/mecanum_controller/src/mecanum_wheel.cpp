#include "mecanum_controller/mecanum_wheel.hpp"

using namespace mecanum_controller;

MecanumWheel::MecanumWheel(
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command
    ):  position_state_(position_state), 
        velocity_state_(velocity_state), 
        velocity_command_(velocity_command)
{

}

void MecanumWheel::set_velocity(double value)
{
    velocity_command_.get().set_value(value);
}