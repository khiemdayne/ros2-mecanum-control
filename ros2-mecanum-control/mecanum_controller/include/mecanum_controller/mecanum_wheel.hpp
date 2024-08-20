#ifndef MECANUM_CONTROLLER__MECANUM_WHEEL_HPP__
#define MECANUM_CONTROLLER__MECANUM_WHEEL_HPP__

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

namespace mecanum_controller
{
    class MecanumWheel
    {
    public:
        MecanumWheel(
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state,
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state,
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command
            );

        void set_velocity(double value);

    private:
        std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state_;
        std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state_;
        std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command_;

    };
}

#endif // MECANUM_CONTROLLER__MECANUM_WHEEL_HPP__