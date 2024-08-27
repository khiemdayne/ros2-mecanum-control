#ifndef MECANUM_CONTROLLER__SPEED_LIMITER_HPP_
#define MECANUM_CONTROLLER__SPEED_LIMITER_HPP_

#include <cmath>

namespace mecanum_controller
{

class SpeedLimiter
{
public:
    /**
     * \brief Constructor
     * \param [in] has_velocity_limits     if true, applies velocity limits
     * \param [in] has_acceleration_limits if true, applies acceleration limits
     * \param [in] has_jerk_limits         if true, applies jerk limits
     * \param [in] min_velocity            Minimum velocity [m/s], usually <= 0
     * \param [in] max_velocity            Maximum velocity [m/s], usually >= 0
     * \param [in] min_acceleration        Minimum acceleration [m/s^2], usually <= 0
     * \param [in] max_acceleration        Maximum acceleration [m/s^2], usually >= 0
     * \param [in] min_jerk                Minimum jerk [m/s^3], usually <= 0
     * \param [in] max_jerk                Maximum jerk [m/s^3], usually >= 0
     */
    SpeedLimiter(
        bool has_velocity_limits = false, bool has_acceleration_limits = false,
        bool has_jerk_limits = false, double min_velocity = NAN, double max_velocity = NAN,
        double min_acceleration = NAN, double max_acceleration = NAN, double min_jerk = NAN,
        double max_jerk = NAN);

    /**
     * \brief Limit the velocity and acceleration
     * \param [in, out] v  Velocity [m/s]
     * \param [in]      v0 Previous velocity to v  [m/s]
     * \param [in]      v1 Previous velocity to v0 [m/s]
     * \param [in]      dt Time step [s]
     * \return Limiting factor (1.0 if none)
     */
    double limit(double & v, double v0, double v1, double dt);

    /**
     * \brief Limit the velocity
     * \param [in, out] v Velocity [m/s]
     * \return Limiting factor (1.0 if none)
     */
    double limit_velocity(double & v);

    /**
     * \brief Limit the acceleration
     * \param [in, out] v  Velocity [m/s]
     * \param [in]      v0 Previous velocity [m/s]
     * \param [in]      dt Time step [s]
     * \return Limiting factor (1.0 if none)
     */
    double limit_acceleration(double & v, double v0, double dt);

    /**
     * \brief Limit the jerk
     * \param [in, out] v  Velocity [m/s]
     * \param [in]      v0 Previous velocity to v  [m/s]
     * \param [in]      v1 Previous velocity to v0 [m/s]
     * \param [in]      dt Time step [s]
     * \return Limiting factor (1.0 if none)
     * \see http://en.wikipedia.org/wiki/Jerk_%28physics%29#Motion_control
     */
    double limit_jerk(double & v, double v0, double v1, double dt);

private:
    // Enable/Disable velocity/acceleration/jerk limits:
    bool _has_velocity_limits;
    bool _has_acceleration_limits;
    bool _has_jerk_limits;

    // Velocity limits:
    double _min_velocity;
    double _max_velocity;

    // Acceleration limits:
    double _min_acceleration;
    double _max_acceleration;

    // Jerk limits:
    double _min_jerk;
    double _max_jerk;
};

}

#endif  // MECANUM_CONTROLLER__SPEED_LIMITER_HPP_
