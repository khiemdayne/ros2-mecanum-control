#include <algorithm>
#include <stdexcept>

#include "mecanum_controller/speed_limiter.hpp"

namespace mecanum_controller
{

SpeedLimiter::SpeedLimiter(bool has_velocity_limits, bool has_acceleration_limits, bool has_jerk_limits, double min_velocity,
    double max_velocity, double min_acceleration, double max_acceleration, double min_jerk, double max_jerk)
:   _has_velocity_limits(has_velocity_limits),
    _has_acceleration_limits(has_acceleration_limits),
    _has_jerk_limits(has_jerk_limits),
    _min_velocity(min_velocity), _max_velocity(max_velocity),
    _min_acceleration(min_acceleration), _max_acceleration(max_acceleration),
    _min_jerk(min_jerk), _max_jerk(max_jerk)
{
    // Check if limits are valid, max must be specified, min defaults to -max if unspecified
    if (_has_velocity_limits)
    {
        if (std::isnan(_max_velocity))
        {
            throw std::runtime_error("Cannot apply velocity limits if max_velocity is not specified");
        }
        if (std::isnan(_min_velocity))
        {
            _min_velocity = -_max_velocity;
        }
    }
    if (_has_acceleration_limits)
    {
        if (std::isnan(_max_acceleration))
        {
            throw std::runtime_error("Cannot apply acceleration limits if max_acceleration is not specified");
        }
        if (std::isnan(_min_acceleration))
        {
            _min_acceleration = -_max_acceleration;
        }
    }
    if (_has_jerk_limits)
    {
        if (std::isnan(_max_jerk))
        {
            throw std::runtime_error("Cannot apply jerk limits if max_jerk is not specified");
        }
        if (std::isnan(_min_jerk))
        {
            _min_jerk = -_max_jerk;
        }
    }
}


double SpeedLimiter::limit(double & v, double v0, double v1, double dt)
{
    const double tmp = v;

    limit_jerk(v, v0, v1, dt);
    limit_acceleration(v, v0, dt);
    limit_velocity(v);

    return tmp != 0.0 ? v / tmp : 1.0;
}


double SpeedLimiter::limit_velocity(double & v)
{
    const double tmp = v;

    if (_has_velocity_limits)
    {
        v = std::clamp(v, _min_velocity, _max_velocity);
    }

    return tmp != 0.0 ? v / tmp : 1.0;
}


double SpeedLimiter::limit_acceleration(double & v, double v0, double dt)
{
    const double tmp = v;

    if (_has_acceleration_limits)
    {
        const double dv_min = _min_acceleration * dt;
        const double dv_max = _max_acceleration * dt;

        const double dv = std::clamp(v - v0, dv_min, dv_max);

        v = v0 + dv;
    }

    return tmp != 0.0 ? v / tmp : 1.0;
}

double SpeedLimiter::limit_jerk(double & v, double v0, double v1, double dt)
{
    const double tmp = v;

    if (_has_jerk_limits)
    {
        const double dv = v - v0;
        const double dv0 = v0 - v1;

        const double dt2 = 2. * dt * dt;

        const double da_min = _min_jerk * dt2;
        const double da_max = _max_jerk * dt2;

        const double da = std::clamp(dv - dv0, da_min, da_max);

        v = v0 + dv0 + da;
    }

    return tmp != 0.0 ? v / tmp : 1.0;
}

}  // namespace mecanum_controller
