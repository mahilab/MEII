#include <MEII/MahiExoII/Joint.hpp>
#include <Mahi/Util/Math/Functions.hpp>
#include <Mahi/Util/Logging/Log.hpp>
#include <iostream>

using namespace mahi::util;

namespace meii {

Joint::Joint(const std::string &name,
             std::array<double, 2> position_limits,
             double velocity_limit,
             double torque_limit,
             mahi::robo::Limiter limiter):
    m_name(name),
    m_torque(0.0),
    m_position(0.0),
    m_velocity(0.0),
    m_torque_limit(torque_limit),
    m_limiter(limiter),
    m_position_limits(position_limits),
    m_velocity_limit(velocity_limit)
    {
    }

bool Joint::position_limit_exceeded() {
    get_position();
    bool exceeded = false;
    if (has_position_limits_ && m_position < m_position_limits[0]) {
        LOG(Warning) << "Joint " << get_name() << " position exceeded the min position limit " << m_position_limits[0] << " with a value of " << m_position;
        exceeded = true;
    }
    if (has_position_limits_ && m_position > m_position_limits[1]) {
        LOG(Warning) << "Joint " << get_name() << " position exceeded the max position limit " << m_position_limits[1] << " with a value of " << m_position;
        exceeded = true;
    }
    return exceeded;
}

bool Joint::velocity_limit_exceeded() {
    get_velocity();
    bool exceeded = false;
    if (has_velocity_limit_ && abs(m_velocity) > m_velocity_limit) {
        LOG(Warning) << "Joint " << get_name() << " velocity exceeded the velocity limit " << m_velocity_limit << " with a value of " << m_velocity;
        exceeded = true;
    }
    return exceeded;
}

bool Joint::torque_limit_exceeded() {
    bool exceeded = false;
    if (has_torque_limit_ && abs(m_torque) > m_torque_limit) {
        LOG(Warning) << "Joint " << get_name() << " command torque exceeded the torque limit " << m_torque_limit << " with a value of " << m_torque;
        exceeded = true;
    }
    return exceeded;
}

bool Joint::any_limit_exceeded() {
    bool exceeded = false;
    if (position_limit_exceeded())
        exceeded = true;
    if (velocity_limit_exceeded())
        exceeded = true;
    if (torque_limit_exceeded())
        exceeded = true;
    return exceeded;
}

} // namespace meii