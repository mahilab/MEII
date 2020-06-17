#include <MEII/MahiExoII/Joint.hpp>
#include <Mahi/Util/Math/Functions.hpp>
#include <Mahi/Util/Logging/Log.hpp>
#include <iostream>

using namespace mahi::util;

namespace meii {

Joint::Joint(const std::string &name,
        double actuator_transmission,
        mahi::daq::EncoderHandle* position_sensor,
        double position_sensor_transmission,
        const double &velocity_sensor,
        double velocity_sensor_transmission,
        std::array<double, 2> position_limits,
        double velocity_limit,
        double torque_limit,
        double motor_kt,
        double amp_gain,    
        mahi::robo::Limiter limiter,
        mahi::daq::DOHandle motor_enable_handle,
        mahi::daq::TTL motor_enable_value,
        mahi::daq::AOHandle amp_write_handle):
    m_name(name),
    m_position_sensor(position_sensor),
    m_velocity_sensor(velocity_sensor),
    m_actuator_transmission(actuator_transmission),
    m_position_transmission(position_sensor_transmission),
    m_velocity_transmission(velocity_sensor_transmission),
    m_torque(0.0),
    m_position(0.0),
    m_velocity(0.0),
    m_torque_limit(torque_limit),
    m_motor_kt(motor_kt),
    m_amp_gain(amp_gain),
    m_limiter(limiter),
    m_position_limits(position_limits),
    m_velocity_limit(velocity_limit),
    m_motor_enable_handle(motor_enable_handle),
    m_motor_enable_value(motor_enable_value),
    m_amp_write_handle(amp_write_handle)
{ }

bool Joint::enable() {
    m_limiter.reset();
    if (m_motor_enable_handle.write_level(m_motor_enable_value)){
        m_enabled = true;
        return true;
    }
    return false;
}

bool Joint::disable() {
    if (m_motor_enable_handle.write_level(!m_motor_enable_value)){
        m_enabled = false;
        return true;
    }   
    return false;
}

void Joint::set_torque(double new_torque) {
    if (m_enabled){
            m_torque = new_torque;
        if (torque_limit_exceeded()) {
            LOG(Warning) << "Joint " << get_name() << " command torque saturated to " << m_torque_limit;
            m_torque = clamp(m_torque, m_torque_limit);
        }
        double motor_torque = m_torque * m_actuator_transmission;             // (Nm) * (Nm / Nm) = Nm
        double command_current = motor_torque/m_motor_kt;                     // (Nm) / (Nm / A)  = A
        double command_voltage = m_limiter.limit(command_current)/m_amp_gain; // (A)  / (A / V)   = V
        m_amp_write_handle.set_volts(command_voltage);
    }
}

bool Joint::torque_limit_exceeded() {
    bool exceeded = false;
    if (has_torque_limit_ && abs(m_torque) > m_torque_limit) {
        LOG(Warning) << "Joint " << get_name() << " command torque exceeded the torque limit " << m_torque_limit << " with a value of " << m_torque;
        exceeded = true;
    }
    return exceeded;
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