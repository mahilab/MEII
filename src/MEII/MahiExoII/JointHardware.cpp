#include <MEII/MahiExoII/JointHardware.hpp>
#include <Mahi/Util/Math/Functions.hpp>
#include <Mahi/Util/Logging/Log.hpp>
#include <Mahi/Com/MelShare.hpp>
#include <Mahi/Util/Timing/Frequency.hpp>
#include <Mahi/Util/Print.hpp>
#include <iostream>

using namespace mahi::util;

namespace meii {

JointHardware::JointHardware(const std::string &name,
             std::array<double, 2> position_limits,
             double velocity_limit,
             double torque_limit,
             mahi::robo::Limiter limiter,
             double actuator_transmission,
             std::shared_ptr<mahi::daq::EncoderHandle> position_sensor,
             double position_transmission,
             const double &velocity_sensor,
             VelocityEstimator velocity_estimator,
             double velocity_transmission,
             double motor_kt,
             double amp_gain,
             mahi::daq::DOHandle motor_enable_handle,
             mahi::daq::TTL motor_enable_value,
             mahi::daq::AOHandle amp_write_handle):
    Joint(name,position_limits,velocity_limit,torque_limit,limiter),
    m_position_sensor(position_sensor),
    m_velocity_sensor(velocity_sensor),
    m_velocity_estimator(velocity_estimator),
    m_actuator_transmission(actuator_transmission),
    m_position_transmission(position_transmission),
    m_velocity_transmission(velocity_transmission),
    m_motor_kt(motor_kt),
    m_amp_gain(amp_gain),
    m_motor_enable_handle(motor_enable_handle),
    m_motor_enable_value(motor_enable_value),
    m_amp_write_handle(amp_write_handle),
    m_velocity_filter(2,400_Hz,1000_Hz),
    m_clock()
    {
        m_clock.restart();
    }

bool JointHardware::enable() {
    m_limiter.reset();
    if (!m_motor_enable_handle.write_level(m_motor_enable_value)) return false;
    set_torque(0.0);
    m_enabled = true;
    return true;
}

bool JointHardware::disable() {
    set_torque(0.0);
    if (!m_motor_enable_handle.write_level(!m_motor_enable_value)) return false;
    m_enabled = false;
    return true;
}

void JointHardware::filter_velocity(){
    // only filter velocity if we are doing software filtering. otherwise it will
    // be coming straight from hardware already filtered
    if(m_velocity_estimator == VelocityEstimator::Software){
        // if this is the first loop through and it hasn't had a chance to get
        // position yet, then get the position. If this is the case, then the velocity
        // will read 0 for the first iteration (m_pos_last = pos_curr = get_position()),
        // but this should correct on the second pass-through
        if (m_pos_last == 0) {
            m_pos_last = get_position(); 
        }
        auto pos_curr  = get_position();
        auto time_curr = m_clock.get_elapsed_time().as_seconds();
        // mahi::util::print("curr time: {}, last time: {}\ncurr pos: {}, last pos: {}", time_curr, m_time_last, pos_curr, m_pos_last);
        auto vel_estimate = (pos_curr-m_pos_last)/(time_curr - m_time_last);

        m_vel_filtered = m_velocity_filter.update(vel_estimate);
        
        m_time_last = time_curr;
        m_pos_last  = pos_curr;
    }
}

double JointHardware::get_position() {
    return m_position_transmission*m_position_sensor->get_pos();
}

double JointHardware::get_velocity() {
    if(m_velocity_estimator == VelocityEstimator::Hardware){
        return m_velocity_transmission*m_velocity_sensor;
    }
    else{
        return m_vel_filtered;
    }
}

void JointHardware::set_torque(double new_torque) {
    if (m_enabled){
        m_com_torque = new_torque;
        if (torque_limit_exceeded()) {
            LOG(Warning) << "Joint " << get_name() << " command torque saturated to " << m_torque_limit;
            m_torque = clamp(m_com_torque, m_torque_limit);
        }
        else{
            m_torque = m_com_torque;
        }
        double motor_torque = m_torque * m_actuator_transmission;             // (Nm) * (Nm / Nm) = Nm
        double command_current = motor_torque/m_motor_kt;                     // (Nm) / (Nm / A)  = A
        double command_voltage = m_limiter.limit(command_current)/m_amp_gain; // (A)  / (A / V)   = V
        m_amp_write_handle.set_volts(command_voltage);
    }
}

} // namespace meii