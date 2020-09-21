#include <MEII/MahiExoII/JointVirtual.hpp>
#include <Mahi/Util/Math/Functions.hpp>
#include <Mahi/Util/Logging/Log.hpp>
#include <Mahi/Com/MelShare.hpp>
#include <iostream>

using namespace mahi::util;

namespace meii {

JointVirtual::JointVirtual(const std::string &name,
                           std::array<double, 2> position_limits,
                           double velocity_limit,
                           double torque_limit,
                           mahi::robo::Limiter limiter,
                           std::shared_ptr<mahi::com::MelShare> ms_trq,
                           std::shared_ptr<mahi::com::MelShare> ms_pos,
                           const double rest_pos):
    Joint(name,position_limits,velocity_limit,torque_limit,limiter),
    ms_torque(ms_trq),
    ms_posvel(ms_pos),
    m_rest_pos(rest_pos)
    {

    }

bool JointVirtual::enable() {
    m_limiter.reset();
    set_torque(0.0);
    m_enabled = true;
    return m_enabled;
}

bool JointVirtual::disable() {
    set_torque(0.0);
    m_enabled = false;
    return true;
}

double JointVirtual::get_position() {
    std::vector<double> pos_vel_data = ms_posvel->read_data();
    return !pos_vel_data.empty() ? pos_vel_data[0] : m_rest_pos;
}

double JointVirtual::get_velocity() {
    std::vector<double> pos_vel_data = ms_posvel->read_data();
    return !pos_vel_data.empty() ? pos_vel_data[1] : 0.0;
}

void JointVirtual::set_torque(double new_torque) {
    if (m_enabled){
        m_com_torque = new_torque;
        if (torque_limit_exceeded()) {
            LOG(Warning) << "Joint " << get_name() << " command torque saturated to " << m_torque_limit;
            m_torque = clamp(m_com_torque, m_torque_limit);
        }
        else{
            m_torque = m_com_torque;
        }
        ms_torque->write_data({m_torque});
    }
}
} // namespace meii