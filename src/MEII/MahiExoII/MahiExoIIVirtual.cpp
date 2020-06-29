#include <MEII/MahiExoII/MahiExoIIVirtual.hpp>
#include <MEII/MahiExoII/JointVirtual.hpp>
#include <Mahi/Robo/Control/Limiter.hpp>
#include <Mahi/Com/MelShare.hpp>
#include <array>

using namespace mahi::util;
using namespace mahi::com;
using namespace mahi::robo;

namespace meii {  

    ///////////////////////// STANDARD CLASS FUNCTIONS AND PARAMS /////////////////////////

    MahiExoIIVirtual::MahiExoIIVirtual(MeiiConfigurationVirtual configuration) :
        MahiExoII(),
        config_vr(configuration)
    {

        for (int i = 0; i < n_rj; ++i) {

            auto ms_trq = std::make_shared<MelShare>(config_vr.m_torque_ms_names[i]);
            auto ms_pos = std::make_shared<MelShare>(config_vr.m_posvel_ms_names[i]);

            auto joint = std::make_shared<JointVirtual>("meii_joint_" + std::to_string(i+1),
                                                 std::array<double, 2>({ params_.pos_limits_min_[i] , params_.pos_limits_max_[i] }),
                                                 params_.vel_limits_[i],
                                                 params_.joint_torque_limits[i],
                                                 Limiter(params_.motor_cont_limits_[i],
                                                            params_.motor_peak_limits_[i],
                                                            params_.motor_i2t_times_[i]),
                                                 ms_trq,
                                                 ms_pos,
                                                 config_vr.m_rest_positions[i]);

            meii_joints.push_back(joint);
        }
    }


} // namespace meii