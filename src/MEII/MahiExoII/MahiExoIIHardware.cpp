#include <MEII/MahiExoII/MahiExoIIHardware.hpp>
#include <MEII/MahiExoII/JointHardware.hpp>
#include <MEII/MahiExoII/MeiiConfigurationHardware.hpp>
#include <Mahi/Daq/Quanser/Q8Usb.hpp>
#include <Mahi/Util/Math/Functions.hpp>
#include <Mahi/Util/Timing/Timer.hpp>
#include <iomanip>
#include <Mahi/Util/Print.hpp>
#include <Mahi/Util/Logging/Log.hpp>

using namespace mahi::util;
using namespace mahi::daq;
using namespace mahi::robo;

namespace meii {  

    ///////////////////////// STANDARD CLASS FUNCTIONS AND PARAMS /////////////////////////

    MahiExoIIHardware::MahiExoIIHardware(MeiiConfigurationHardware configuration) :
        MahiExoII(),
        config_hw(configuration)
    {

        for (int i = 0; i < n_rj; ++i) {

            // set encoder counts
            config_hw.m_daq.encoder.units[config_hw.m_encoder_channels[i]] = (2 * PI / params_.encoder_res_[i]);
            // encoder_handles.push_back(&EncoderHandle(config_hw.m_daq.encoder,i));

            auto encoder_handle = std::make_shared<EncoderHandle>(config_hw.m_daq.encoder,config_hw.m_encoder_channels[i]);

            auto joint = std::make_shared<JointHardware>("meii_joint_" + std::to_string(i),
                                                        std::array<double, 2>({ params_.pos_limits_min_[i] , params_.pos_limits_max_[i] }),
                                                        params_.vel_limits_[i],
                                                        params_.joint_torque_limits[i],
                                                        Limiter(params_.motor_cont_limits_[i],
                                                                    params_.motor_peak_limits_[i],
                                                                    params_.motor_i2t_times_[i]),
                                                        params_.eta_[i],
                                                        encoder_handle,
                                                        params_.eta_[i],
                                                        config_hw.m_daq.velocity.velocities[config_hw.m_encoder_channels[i]],
                                                        params_.eta_[i],
                                                        params_.kt_[i],
                                                        config_hw.m_amp_gains[i],
                                                        DOHandle(config_hw.m_daq.DO,config_hw.m_enable_channels[i]),
                                                        config_hw.m_enable_values[i],
                                                        AOHandle(config_hw.m_daq.AO,config_hw.m_current_write_channels[i]));

            meii_joints.push_back(joint);
        }
    }


} // namespace meii