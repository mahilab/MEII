// MIT License
//
// MEII - MAHI Exo-II Library
// Copyright (c) 2020 Mechatronics and Haptic Interfaces Lab - Rice University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// Author(s): Craig McDonald (craig.g.mcdonald@gmail.com)

#pragma once

#include <MEII/MahiExoII/MeiiConfigurationHardware.hpp>
#include <MEII/MahiExoII/MahiExoII.hpp>
#include <MEII/MahiExoII/JointHardware.hpp>
#include <Mahi/Robo/Control/Limiter.hpp>
#include <Mahi/Daq/Handle.hpp>

namespace meii {
    /// Class for controlling the Mahi Exo II Exoskeleton
    template<typename Q>
    class MahiExoIIHardware : public MahiExoII {

    ///////////////////////// STANDARD CLASS FUNCTIONS AND PARAMS /////////////////////////
    
    public:
    /// Constructor
        MahiExoIIHardware(MeiiConfigurationHardware<Q> configuration) :
            MahiExoII(),
            config_hw(configuration)
        {
            for (int i = 0; i < n_rj; ++i) {

                // set encoder counts
                config_hw.m_daq.encoder.units[config_hw.m_encoder_channels[i]] = (2 * mahi::util::PI / params_.encoder_res_[i]);
                // encoder_handles.push_back(&EncoderHandle(config_hw.m_daq.encoder,i));

                auto encoder_handle = std::make_shared<mahi::daq::EncoderHandle>(config_hw.m_daq.encoder,config_hw.m_encoder_channels[i]);

                auto joint = std::make_shared<JointHardware>("meii_joint_" + std::to_string(i),
                                                            std::array<double, 2>({ params_.pos_limits_min_[i] , params_.pos_limits_max_[i] }),
                                                            params_.vel_limits_[i],
                                                            params_.joint_torque_limits[i],
                                                            mahi::robo::Limiter(params_.motor_cont_limits_[i],
                                                                        params_.motor_peak_limits_[i],
                                                                        params_.motor_i2t_times_[i]),
                                                            params_.eta_[i],
                                                            encoder_handle,
                                                            params_.eta_[i],
                                                            config_hw.m_daq.velocity.velocities[config_hw.m_encoder_channels[i]],
                                                            (i > 0) ? VelocityEstimator::Hardware : config_hw.m_velocity_estimator,
                                                            params_.eta_[i],
                                                            params_.kt_[i],
                                                            config_hw.m_amp_gains[i],
                                                            mahi::daq::DOHandle(config_hw.m_daq.DO,config_hw.m_enable_channels[i]),
                                                            config_hw.m_enable_values[i],
                                                            mahi::daq::AOHandle(config_hw.m_daq.AO,config_hw.m_current_write_channels[i]));

                meii_joints.push_back(joint);
            }
        }
        MeiiConfigurationHardware<Q> config_hw;                       // meii configuration, consisting of daq, parameters, etc

        std::vector<mahi::daq::EncoderHandle*> encoder_handles;

    //////////////// OVERRIDING PURE VIRTUAL FUNCTIONS OF MEII ////////////////

    public:
        /// enables the daq
        bool daq_enable(){return config_hw.m_daq.enable();};
        /// disables the daq
        bool daq_disable(){return config_hw.m_daq.disable();};
        /// opens the daq
        bool daq_open(){return config_hw.m_daq.open();};
        /// closes the daq
        bool daq_close(){return config_hw.m_daq.close();};
        /// starts the watchdog on the daq
        bool daq_watchdog_start(){return config_hw.m_daq.watchdog.start();};
        /// starts the watchdog on the daq
        bool daq_watchdog_kick(){return config_hw.m_daq.watchdog.kick();};
        /// reads all from the daq
        bool daq_read_all(){return config_hw.m_daq.read_all();};
        /// writes all from the daq
        bool daq_write_all(){return config_hw.m_daq.write_all();};
        /// sets encoders to input position (in counts)
        bool daq_encoder_write(int index, mahi::util::int32 encoder_offset){return config_hw.m_daq.encoder.write(index,encoder_offset);};
    };
} // namespace meii