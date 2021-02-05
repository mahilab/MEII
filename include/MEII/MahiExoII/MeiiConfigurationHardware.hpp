// MIT License
//
// MEII - MAHI Exo-II Extension of MEL, the MAHI Exoskeleton Library
// Copyright (c) 2018 Mechatronics and Haptic Interfaces Lab - Rice University
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

#include <Mahi/Daq/Quanser/Q8Usb.hpp>
#include <Mahi/Daq/Quanser/QPid.hpp>
#include <Mahi/Daq/Quanser/QuanserEncoder.hpp>
#include <Mahi/Daq/Types.hpp>
#include <Mahi/Daq/Watchdog.hpp>
#include <vector>

namespace meii {

    //==============================================================================
    // FORWARD DECLARATIONS
    //==============================================================================
    template<typename Q>
    class MahiExoIIHardware;

    //==============================================================================
    // CLASS DECLARATION
    //==============================================================================

    /// Encapsulates the hardware configuration for a MahiExoII
    template<typename Q>
    class MeiiConfigurationHardware {

    public:

        /// Constructor for standard configuration
        MeiiConfigurationHardware(Q&                     daq,
                                  std::vector<mahi::daq::ChanNum> enable_channels = {1,2,3,4,5},
                                  std::vector<mahi::daq::ChanNum> current_write_channels = {1,2,3,4,5},
                                  std::vector<mahi::daq::TTL>     enable_values = std::vector<mahi::daq::TTL>(5,mahi::daq::TTL_LOW),
                                  std::vector<double>             amp_gains = {1.8, 1.8, 0.184, 0.184, 0.184}):
            m_daq(daq),
            m_enable_channels(enable_channels),
            m_current_write_channels(current_write_channels),
            m_enable_values(enable_values),
            m_amp_gains(amp_gains)
            {
                if (std::is_same<Q, mahi::daq::QPid>::value){
                    // the encoder channel 1 on the qpid is broken, so we use 6 instead
                    m_encoder_channels = {6,2,3,4,5};
                    // QPid uses DIO, so we need to explicitly specifiy that we are using DO
                    daq.DO.set_channels({0,1,2,3,4,5,6,7});
                } 
                else{
                    m_encoder_channels = {1,2,3,4,5};
                }
                    
            }

    private:
        template<typename Q>
        friend class MahiExoIIHardware;

        Q&                     m_daq;                    // DAQ controlling the MahiExoII
        std::vector<mahi::daq::ChanNum> m_encoder_channels;       // encoder channels that measure motor positions
        std::vector<mahi::daq::ChanNum> m_enable_channels;        // DO channels that enable/disable motors
        std::vector<mahi::daq::ChanNum> m_current_write_channels; // AI channels that write current to amps
        std::vector<mahi::daq::TTL>     m_enable_values;          // enable values for the amplifiers to enable the motors
        std::vector<double>             m_amp_gains;               // aplifier gain to convert volts to amps
    };
} // namespace meii
