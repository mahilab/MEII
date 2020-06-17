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
#include <Mahi/Daq/Quanser/QuanserEncoder.hpp>
// #include <Mahi/Daq/Input.hpp>
// #include <Mahi/Daq/Output.hpp>
#include <Mahi/Daq/Watchdog.hpp>
#include <vector>

namespace meii {

    //==============================================================================
    // FORWARD DECLARATIONS
    //==============================================================================

    class MahiExoII;

    //==============================================================================
    // CLASS DECLARATION
    //==============================================================================

    /// Encapsulates the hardware configuration for a MahiExoII
    class MeiiConfiguration {

    public:

        /// Constructor for standard configuration
        MeiiConfiguration(
            mahi::daq::Q8Usb& daq,
            const std::vector<mahi::daq::ChanNum> encoder_channels,
            const std::vector<mahi::daq::ChanNum> enable_channels,
            const std::vector<mahi::daq::ChanNum> current_write_channels,
            const std::vector<mahi::daq::TTL>     enable_values,
            const std::vector<double>             amp_gains);

    private:

        friend class MahiExoII;

        mahi::daq::Q8Usb&                     daq_;                    // DAQ controlling the MahiExoII
        const std::vector<mahi::daq::ChanNum> encoder_channels_;       // encoder channels that measure motor positions
        const std::vector<mahi::daq::ChanNum> enable_channels_;        // DO channels that enable/disable motors
        const std::vector<mahi::daq::ChanNum> current_write_channels_; // AI channels that write current to amps
        const std::vector<mahi::daq::TTL>     enable_values_;          // enable values for the amplifiers to enable the motors
        const std::vector<double>             amp_gains_;               // aplifier gain to convert volts to amps
    };
} // namespace meii
