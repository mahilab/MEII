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

#ifndef MEII_MEII_CONFIGURATION_HPP
#define MEII_MEII_CONFIGURATION_HPP

#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEL/Daq/Encoder.hpp>
#include <MEL/Daq/Input.hpp>
#include <MEL/Daq/Output.hpp>
#include <MEL/Daq/Watchdog.hpp>
#include <MEL/Mechatronics/Amplifier.hpp>
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
            mel::Q8Usb& daq,
            mel::Watchdog& watchdog,
            const std::vector<mel::Encoder::Channel>& encoder_channels,
            const std::vector<mel::Amplifier>& amplifiers);

        /// Constructor for EMG or Force Sensor configuration
        MeiiConfiguration(
            mel::Q8Usb& daq,
            mel::Watchdog& watchdog,
            const std::vector<mel::Encoder::Channel>& encoder_channels,
            const std::vector<mel::Amplifier>& amplifiers,
            const std::vector<mel::AnalogInput::Channel>& ai_channels);



    private:

        friend class MahiExoII;
        friend class MahiExoIIEmg;
        friend class MahiExoIIFrc;

        mel::Q8Usb&                                daq_;                ///< DAQ controlling the MahiExoII
        mel::Watchdog&                             watchdog_;           ///< watchdog the MahiExoII is guarded by
        std::vector<mel::Encoder::Channel>         encoder_channels_;   ///< encoder channels that measure motor positions
        std::vector<mel::Amplifier>                amplifiers_;         ///< amplifiers used to control robot motors
        std::vector<mel::AnalogInput::Channel>     ai_channels_;        ///< analog input channels that measure EMG

    };
} // namespace meii

#endif // MEII_MEII_CONFIGURATION_HPP

  //==============================================================================
  // CLASS DOCUMENTATION
  //==============================================================================