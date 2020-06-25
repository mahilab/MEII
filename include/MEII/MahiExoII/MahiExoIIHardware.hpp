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

namespace meii {
    /// Class for controlling the Mahi Exo II Exoskeleton
    class MahiExoIIHardware : public MahiExoII {

    ///////////////////////// STANDARD CLASS FUNCTIONS AND PARAMS /////////////////////////
    
    public:
        /// Constructor
        MahiExoIIHardware(MeiiConfigurationHardware configuration);

        MeiiConfigurationHardware config_hw;                       // meii configuration, consisting of daq, parameters, etc

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