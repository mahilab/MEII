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

// #include <MEII/MahiExoII/MeiiConfigurationVirtual.hpp>
#include <MEII/MahiExoII/MahiExoII.hpp>
#include <Mahi/Util/Print.hpp>

namespace meii {
    /// Class for controlling the Mahi Exo II Exoskeleton
    class MahiExoIIVirtual : public MahiExoII {

    ///////////////////////// STANDARD CLASS FUNCTIONS AND PARAMS /////////////////////////
    
    public:
        /// Constructor
        MahiExoIIVirtual();

        // MeiiConfigurationVirtual config_vr; // meii configuration, consisting of daq, parameters, etc

    private:
        std::vector<double> m_rest_positions = {-45*mahi::util::DEG2RAD, 0, 0.0952, 0.0952, 0.0952}; // rest positions to use when there is no input from melshare
        std::vector<std::string> m_torque_ms_names = {"ms_torque_1",
                                                      "ms_torque_2",
                                                      "ms_torque_3",
                                                      "ms_torque_4",
                                                      "ms_torque_5"}; // names for the torque melshares
        std::vector<std::string> m_posvel_ms_names = {"ms_posvel_1",
                                                      "ms_posvel_2",
                                                      "ms_posvel_3",
                                                      "ms_posvel_4",
                                                      "ms_posvel_5"}; // names for the position and velocity melshares

    //////////////// OVERRIDING PURE VIRTUAL FUNCTIONS OF MEII ////////////////
    public:
        /// enables the daq
        bool daq_enable(){return true;};
        /// disables the daq
        bool daq_disable(){return true;};
        /// opens the daq
        bool daq_open(){return true;};
        /// closes the daq
        bool daq_close(){return true;};
        /// starts the watchdog on the daq
        bool daq_watchdog_start(){return true;};
        /// starts the watchdog on the daq
        bool daq_watchdog_kick(){return true;};
        /// reads all from the daq
        bool daq_read_all(){return true;};
        /// writes all from the daq
        bool daq_write_all(){return true;};
        /// sets encoders to input position (in counts)
        bool daq_encoder_write(int index, mahi::util::int32 encoder_offset){return true;};
    };
} // namespace meii