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

#include <Mahi/Util/Math/Constants.hpp>
#include <string>
#include <vector>

namespace meii {

    //==============================================================================
    // FORWARD DECLARATIONS
    //==============================================================================

    class MahiExoIIVirtual;

    //==============================================================================
    // CLASS DECLARATION
    //==============================================================================

    /// Encapsulates the hardware configuration for a MahiExoII
    class MeiiConfigurationVirtual {

    public:

        /// Constructor for standard configuration
        MeiiConfigurationVirtual(const std::vector<double> rest_positions = {-45*mahi::util::DEG2RAD, 0, 0.0952, 0.0952, 0.0952},
                                 const std::vector<std::string> torque_ms_names = {"ms_torque_1",
                                                                                   "ms_torque_2",
                                                                                   "ms_torque_3",
                                                                                   "ms_torque_4",
                                                                                   "ms_torque_5"},
                                 const std::vector<std::string> posvel_ms_names = {"ms_posvel_1",
                                                                                   "ms_posvel_2",
                                                                                   "ms_posvel_3",
                                                                                   "ms_posvel_4",
                                                                                   "ms_posvel_5"}):
            m_rest_positions(rest_positions),
            m_torque_ms_names(torque_ms_names),
            m_posvel_ms_names(posvel_ms_names)
            {
            }

    private:

        friend class MahiExoIIVirtual;

        std::vector<double> m_rest_positions; // rest positions to use when there is no input from melshare
        std::vector<std::string> m_torque_ms_names; // names for the torque melshares
        std::vector<std::string> m_posvel_ms_names; // names for the position and velocity melshares
} // namespace meii
