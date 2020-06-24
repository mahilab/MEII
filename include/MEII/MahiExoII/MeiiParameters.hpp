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

#include <Mahi/Util/Math/Constants.hpp>
#include <Mahi/Util/Types.hpp>
#include <Mahi/Util/Timing/Time.hpp>
#include <array>

using mahi::util::seconds;
using mahi::util::INCH2METER;
using mahi::util::DEG2RAD;

namespace meii {

    /// Stores the constant parameters associated with the MAHI Exo-II.
    struct MeiiParameters {

        /// Default constructor.
        MeiiParameters() :
            //                    JOINT 0             JOINT 1             JOINT 2             JOINT 3             JOINT 4
            kt_{                       0.127,        0.0603,           0.175,           0.175,           0.175 }, // [Nm/A]
            motor_cont_limits_{          6.0,          3.17,           0.626,           0.626,           0.626 }, // [A]
            motor_peak_limits_{         18.0,          18.0,             1.8,             1.8,             1.8 }, // [A]
            motor_i2t_times_{   seconds(2.0),  seconds(2.0),    seconds(2.0),    seconds(2.0),    seconds(2.0) }, // [s]
            eta_{                 0.42 / 4.5,     0.0662864, 0.23*INCH2METER, 0.23*INCH2METER, 0.23*INCH2METER }, // [inch/inch] or [m]
            encoder_res_{               2048,          2048,            2048,            2048,            2048 }, // [counts/rev]
            pos_limits_min_{ -91.5 * DEG2RAD, -99 * DEG2RAD,           0.050,           0.050,           0.050 }, // [rad] or [m]
            pos_limits_max_{   3.0 * DEG2RAD, 108 * DEG2RAD,           0.133,           0.133,           0.133 }, // [rad] or [m]
            vel_limits_{       250 * DEG2RAD, 300 * DEG2RAD,             0.4,             0.4,             0.4 }, // [rad/s] or [m/s]
            joint_torque_limits{        10.0,          10.0,            50.0,            50.0,            50.0 }, // [Nm] or [N]
            kin_friction_{               0.0,           0.0,             0.0,             0.0,             0.0 }  // [Nm] or [N]
        { }



        /// motor torque constants [Nm/A]
        std::array<double, 5> kt_;
        /// motor continous current limits [A]
        std::array<double, 5> motor_cont_limits_;
        /// motor peak current limits [A]
        std::array<double, 5> motor_peak_limits_;
        /// motor i^2*t times [s]
        std::array<mahi::util::Time, 5> motor_i2t_times_;
        /// transmission ratios [inch/inch] or [m]
        std::array<double, 5> eta_;
        /// encoder resolutions [counts/rev]
        std::array<mahi::util::uint32, 5> encoder_res_;
        /// joint position limits minimum [rad] or [m]
        std::array<double, 5> pos_limits_min_;
        /// joint position limits maximum [rad] or [m]
        std::array<double, 5> pos_limits_max_;
        /// joint velocity limits [rad/s] or [m/s]
        std::array<double, 5> vel_limits_;
        /// joint torque limits [Nm] or [N]
        std::array<double, 5> joint_torque_limits;
        /// joint kinetic friction [Nm] or [N]
        std::array<double, 5> kin_friction_;
    };

} // namespace meii