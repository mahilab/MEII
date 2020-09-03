// MIT License
//
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
// Author(s): Evan Pezent (epezent@rice.edu)
//            Craig McDonald (craig.g.mcdonald@gmail.com)

#pragma once

#include <MEII/MahiExoII/Joint.hpp>
#include <Mahi/Com/MelShare.hpp>

namespace meii {

//==============================================================================
// CLASS DECLARATION
//==============================================================================

class JointVirtual : public Joint {
public:
    /// Constructor
    JointVirtual(const std::string &name,
                 std::array<double, 2> position_limits,
                 double velocity_limit,
                 double torque_limit,
                 mahi::robo::Limiter limiter,
                 std::shared_ptr<mahi::com::MelShare> ms_trq,
                 std::shared_ptr<mahi::com::MelShare> ms_pos,
                 const double rest_pos);
    
    /// Converts PositionSensor position to Joint position
    double get_position();

    /// Converts PositionSensor velocity to Joint velocity
    double get_velocity();

    /// Sets the joint torque to #new_torque
    void set_torque(double new_torque);

    /// Enables the joint's position sensor, velocity sensor, and actuator
    bool enable();

    /// Disables the joint's position sensor, velocity sensor, and actuator
    bool disable();

private:
    const double m_rest_pos; // value to send if there is no melshare available

    std::shared_ptr<mahi::com::MelShare> ms_torque; // melshare to send torque to simulation
    std::shared_ptr<mahi::com::MelShare> ms_posvel; // melshare to receive position and velocity from simulation
};

} // namespace meii
