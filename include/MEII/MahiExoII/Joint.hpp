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

#include <Mahi/Robo/Control/Limiter.hpp>
#include <array>

namespace meii {

//==============================================================================
// CLASS DECLARATION
//==============================================================================

class Joint {
public:
    /// Constructor
    Joint(const std::string &name,
          std::array<double, 2> position_limits,
          double velocity_limit,
          double torque_limit,
          mahi::robo::Limiter limiter);

    /// returns motor name
    std::string get_name() {return m_name;};
    
    /// Converts PositionSensor position to Joint position
    virtual double get_position() = 0;

    /// Converts PositionSensor velocity to Joint velocity
    virtual double get_velocity() = 0;

    /// Returns the currently set joint torque
    double get_torque_command() {return m_torque;};

    /// Sets the joint torque to #new_torque
    virtual void set_torque(double new_torque) = 0;

    /// Gets current position, checks it against limits, and returns true if min
    /// or max exceeded, false otherwise
    bool position_limit_exceeded();

    /// Gets current velocity, checks it against limit, and returns true if
    /// exceeded, false otherwise
    bool velocity_limit_exceeded();

    ///  Gets last commanded torque, checks it against torque limit, and returns
    ///  true if exceeded, false otherise
    bool torque_limit_exceeded();

    /// Gets current position, velocity, and torque, checks them against limits,
    /// and returns true if either exceeded, false otherwise
    bool any_limit_exceeded();

    /// Enables the joint's position sensor, velocity sensor, and actuator
    virtual bool enable() = 0;

    /// Disables the joint's position sensor, velocity sensor, and actuator
    virtual bool disable() = 0;

protected:
    std::string m_name;              // pointer to the Actuator of this Joint

    double m_torque;     // the stored torque (limited) of the Joint since the last call to set_torque()
    double m_com_torque; // the stored commanded (unlimited) torque of the Joint since the last call to set_torque()
    double m_position;   // the stored position of the Joint since the last call to get_position()
    double m_velocity;   // the stored velocity of the Joint since the last call to get_velocity()

    mahi::robo::Limiter m_limiter; // limiter that handles the limiting of the joint based on I^2t values

    bool saturate_;  // command torques will be saturated at the torque limit if this is true

    double m_torque_limit;  // the absolute limit on torque that should be allowed to the Joint

    std::array<double, 2> m_position_limits; // the [min, max] position limits of the Joint
    double m_velocity_limit;  // the absolute limit on the Joint's velocity

    bool has_torque_limit_    = true;  // whether or not the Joint should enforce torque limits
    bool has_position_limits_ = true;  // whether or not the Joint should check position limits
    bool has_velocity_limit_  = true;  // whether or not the Joint should check velocity limits

    bool m_enabled = false;
};

} // namespace meii
