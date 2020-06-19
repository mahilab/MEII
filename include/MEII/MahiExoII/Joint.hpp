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

#include <Mahi/Robo/Mechatronics/DcMotor.hpp>
#include <Mahi/Daq/Handle.hpp>
#include <Mahi/Com/MelShare.hpp>
#include <array>
#include <memory>

namespace meii {

//==============================================================================
// CLASS DECLARATION
//==============================================================================

class Joint {
public:
    /// Constructor
    Joint(const std::string &name,
          double actuator_transmission,
          mahi::daq::EncoderHandle* position_sensor,
          double position_transmission,
          const double &velocity_sensor,
          double velocity_transmission,
          std::array<double, 2> position_limits,
          double velocity_limit,
          double torque_limit,
          double motor_kt,
          double amp_gain,
          mahi::robo::Limiter limiter,
          mahi::daq::DOHandle motor_enable_handle,
          mahi::daq::TTL motor_enable_value,
          mahi::daq::AOHandle amp_write_handle,
          const bool is_virtual,
          std::shared_ptr<mahi::com::MelShare> ms_trq,
          std::shared_ptr<mahi::com::MelShare> ms_pos,
          const double rest_pos);

    /// returns motor name
    std::string get_name() {return m_name;};
    
    /// Converts PositionSensor position to Joint position
    double get_position();

    /// Converts PositionSensor velocity to Joint velocity
    double get_velocity();

    /// Returns the currently set joint torque
    double get_torque_command() {return m_torque;};

    /// Sets the joint torque to #new_torque
    void set_torque(double new_torque);

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
    bool enable();

    /// Disables the joint's position sensor, velocity sensor, and actuator
    bool disable();

private:
    std::string m_name;              // pointer to the Actuator of this Joint
    
    mahi::daq::EncoderHandle* m_position_sensor;  // pointer to the PositionSensor of this Joint
    double m_velocity_sensor;                     // pointer to the VelocitySensor of this Joint

    double m_actuator_transmission;    // transmission ratio describing the
                                     // multiplicative gain in torque from Joint
                                     // space to Actuator space, actuator torque =
                                     // actuator transmission * joint torque
    double m_position_transmission;  /// gain in position from PositionSensor space to Joint space, joint position = position sensor transmission * sensed position
    double m_velocity_transmission;  /// gain in velocity from VelocitySensor space to Joint space, joint velocity = velocity velocity sensor transmission * sensed

    double m_torque;    // the stored torque of the Joint since the last call to set_torque()
    double m_position;  // the stored position of the Joint since the last call to get_position()
    double m_velocity;  // the stored velocity of the Joint since the last call to get_velocity()

    double m_motor_kt; // motor gian in T/A
    double m_amp_gain; // amplifier gain A/V

    const double m_rest_pos; // value to send if there is no melshare available

    std::shared_ptr<mahi::com::MelShare> ms_torque; // melshare to send torque to simulation
    std::shared_ptr<mahi::com::MelShare> ms_posvel; // melshare to receive position and velocity from simulation

    mahi::robo::Limiter m_limiter; // limiter that handles the limiting of the joint based on I^2t values

    mahi::daq::DOHandle m_motor_enable_handle; // DO channel used to control if motor is enabled/disabled
    mahi::daq::TTL m_motor_enable_value; // Digital value to set for to enable motor through the amplifier
    mahi::daq::AOHandle m_amp_write_handle; // AO channel to write to for setting desired torque values

    bool saturate_;  // command torques will be saturated at the torque limit if this is true

    const bool m_is_virtual; // determines whether information is read from/sent to the daq or the melshare

    double m_torque_limit;  // the absolute limit on torque that should be allowed to the Joint

    std::array<double, 2> m_position_limits; // the [min, max] position limits of the Joint
    double m_velocity_limit;  // the absolute limit on the Joint's velocity

    bool has_torque_limit_    = true;  // whether or not the Joint should enforce torque limits
    bool has_position_limits_ = true;  // whether or not the Joint should check position limits
    bool has_velocity_limit_  = true;  // whether or not the Joint should check velocity limits

    bool m_enabled = false;
};

} // namespace meii
