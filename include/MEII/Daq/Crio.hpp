// MIT License
//
// MEL - Mechatronics Engine & Library
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
// Author(s): Nathan Dunkelberger (nbd2@rice.edu)

#ifndef MEII_CRIO_HPP
#define MEII_CRIO_HPP

#include <MEL/Daq/DaqBase.hpp>
#include <MEL/Core/NonCopyable.hpp>

#include <MEII/Daq/CrioAO.hpp>
#include <MEII/Daq/CrioAI.hpp>
#include <MEII/Daq/CrioDIO.hpp>

namespace meii {

class CrioAI;
class CrioAO;
class CrioDIO;

/// National Instruments myRIO embedded system
class Crio : public mel::DaqBase, mel::NonCopyable {

public:

    /// Constructor
    Crio(const std::string& name);

    /// Default Destructor
    ~Crio();

    /// Updates all Input Modules simultaneously. It is generally more
    /// efficient to call this once per loop, than to call the update()
    /// function on each module separately.
    bool update_input() override;

    /// Updates all Output Modules simultaneously. It is generally more
    /// efficient to call this once per loop, than to call the update()
    /// function on each module separately.
    bool update_output() override;

private:

    bool on_open() override;
    bool on_close() override;

    /// Enables the myRIO by sequentially calling the enable() function
    /// on all I/O modules. Consult the documentation for each module for
    /// details on what the enable functions do.
    bool on_enable() override;

    /// Disables the myRIO by sequentially calling the disable() function
    /// on all I/O modules. Consult the documentation for each module for
    /// details on what the enable functions do.
    bool on_disable() override;

    /// Represents a myRIO connector
    class Module : public Device {
    public:
        Module();
        bool update_input();
        bool update_output();
    public:
        CrioAI  AI;
        CrioAO  AO;
        CrioDIO DIO;
    private:
        bool on_enable() override;
        bool on_disable() override;
    };

public:

     Module    Crio9264;  ///< MXP connector A
     Module    Crio9205;  ///< MXP connector B
     Module    Crio9401;  ///< MSP connector C

};

}  // namespace mel

#endif  // MEL_MYRIO_HPP
