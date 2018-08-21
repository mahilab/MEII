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

#ifndef MEII_CRIOAO_HPP
#define MEII_CRIOAO_HPP

#include <MEL/Daq/Output.hpp>
#include <MEL/Core/NonCopyable.hpp>

//==============================================================================
// FORWARD DECLARATIONS
//==============================================================================

namespace meii {

class Crio;

//==============================================================================
// CLASS DECLARATION
//==============================================================================

class CrioAO : public mel::AnalogOutput, mel::NonCopyable {
public:

    CrioAO(Crio& daq, const std::vector<mel::uint32>& channel_numbers);

    bool update_channel(mel::uint32 channel_number) override;

private:

    Crio& daq_;

};

}  // namespace meii

#endif  // MEII_CRIOAO_HPP
