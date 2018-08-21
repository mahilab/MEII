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

#include <MEII/Daq/Crio.hpp>
#include <MEII/Daq/CrioAI.hpp>
#include <MEL/Logging/Log.hpp>
#include "Detail/NiFpga.h"

extern NiFpga_Session my_session;

using namespace mel;
namespace meii {

CrioAI::CrioAI(Crio& daq, const std::vector<mel::uint32>& channel_numbers) :
  daq_(daq)
{
    set_name(daq.get_name() + "_AI");
    set_channel_numbers(channel_numbers);
}

bool CrioAI::update_channel(mel::uint32 channel_number) {
    if (!daq_.is_open()) {
        LOG(Error) << "Unable to call " << __FUNCTION__ << " because "
                   << daq_.get_name() << " is not open";
        return false;
    }
    NiFpga_Status status;
    uint16_t value = 0;
    status = NiFpga_ReadU16(my_session, REGISTERS[type_][channel_number], &value);
    if (status < 0) {
        LOG(Error) << "Failed to update " << get_name() << " channel number "  << channel_number;
        return false;
    }
    else {
        if (type_ == MyRioConnectorType::MspC)
            values_[channel_number] = (int16_t)value * WEIGHTS[type_][channel_number] + OFFSETS[type_][channel_number];
        else
            values_[channel_number] = value * WEIGHTS[type_][channel_number] + OFFSETS[type_][channel_number];
        return true;
    }
}

} // namespace mel