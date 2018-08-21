#include <MEII/Daq/Crio.hpp>
#include <MEII/Daq/CrioAI.hpp>
#include <MEL/Logging/Log.hpp>
#include "Detail/NiFpga.h"

extern NiFpga_Session my_session;

namespace meii {

CrioAO::CrioAO(Crio& daq, const std::vector<mel::uint32>& channel_numbers) :
  daq_(daq)
{
    set_name(daq.get_name() + "_AO");
    set_channel_numbers(channel_numbers);
}

bool CrioAO::update_channel(mel::uint32 channel_number) {
    if (!daq_.is_open()) {
        LOG(Error) << "Unable to call " << __FUNCTION__ << " because "
                   << daq_.get_name() << " is not open";
        return false;
    }

    NiFpga_Status status;
    uint16_t valueScaled;
    double value = values_[channel_number];

    status = NiFpga_WriteU16(my_session, REGISTERS[type_][channel_number], valueScaled);
    if (status < 0) {
        LOG(Error) << "Failed to update " << get_name() << " channel number " << channel_number;
        return false;
    }

    status = NiFpga_WriteU16(my_session, AOSYSGO, 1);
    if (status < 0) {
        LOG(Error) << "Failed to update " << get_name() << " channel number " << channel_number;
        return false;
    }


    return true;
}

}  // namespace mel
