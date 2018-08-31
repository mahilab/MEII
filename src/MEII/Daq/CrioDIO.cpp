#include <MEII/Daq/Crio.hpp>
#include <MEII/Daq/CrioDIO.hpp>
#include <MEL/Logging/Log.hpp>
#include "Detail/NiFpga.h"

extern NiFpga_Session my_session;

using namespace mel;
namespace meii {

CrioDIO::CrioDIO(Crio& daq, const std::vector<mel::uint32>& channel_numbers) :
    daq_(daq)
{
    set_name(daq.get_name() + "_DIO");
    set_channel_numbers(channel_numbers);
}

bool CrioDIO::set_directions(const std::vector<Direction> &directions){
    if (!InputOutput::set_directions(directions))
        return false;
    t_error result;
    result = hil_set_digital_directions(daq_.handle_,
        &input_channel_numbers_[0], static_cast<uint32>(input_channel_numbers_.size()),
        &output_channel_numbers_[0], static_cast<uint32>(output_channel_numbers_.size()));
    if (result == 0) {
        LOG(Verbose) << "Set " << get_name() << " directions";
        return true;
    }
    else {
        LOG(Error) << "Failed to set " << get_name() << " directions "
            << QuanserDaq::get_quanser_error_message(result);
        return false;
    }
}

bool CrioDIO::set_direction(uint32 channel_number, Direction direction){
    if (!InputOutput::set_direction(channel_number, direction))
            return false;
    return set_directions(directions_.get());
}

bool CrioDIO::on_enable() {
    set_values(enable_values_.get());
    if (update()) {
        LOG(Verbose) << "Set " << get_name() << " enable values to "
                     << enable_values_;
        return true;
    } else {
        LOG(Error) << "Failed to set " << get_name() << " enable values to "
                   << enable_values_;
        return false;
    }
}

bool CrioDIO::on_disable() {
    set_values(disable_values_.get());
    if (update()) {
        LOG(Verbose) << "Set " << get_name() << " disable values to "
                     << disable_values_;
        return true;
    } else {
        LOG(Error) << "Failed to set " << get_name() << " disable values to "
                   << disable_values_;
        return false;
    }
}

bool CrioDIO::update_channel(uint32 channel_number) {
    NiFpga_Status status;
    uint8_t dirValue;
    if (directions_[channel_number] == In) {
        // read value
        uint8_t inValue;
        uint8_t dirMask;
        status = NiFpga_ReadU8(my_session, DIRS[type_][channel_number], &dirValue);
        if (status < 0) {
            LOG(Error) << "Could not read from DIO channel direction register";
            return false;
        }
        dirMask = 1 << BITS[type_][channel_number];
        dirMask = ~dirMask;
        dirValue = dirValue & dirMask;
        status = NiFpga_WriteU8(my_session, DIRS[type_][channel_number], dirValue);
        NiFpga_MergeStatus(&status, NiFpga_ReadU8(my_session, INS[type_][channel_number], &inValue));
        if (status < 0) {
            LOG(Error) << "Could not write to/read from the DIO channel direction/in register";
            return false;
        }
        inValue = inValue & (1 << BITS[type_][channel_number]);
        values_[channel_number] = (inValue > 0) ? High : Low;
    }
    else {
        // write value
        uint8_t outValue;
        NiFpga_Bool value = static_cast<NiFpga_Bool>(values_[channel_number]);
        status = NiFpga_ReadU8(my_session, OUTS[type_][channel_number], &outValue);
        NiFpga_MergeStatus(&status,  NiFpga_ReadU8(my_session, DIRS[type_][channel_number], &dirValue));
        if (status < 0) {
            LOG(Error) << "Could not read from the DIO channel registers!";
            return false;
        }
        dirValue = dirValue | (1 << BITS[type_][channel_number]);
        outValue = outValue & ~(1 << BITS[type_][channel_number]);
        outValue = outValue | (value << BITS[type_][channel_number]);
        NiFpga_MergeStatus(&status, NiFpga_WriteU8(my_session, OUTS[type_][channel_number], outValue));
        NiFpga_MergeStatus(&status, NiFpga_WriteU8(my_session, DIRS[type_][channel_number], dirValue));
        if (status < 0) {
            LOG(Error) << "Could not write to the DIO chanel registers!";
            return false;
        }
    }
    return true;
}

}  // namespace mel
