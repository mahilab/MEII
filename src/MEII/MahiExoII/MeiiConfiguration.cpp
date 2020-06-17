#include <MEII/MahiExoII/MeiiConfiguration.hpp>

using namespace mahi::daq;

namespace meii {

    //==============================================================================
    // CLASS DEFINTIONS
    //==============================================================================

    // MEII configuration without AI channels
    MeiiConfiguration::MeiiConfiguration(
        Q8Usb& daq,
        const std::vector<mahi::daq::ChanNum> encoder_channels,
        const std::vector<mahi::daq::ChanNum> enable_channels,
        const std::vector<mahi::daq::ChanNum> current_write_channels,
        const std::vector<mahi::daq::TTL>     enable_values,
        const std::vector<double>             amp_gains) :
        daq_(daq),
        encoder_channels_(encoder_channels),
        enable_channels_(enable_channels),
        current_write_channels_(current_write_channels),
        enable_values_(enable_values),
        amp_gains_(amp_gains)
    {
    }

}