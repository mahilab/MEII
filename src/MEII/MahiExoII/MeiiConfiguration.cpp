#include <MEII/MahiExoII/MeiiConfiguration.hpp>

using namespace mel;

namespace meii {

    //==============================================================================
    // CLASS DEFINTIONS
    //==============================================================================

    // MEII configuration without AI channels
    MeiiConfiguration::MeiiConfiguration(
        Q8Usb& daq,
        Watchdog& watchdog,
        const std::vector<Encoder::Channel>& encoder_channels,
        const std::vector<Amplifier>& amplifiers) :
        daq_(daq),
        watchdog_(watchdog),
        encoder_channels_(encoder_channels),
        amplifiers_(amplifiers)
    {
    }

    // MEII configuration with AI channels
    MeiiConfiguration::MeiiConfiguration(
        Q8Usb& daq,
        Watchdog& watchdog,
        const std::vector<Encoder::Channel>& encoder_channels,
        const std::vector<Amplifier>& amplifiers,
        const std::vector<AnalogInput::Channel>& ai_channels) :
        daq_(daq),
        watchdog_(watchdog),
        encoder_channels_(encoder_channels),
        amplifiers_(amplifiers),
        ai_channels_(ai_channels)
    {
    }

    // MEII configuration with cRIO & no AI channels

}