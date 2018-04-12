#include <MEII/Utility/VirtualInput.hpp>
#include <chrono>
#include <random>

using namespace mel;

namespace meii {

	VirtualInput::VirtualInput(const std::string& name, std::vector<mel::uint32> channel_numbers) :
		Input<Voltage>(name, channel_numbers),
		seed_(std::chrono::system_clock::now().time_since_epoch().count()),
		generator_(seed_),
		distribution_(0.0,1.0)
	{}

	bool VirtualInput::enable() {
		return Device::enable();
	}

	bool VirtualInput::disable() {
		return Device::disable();
	}

	bool VirtualInput::update() {
		for (std::size_t i = 0; i < channel_count_; ++i) {
			values_[i] = 0; // random noise
		}
	}

	bool VirtualInput::update_channel(mel::uint32 channel_number) {
		values_[channel_map_[channel_number]] = 0 ;//...
	}


} // namespace meii