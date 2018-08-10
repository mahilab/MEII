#include <MEII/Utility/VirtualInput.hpp>


using namespace mel;

namespace meii {

	VirtualInput::VirtualInput(const std::string& name, std::vector<mel::uint32> channel_numbers) :
		seed_(std::chrono::system_clock::now().time_since_epoch().count()),
		generator_(seed_),
		distribution_(0.0,1.0)
	{
		set_name(name);
		set_channel_numbers(channel_numbers);
	}

	bool VirtualInput::enable() {
		return Device::enable();
	}

	bool VirtualInput::disable() {
		return Device::disable();
	}

	bool VirtualInput::update() { 
		for (std::size_t i = 0; i < get_channel_count(); ++i) {
			values_[i] = distribution_(generator_);
		}
		return true;
	}

	bool VirtualInput::update_channel(mel::uint32 channel_number) {
		values_[channel_number] = distribution_(generator_);
		return true;
	}


} // namespace meii