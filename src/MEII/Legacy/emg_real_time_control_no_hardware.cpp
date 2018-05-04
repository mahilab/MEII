#include <MEII/EmgRealTimeControl/EmgRealTimeControlNoHardware.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Communications/Windows/MelShare.hpp>
#include <MEL/Utility/Windows/Keyboard.hpp>
#include <MEL/Utility/Options.hpp>
#include <MEL/Utility/System.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Utility/Console.hpp>
#include <iostream>

using namespace mel;
using namespace meii;

// create global stop variable CTRL-C handler function
ctrl_bool stop(false);
bool handler(CtrlEvent event) {
	stop = true;
	return true;
}

int main(int argc, char * argv[]) {

	// make options
	Options options("emg_real_time_control_no_hardware.exe", "EMG Real-Time Control without MAHI Exo-II or real EMG");
	options.add_options()
		("r,run", "Runs the EMG Real-Time Control Experiment with No Hardware")
		("h,help", "Prints this help message");

	auto result = options.parse(argc, argv);

	if (result.count("help") > 0) {
		print(options.help());
		return 0;
	}

	//==========================================================================
	// SET THE EMG CHANNEL NUMBERS HERE
	//==========================================================================
	std::vector<uint32> emg_channel_numbers = { 0 };
	//==========================================================================


	// enable Windows realtime
	enable_realtime();

	// initialize default MEL logger
	init_logger();

	// register ctrl-c handler
	register_ctrl_handler(handler);


	// run the experiment
	if (result.count("run") > 0) {

		EmgRealTimeControlNoHardware emg_real_time_control(stop, emg_channel_numbers);
		emg_real_time_control.execute();

	}

	disable_realtime();
	return 0;

}