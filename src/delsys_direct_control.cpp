#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEII/MahiExoII/MahiExoII.hpp>
#include <MEL/Utility/System.hpp>
#include <MEL/Communications/Windows/MelShare.hpp>
#include <MEL/Utility/Options.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Logging/DataLogger.hpp>
#include <MEL/Utility/Console.hpp>
#include <MEL/Utility/Windows/Keyboard.hpp>
#include <MEII/Control/Trajectory.hpp>
#include <MEII/Control/DynamicMotionPrimitive.hpp>
#include <MEL/Math/Integrator.hpp>
#include <MEII/OpenSim/osim_utility.hpp>
#include <MEII/Utility/logging_util.hpp>
#include <MEII/EMG/MesArray.hpp>
#include <MEII/Classification/EmgActiveEnsClassifier.hpp>
#include <MEII/Classification/EmgDirClassifier.hpp>
#include <MEII/EMG/EmgDataCapture.hpp>
#include <MEII/EmgRealTimeControl/EmgRealTimeControl.hpp>
#include <MEII/Unity/UnityEmgRtc.hpp>
#include <vector>

using namespace mel;
using namespace meii;

// create global stop variable CTRL-C handler function
ctrl_bool stop(false);
bool handler(CtrlEvent event) {
	stop = true;
	return true;
}

int main(int argc, char *argv[]) {

	// handle inputs 
	std::vector<uint32> emg_channel_numbers;
	if (argc > 1) {
		uint32 ch;
		for (int i = 1; i < argc; ++i) {
			std::stringstream ss(argv[i]);
			ss >> ch;
			if (ch >= 0 && ch < 8) {
				emg_channel_numbers.push_back(ch);
			}
		}
	}
	else {
		return 0;
	}

	// enable Windows realtime
	enable_realtime();

	// initialize logger
	init_logger();

	// register ctrl-c handler
	register_ctrl_handler(handler);	

	// make Q8 USB and configure    
	Q8Usb q8(QOptions(), true, true, emg_channel_numbers); // specify all EMG channels
	q8.digital_output.set_enable_values(std::vector<Logic>(8, High));
	q8.digital_output.set_disable_values(std::vector<Logic>(8, High));
	q8.digital_output.set_expire_values(std::vector<Logic>(8, High));
	if (!q8.identify(7)) {
		LOG(Error) << "Incorrect DAQ";
		return 0;
	}
	emg_channel_numbers = q8.analog_input.get_channel_numbers();
	std::size_t emg_channel_count = q8.analog_input.get_channel_count();
	Time Ts = milliseconds(1); // sample period for DAQ

	// construct array of Myoelectric Signals    
	MesArray mes(q8.analog_input.get_channels(emg_channel_numbers));

	// make MelShares
	MelShare ms_emg("ms_emg");
	MelShare ms_pred("ms_pred");

	// construct global timer in hybrid mode to avoid using 100% CPU
	Timer timer(Ts, Timer::Hybrid);

	// initialize data capture variables			
	Time mes_rest_capture_period = seconds(1);
	Time mes_active_capture_period = seconds(3);
	Time mes_active_period = seconds(0.2);
	Time mes_dir_capture_period = seconds(0.2);
	bool capturing_active_data = false;
		

	// set data capture variables
	std::size_t mes_rest_capture_window_size = (std::size_t)((unsigned)(mes_rest_capture_period.as_seconds() / Ts.as_seconds()));
	std::size_t mes_active_capture_window_size = (std::size_t)((unsigned)(mes_active_capture_period.as_seconds() / Ts.as_seconds()));
	std::size_t mes_dir_capture_window_size = (std::size_t)((unsigned)(mes_dir_capture_period.as_seconds() / Ts.as_seconds()));
	mes.resize_buffer(std::max(mes_rest_capture_window_size, mes_active_capture_window_size));
	std::size_t mes_active_window_size = (std::size_t)((unsigned)(mes_active_period.as_seconds() / Ts.as_seconds()));		

	// construct data logs	
	EmgTable emg_std_log("EmgTable", emg_channel_numbers, true, true, true, true);
	std::vector<double> emg_std_log_row(emg_std_log.col_count());
	bool save_data = true;

	// enable DAQ and exo
	q8.enable();


	// prompt user for input
	print("Press 'Escape' to exit the program without saving data.");

	// start loop
	q8.watchdog.start();
	while (!stop) {

		// update all DAQ input channels
		q8.update_input();

		// update EMG signal processing
		mes.update_and_buffer();

		// predict effort

		// write prediction to melshare

		// clear rest data
		

		// capture rest data


		// clear active data

		// capture active data

		// train the active/rest classifiers
		

		// finish calibration and save the computed classifier
		

		// write to MelShares
		ms_emg.write_data(mes.get_tkeo_envelope());

		// write to EMG standard data log
		emg_std_log_row[0] = timer.get_elapsed_time_ideal().as_seconds();
		for (std::size_t i = 0; i < emg_channel_count; ++i) {
			emg_std_log_row[i + 1] = mes.get_raw()[i];
		}
		for (std::size_t i = 0; i < emg_channel_count; ++i) {
			emg_std_log_row[i + 1 + emg_channel_count] = mes.get_demean()[i];
		}
		for (std::size_t i = 0; i < emg_channel_count; ++i) {
			emg_std_log_row[i + 1 + 2 * emg_channel_count] = mes.get_envelope()[i];
		}
		for (std::size_t i = 0; i < emg_channel_count; ++i) {
			emg_std_log_row[i + 1 + 3 * emg_channel_count] = mes.get_tkeo_envelope()[i];
		}
		emg_std_log.push_back_row(emg_std_log_row);

		// update all DAQ output channels
		q8.update_output();

		// check for exit key
		if (Keyboard::is_key_pressed(Key::Escape)) {
			stop = true;
			save_data = false;
		}

		// kick watchdog
		if (!q8.watchdog.kick()) {
			stop = true;
		}

		// wait for remainder of sample period
		timer.wait();

	}
	q8.disable();


	if (save_data) {
		DataLogger::write_to_csv(emg_std_log, "emg_std_log", ".", false);
	}

	mel::disable_realtime();
	return 0;
}


