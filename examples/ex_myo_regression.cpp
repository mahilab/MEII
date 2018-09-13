#include <MEL/Utility/System.hpp>
#include <MEL/Communications/MelShare.hpp>
#include <MEL/Utility/Options.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Logging/DataLogger.hpp>
#include <MEL/Core/Console.hpp>
#include <MEL/Devices/Windows/Keyboard.hpp>
#include <MEII/Utility/logging_util.hpp>
#include <MEII/EMG/MesArray.hpp>
#include <MEL/Devices/Myo/MyoBand.hpp>
#include <MEII/EMG/EmgDataCapture.hpp>
#include <MEII/EmgRealTimeControl/EmgRealTimeControl.hpp>
#include <MEII/EMG/EmgDirectMapping.hpp>
#include <MEII/Unity/UnityMyoML.hpp>
#include <vector>

using namespace mel;
using namespace meii;

// create global stop variable CTRL-C handler function
ctrl_bool stop(false);
bool handler(CtrlEvent event) {
	stop = true;
	return true;
}

int main() {

	// enable Windows realtime
	enable_realtime();

	// register ctrl-c handler
	register_ctrl_handler(handler);

	// set emg channel numbers
	std::vector<uint32> emg_channel_numbers = { 0, 1, 2, 3, 4, 5, 6, 7 };
	std::size_t emg_channel_count = emg_channel_numbers.size();

	// set sample rate
	Time Ts = milliseconds(1);

	// initialize data capture variables		
	Time mes_baseline_capture_period = seconds(1);
	Time mes_active_capture_period = seconds(5);

	// set data capture variables
	std::size_t mes_baseline_capture_window_size = (std::size_t)((unsigned)(mes_baseline_capture_period.as_seconds() / Ts.as_seconds()));
	std::size_t mes_active_capture_window_size = (std::size_t)((unsigned)(mes_active_capture_period.as_seconds() / Ts.as_seconds()));
	Clock baseline_capture_clock;
	Clock active_capture_clock;

	// construct and enable myo
	MyoBand myo("my_myo");
	myo.enable();

	// construct Myoelectric Signal (MES) Array
	std::size_t mes_buffer_capacity = std::max(mes_baseline_capture_window_size, mes_active_capture_window_size);
	MesArray mes(myo.get_channels(emg_channel_numbers), mes_buffer_capacity);

	// construct regressor
	EmgDirectMapping mes_map(emg_channel_count, Ts);
	mes_map.set_scaling(std::vector<double>(emg_channel_count, 2.0));
	mes_map.set_bias(std::vector<double>(emg_channel_count, 0.0));
	mes_map.set_channel_map(std::vector<std::size_t>{ 4, 1, 2, 3, 0, 5, 6, 7 });
	std::vector<double> pred(emg_channel_count, 0.0);

	// make MelShares
	MelShare ms_emg("emg");	
	MelShare ms_pred("pred");

	// construct global timer in hybrid mode to avoid using 100% CPU
	Timer global_timer(Ts, Timer::Hybrid);

	// construct clock for regulating keypress
	Clock keypress_refract_clock;
	Time keypress_refract_time = seconds(0.5);
	
	LOG(Info) << "Reading EMG from Myo Armband.";
	while (!stop) {

		// update all input channels
		myo.update();

		// update EMG signal processing
		mes.update_and_buffer();

		// predict dependent variable
		if (mes_map.update(mes.get_envelope())) {
			pred = mes_map.get_pred();
		}

		// write prediction to melshare
		ms_pred.write_data(pred);

		// clear baseline data
		if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
			if (Keyboard::are_all_keys_pressed({ Key::C, Key::Num0 })) {				
				mes_map.clear_baseline_data();
				LOG(Info) << "Cleared baseline data.";
				keypress_refract_clock.restart();
			}
		}

		// capture baseline data
		if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
			if (Keyboard::are_all_keys_pressed({ Key::A, Key::Num0 })) {			
				if (mes.is_buffer_full()) {
					if (baseline_capture_clock.get_elapsed_time() > mes_baseline_capture_period) {
						if (mes_map.add_baseline_data(mes.get_env_buffer_data(mes_baseline_capture_window_size))) {
							LOG(Info) << "Added baseline data";
						}
						baseline_capture_clock.restart();
					}
				}				
				else {
					LOG(Warning) << "MES buffer not full. Cannot add baseline data.";
				}
				keypress_refract_clock.restart();
			}		
		}

		// clear active data
		if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
			if (Keyboard::are_all_keys_pressed({ Key::C, Key::Num1 })) {
				mes_map.clear_active_data();
				LOG(Info) << "Cleared active data.";
				keypress_refract_clock.restart();
			}
		}

		// capture active data
		if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
			if (Keyboard::are_all_keys_pressed({ Key::A, Key::Num1 })) {
				if (mes.is_buffer_full()) {
					if (active_capture_clock.get_elapsed_time() > mes_active_capture_period) {
						if (mes_map.add_active_data(mes.get_env_buffer_data(mes_active_capture_window_size))) {
							LOG(Info) << "Added active data";
						}
						active_capture_clock.restart();
					}
				}
				else {
					LOG(Warning) << "MES buffer not full. Cannot add active data.";
				}
				keypress_refract_clock.restart();
			}
		}		

		// fit the direct mapping to the stored data
		if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
			if (Keyboard::is_key_pressed(Key::F)) {
				if (mes_map.fit()) {
					LOG(Info) << "Fit new direct mappping based on given data.";
				}
				keypress_refract_clock.restart();
			}
		}

		// write to MelShares
		ms_emg.write_data(mes.get_envelope());
		//ms_pred.write_data();


		// check for exit key
		if (Keyboard::is_key_pressed(Key::Escape)) {
			stop = true;
		}

		// wait for remainder of sample period
		global_timer.wait();

	}	


	mel::disable_realtime();
	return 0;
}
