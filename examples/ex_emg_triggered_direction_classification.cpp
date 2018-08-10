#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEL/Communications/MelShare.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Core/Clock.hpp>
#include <MEL/Logging/DataLogger.hpp>
#include <MEL/Utility/Console.hpp>
#include <MEL/Utility/System.hpp>
#include <MEL/Utility/Windows/Keyboard.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEII/EMG/MesArray.hpp>
#include <MEII/Classification/EmgActiveEnsClassifier.hpp>
#include <MEII/Classification/EmgDirClassifier.hpp>
#include <MEII/EMG/EmgDataCapture.hpp>
#include <MEL/Utility/Options.hpp>
#include <MEII/Unity/UnityEmgRtc.hpp>

using namespace mel;
using namespace meii;

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

	// construct Q8 USB and configure    
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


	// construct array of Myoelectric Signals    
	MesArray mes(q8.analog_input.get_channels(emg_channel_numbers));

	// make MelShares
	MelShare ms_mes_tkeo_env("mes_tkeo_env");
	MelShare ms_mes_dm("mes_dm");

	// initialize testing conditions
	Time Ts = milliseconds(1); // sample period
	std::size_t hand = 0;
	std::size_t dof = 0;
	std::size_t condition = 0;
	std::size_t num_classes = 0;
	Time mes_rest_capture_period = seconds(1);
	Time mes_active_capture_period = seconds(1);
	Time mes_active_period = seconds(0.2);
	Time mes_dir_capture_period = seconds(0.2);
	Time active_training_refract_time = mes_active_capture_period;
	Time dir_training_refract_time = seconds(1);
	Time keypress_refract_time = seconds(0.5);

	// initialize active/rest calibration variables
	std::size_t mes_rest_capture_window_size = (std::size_t)((unsigned)(mes_rest_capture_period.as_seconds() / Ts.as_seconds()));
	std::size_t mes_active_capture_window_size = (std::size_t)((unsigned)(mes_active_capture_period.as_seconds() / Ts.as_seconds()));
	std::size_t mes_dir_capture_window_size = (std::size_t)((unsigned)(mes_dir_capture_period.as_seconds() / Ts.as_seconds()));
	mes.resize_buffer(std::max(mes_rest_capture_window_size, mes_active_capture_window_size));
	active_training_refract_time = seconds(std::max((double)((signed)mes.get_buffer_capacity()) * Ts.as_seconds(), active_training_refract_time.as_seconds()));
	std::size_t mes_active_window_size = (std::size_t)((unsigned)(mes_active_period.as_seconds() / Ts.as_seconds()));
	bool active_detector_saved = false;
	std::size_t active_state = 0;
	bool init_loop = true;

	// initialize direction classification variables
	std::size_t pred_dir = 0;

	// initialize UI variables
	std::vector<Key> active_keys = { Key::Num1, Key::Num2, Key::Num3, Key::Num4 };
	bool menu = false;
	bool menu_prev = true;
	int number_keypress;

	// initialize classifier
	bool RMS = true;
	bool MAV = true;
	bool WL = true;
	bool ZC = true;
	bool SSC = true;
	bool AR1 = true;
	bool AR2 = true;
	bool AR3 = true;
	bool AR4 = true;
	EmgActiveEnsClassifier active_detector(emg_channel_count, Ts);
	EmgDirClassifier dir_classifier(num_classes, emg_channel_count, Ts, RMS, MAV, WL, ZC, SSC, AR1, AR2, AR3, AR4);

	// construct Unity game
	UnityEmgRtc game;
	game.launch();
	game.set_target(-1);
	game.set_effort(0);
	game.set_center(false);

	// construct clock to regulate interaction
	Clock training_refract_clock;
	Clock keypress_refract_clock;
	
	

	// construct timer in hybrid mode to avoid using 100% CPU
	Timer timer(Ts, Timer::Hybrid);

	// enable DAQ
	q8.enable();

	// start while loop
	q8.watchdog.start();

	// promt the user for input
	print("Select a mode from the Unity main menu.");
	print("Press target number key to enable data capture for that target.");
	print("Press 'A + 0' to add 'rest' state training data to all classifiers.");
	print("Press 'C + 0' to clear 'rest' state training data from all classifiers.");
	print("Press 'A + target #' to add 'active' state training data for one classifier.");
	print("Press 'C + target #' to clear 'active' state training data for one classifier.");
	print("Press 'T' to train classifier and begin real-time classification.");
	print("Press 'Enter' to save active/rest classifier and move on to triggered direction classification.");
	print("Press 'Escape' to exit.");

	while (!stop) {

		// update all DAQ input channels
		q8.update_input();

		// emg signal processing
		mes.update_and_buffer();

		// check game conditions
		//game.set_experiment_conditions(hand, dof, num_classes, condition, menu);
		if (init_loop || (!menu && menu_prev)) {
			active_detector.resize(num_classes);
			dir_classifier.set_class_count(num_classes);
			game.set_target(-1);
			game.set_effort(0);
			game.set_center(false);
		}
		menu_prev = menu;

		// if scene has been selected
		if (!menu) {

			// predict state
			if (active_detector.update(mes.get_tkeo_envelope())) {
				active_state = active_detector.get_class();
				game.set_center(active_state == 1);
			}
			if (dir_classifier.update(mes.get_demean())) {
				pred_dir = dir_classifier.get_class();
			}

			if (!active_detector_saved) {

				// clear rest data
				if (Keyboard::are_all_keys_pressed({ Key::C, Key::Num0 })) {
					if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
						bool cleared_successfully = true;
						for (std::size_t k = 0; k < num_classes; ++k) {
							if (!active_detector.clear_training_data(k, 0)) {
								cleared_successfully = false;
							}
						}
						if (cleared_successfully) {
							LOG(Info) << "Cleared rest data.";
						}
						keypress_refract_clock.restart();
					}
				}

				// capture rest data
				if (Keyboard::are_all_keys_pressed({ Key::A, Key::Num0 })) {
					if (game.get_target_label() == -1) {
						if (mes.is_buffer_full()) {
							if (training_refract_clock.get_elapsed_time() > active_training_refract_time) {
								bool added_successfully = true;
								for (std::size_t k = 0; k < num_classes; ++k) {
									if (!active_detector.add_training_data(k, 0, mes.get_tkeo_env_buffer_data(mes_rest_capture_window_size))) {
										added_successfully = false;
									}
								}
								if (added_successfully) {
									LOG(Info) << "Added rest data.";
								}
								training_refract_clock.restart();
							}
						}
					}
				}

				// clear active data
				for (std::size_t k = 0; k < num_classes; ++k) {
					if (Keyboard::are_all_keys_pressed({ Key::C, active_keys[k] })) {
						if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
							if (active_detector.clear_training_data(k, 1))
								LOG(Info) << "Cleared active data for target " + stringify(k + 1) + ".";
							keypress_refract_clock.restart();
						}
					}
				}

				// capture active data
				for (std::size_t k = 0; k < num_classes; ++k) {
					if (Keyboard::are_all_keys_pressed({ Key::A, active_keys[k] })) {
						if ((std::size_t)((unsigned)game.get_target_label()) == k + 1) {
							if (mes.is_buffer_full()) {
								if (training_refract_clock.get_elapsed_time() > active_training_refract_time) {
									if (active_detector.add_training_data(k, 1, find_sum_max_window(mes.get_tkeo_env_buffer_data(mes_active_capture_window_size), mes_active_window_size))) {
										LOG(Info) << "Added active data for target " + stringify(k + 1) + ".";
									}
									training_refract_clock.restart();
								}
							}
						}
					}
				}

				// train the active/rest classifiers
				if (Keyboard::is_key_pressed(Key::T)) {
					if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
						if (active_detector.train())
							LOG(Info) << "Trained new active/rest classifier based on given data.";
						keypress_refract_clock.restart();
					}
				}

				// update visualization target
				if (!Keyboard::is_key_pressed(Key::A) && !Keyboard::is_key_pressed(Key::C)) {
					number_keypress = Keyboard::is_any_num_key_pressed();
					if (number_keypress >= 0) {
						if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
							if (number_keypress == 0 || number_keypress > num_classes) {
								number_keypress = -1;
							}
							game.set_target(number_keypress);
							keypress_refract_clock.restart();
						}
					}
				}
			
				// finish calibration and save the computed classifier
				if (Keyboard::is_key_pressed(Key::Enter)) {
					active_detector_saved = true;
					mes.resize_buffer(mes_dir_capture_window_size);
					dir_training_refract_time = seconds(std::max((double)((signed)mes.get_buffer_capacity()) * Ts.as_seconds(), dir_training_refract_time.as_seconds()));
					training_refract_clock.restart();
					game.set_target(-1);
					LOG(Info) << "Active/rest classifier saved.";
					print("Press target number key to enable triggered data capture for that target.");
					print("Press 'T' to train direction classifier and begin real-time classification.");
					print("Press 'Escape' to exit.");
				}

			} // end active/rest calibration and begin direction classification
			else {
				
				// clear training data
				for (std::size_t k = 0; k < num_classes; ++k) {
					if (Keyboard::are_all_keys_pressed({ Key::C, active_keys[k] })) {
						if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
							if (dir_classifier.clear_training_data(k)) {
								LOG(Info) << "Cleared training data for target " + stringify(k + 1) + ".";
							}
							keypress_refract_clock.restart();
						}
					}
				}

				// add training data
				for (std::size_t k = 0; k < num_classes; ++k) {
					if ((std::size_t)((unsigned)game.get_target_label()) == k + 1) {
						if (mes.is_buffer_full()) {
							if (active_state == 1) {
								if (training_refract_clock.get_elapsed_time() > dir_training_refract_time) {	
									if (dir_classifier.add_training_data(k, mes.get_dm_buffer_data(mes_dir_capture_window_size))) {
										LOG(Info) << "Added training data for target " + stringify(k + 1) + ".";
									}
									training_refract_clock.restart();
								}							
							}
						}
					}
				}

				// train the direction classifier
				if (Keyboard::is_key_pressed(Key::T)) {
					if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
						if (dir_classifier.train()) {
							LOG(Info) << "Trained new directional classifier based on given data.";
						}
						dir_classifier.train();
						keypress_refract_clock.restart();
					}
				}

				// update visualization target and enable active detection
				if (!Keyboard::is_key_pressed(Key::A) && !Keyboard::is_key_pressed(Key::C)) {
					number_keypress = Keyboard::is_any_num_key_pressed();
					if (number_keypress >= 0) {
						if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
							if (number_keypress == 0 || number_keypress > num_classes) {
								number_keypress = -1;
							}
							game.set_target(number_keypress);
							keypress_refract_clock.restart();
						}
					}
				}
			}
		}

		// write to MelShares
		ms_mes_tkeo_env.write_data(mes.get_tkeo_envelope());
		ms_mes_dm.write_data(mes.get_demean());

		// check for exit key
		if (Keyboard::is_key_pressed(Key::Escape)) {
			stop = true;
		}

		// kick watchdog
		if (!q8.watchdog.kick())
			stop = true;

		// wait for remainder of sample period
		timer.wait();

		if (init_loop) {
			init_loop = false;
		}
	} // end while loop       

	disable_realtime();
	return 0;
}

