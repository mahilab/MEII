#include <MEL/Utility/System.hpp>
#include <MEL/Communications/Windows/MelShare.hpp>
#include <MEL/Utility/Options.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Logging/DataLogger.hpp>
#include <MEL/Utility/Console.hpp>
#include <MEL/Utility/Windows/Keyboard.hpp>
#include <MEII/Utility/logging_util.hpp>
#include <MEII/EMG/MesArray.hpp>
#include <MEL/Devices/Myo/MyoBand.hpp>
#include <MEII/EMG/EmgDataCapture.hpp>
#include <MEII/EmgRealTimeControl/EmgRealTimeControl.hpp>
#include <MEII/Unity/UnityMyoML.hpp>
#include <vector>

using namespace mel;
using namespace meii;


int main() {

	// enable Windows realtime
	enable_realtime();

	// initialize logger
	init_logger();

	// set emg channel numbers
	std::vector<uint32> emg_channel_numbers = { 0, 1, 2, 3, 4, 5, 6, 7 };
	std::size_t emg_channel_count = emg_channel_numbers.size();

	// construct and enable myo
	MyoBand myo("my_myo");
	myo.enable();

	// construct Myoelectric Signal (MES) Array
	MesArray mes(myo.get_channels(emg_channel_numbers), 300);	

	// make MelShares
	MelShare ms_emg("ms_emg");		

	// construct data logs
	Table emg_log("emg_log");
	emg_log.push_back_col("time");
	for (std::size_t i = 0; i < emg_channel_count; ++i) {
		emg_log.push_back_col("MES_RAW_" + stringify(emg_channel_numbers[i]));
	}
	emg_log.push_back_col("class_label");
	std::vector<double> emg_log_row(emg_log.col_count());
	bool save_data = true;

	// file management

	// construct timer in hybrid mode to avoid using 100% CPU
	Time Ts = milliseconds(1);
	Timer timer(Ts, Timer::Hybrid);

	// construct clock for regulating keypress
	Clock keypress_refract_clock;
	Time keypress_refract_time = seconds(0.5);
	
	// set up state machine
	enum State {
		Init,
		Neutral,
		Target,
		NewLabel,
		Finish,
		Last
	};
	std::vector<std::string> state_str = { "Init", "Neutral", "Target", "NewLabel", "Finish", "Last" };
	State state = Init;
	Time init_time = seconds(2);
	Time neutral_wait_time = seconds(1);
	Time target_wait_time = seconds(1);
	Time finish_time = seconds(2);
	bool stop = false;
	bool recording = false;
	Clock state_clock;
	

	// path for output data files
	std::string output_path = "C:\\Git\\MEII\\MyoArmbandMLData";

	// construct strings for printing conditions
	std::vector<std::string> arm_str = { "Left", "Right" };
	std::vector<std::string> dof_str = { "ElbowFE", "WristPS", "WristFE", "WristRU", "ElbowFE-WristPS", "WristFE-WristRU" };

	// prompt user for input to select which arm
	print("\r\nPress number key for selecting which arm is in the exo.");
	print("1 = Left arm");
	print("2 = Right arm");
	print("Press 'Escape' to exit the program.\r\n");
	int number_keypress;
	bool arm_selected = false;
	Arm arm = Left; // default
	while (!arm_selected && !stop) {

		// check for number keypress
		number_keypress = Keyboard::is_any_num_key_pressed();
		if (number_keypress >= 0) {
			if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
				if (number_keypress > 0 && number_keypress <= LastArm) {
					arm_selected = true;
					arm = (Arm)(number_keypress - 1);
					LOG(Info) << arm_str[arm] << " selected.";
				}
				keypress_refract_clock.restart();
			}
		}

		// check for exit key
		if (Keyboard::is_key_pressed(Key::Escape)) {
			stop = true;
			return 0;
		}

		// wait for remainder of sample period
		timer.wait();
	}

	// prompt user for input to select which DoF
	print("\r\nPress number key for selecting a single-DoF or multi-DoF trajectory.");
	print("1 = Elbow Flexion/Extension");
	print("2 = Wrist Pronation/Supination");
	print("3 = Wrist Flexion/Extension");
	print("4 = Wrist Radial/Ulnar Deviation");
	print("5 = Elbow Flexion/Extension and Wrist Pronation/Supination");
	print("6 = Wrist Flexion/Extension and Wrist Radial/Ulnar Deviation");
	print("Press 'Escape' to exit the program.\r\n");
	bool dof_selected = false;
	DoF dof = ElbowFE; // default
	std::size_t num_classes = 2; // number of classes based on the dof, 2 for single, 8 for multi
	while (!dof_selected && !stop) {

		// check for number keypress
		number_keypress = Keyboard::is_any_num_key_pressed();
		if (number_keypress >= 0) {
			if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
				if (number_keypress > 0 && number_keypress <= LastDoF) {
					dof = (DoF)(number_keypress - 1);
					dof_selected = true;
					LOG(Info) << dof_str[dof] << " selected.";
					if (is_single_dof(dof)) {
						num_classes = 2;
					}
					else {
						num_classes = 8;
					}
				}
				keypress_refract_clock.restart();
			}
		}

		// check for exit key
		if (Keyboard::is_key_pressed(Key::Escape)) {
			stop = true;
			return 0;
		}

		// wait for remainder of sample period
		timer.wait();
	}		

	// set file name based on selected conditions
	std::string file_prefix = arm_str[arm] + "_" + dof_str[dof];

	// launch and initialize unity game
	UnityMyoML game;
	game.launch();
	game.set_experiment_conditions(arm, dof);
	game.set_target(0);

	// generate labels
	std::size_t num_labels_per_class = 5;
	std::vector<std::size_t> class_labels = rand_shuffle_class_labels(num_labels_per_class, num_classes);
	std::size_t current_class_label_index = 0;
	std::size_t current_class_label = 0;

	// prompt user for input
	LOG(Info) << "Recording Myo Armband EMG for Machine Learning.";
	print("Press 'Enter' to begin the experiment and begin recording data.");
	print("Press 'Escape' to exit the program without saving data.");	

	// start loop
	state_clock.restart();
	while (!stop) {

		// update EMG
		myo.update();

		// update EMG signal processing
		mes.update_and_buffer();

		// begin switch state
		switch (state) {
		case Init:
			if (Keyboard::is_key_pressed(Key::Enter)) {
				if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
					LOG(Info) << "Beginning experiment and recording EMG.";
					recording = true;
					timer.restart();
					state_clock.restart();
					keypress_refract_clock.restart();				
				}
			}

			if (recording && state_clock.get_elapsed_time() >= init_time) {
				state = Neutral;
				current_class_label = 0;
				game.set_target(current_class_label);
				state_clock.restart();
			}
			break;

		case Neutral:
			if (state_clock.get_elapsed_time() >= neutral_wait_time) {
				state = Target;
				current_class_label = class_labels[current_class_label_index];
				game.set_target(current_class_label);
				state_clock.restart();
			}
			break;

		case Target:
			if (state_clock.get_elapsed_time() >= target_wait_time) {
				state = NewLabel;
				state_clock.restart();
			}
			break;

		case NewLabel:
			current_class_label_index++;
			if (current_class_label_index < class_labels.size()) {				
				state = Neutral;
				current_class_label = 0;
				game.set_target(current_class_label);
			}
			else {
				state = Finish;
				current_class_label = 0;
				game.set_target(current_class_label);
			}
			break;

		case Finish:
			if (state_clock.get_elapsed_time() >= finish_time) {
				state = Last;
				stop = true;
				save_data = true;
			}			
			break;

		case Last:
			LOG(Error) << "Should not have entered last state.";
			return 0;
			break;

		} // end switch state

		// write to MelShares
		ms_emg.write_data(mes.get_tkeo_envelope());
			
		// write to EMG data log
		if (recording) {
			emg_log_row[0] = timer.get_elapsed_time_ideal().as_seconds();
			for (std::size_t i = 0; i < emg_channel_count; ++i) {
				emg_log_row[i + 1] = mes.get_raw()[i];
			}
			emg_log_row[1 + emg_channel_count] = current_class_label;
			emg_log.push_back_row(emg_log_row);
		}

		// check for exit key
		if (Keyboard::is_key_pressed(Key::Escape)) {
			stop = true;
			save_data = false;
			LOG(Warning) << "Code stopped in state: " << state_str[state];
		}

		// wait for remainder of sample period
		timer.wait();

	}


	if (save_data) {
		LOG(Info) << "Saving Data.";
		DataLogger::write_to_csv(emg_log, file_prefix + "_" + "myo_armband_ml_emg_log", output_path, true);
		LOG(Info) << "Data Saved.";
	}


	mel::disable_realtime();
	return 0;
}


