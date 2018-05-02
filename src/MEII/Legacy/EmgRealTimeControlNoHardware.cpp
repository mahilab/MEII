#include <MEII/EmgRealTimeControl/EmgRealTimeControlNoHardware.hpp>
#include <MEL/Utility/Windows/Keyboard.hpp>
#include <MEL/Utility/Console.hpp>
#include <MEL/Utility/System.hpp>
#include <MEL/Core/Motor.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEL/Logging/Log.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <random>
#include <numeric>
#include <stdexcept>
#include <ctime>

using namespace mel;

namespace meii {

EmgRealTimeControlNoHardware::EmgRealTimeControlNoHardware(ctrl_bool& manual_stop, std::vector<uint32> input_channel_numbers) :
	StateMachine(15),
	Ts_(milliseconds(1)),
	timer_(Ts_, Timer::Hybrid),
	vi_module_("virtual_input", input_channel_numbers),
	mes_(vi_module_.get_channels(input_channel_numbers)),
	manual_stop_(manual_stop),
	auto_stop_(false),
	exit_program_(false),
	menu_(true),
	end_of_label_sequence_(true),
	active_detector_computed_(false),
	active_detector_(input_channel_numbers.size(), Ts_),
	dir_classifier_(0, input_channel_numbers.size(), Ts_),
	current_class_label_idx_(-1),
	directory_share_("file_path"),
	file_name_share_("file_name"),
	cv_results_("cv_results"),
	lda_training_flag_("lda_training_flag")
{ }


//-----------------------------------------------------------------------------
// "WAIT FOR GUI" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRealTimeControlNoHardware::sf_wait_for_gui(const NoEventData* data) {
	LOG(Info) << "Waiting for Gui Input";

	// start the clock
	timer_.restart();

	// launch game
	game_.launch();

	// prompt user for input
	print("Select a mode in Unity to begin the experiment, or press 'Escape' to stop the experiment.");

	// check if scene selected in Unity and set conditions based on selected scene
	game_.set_experiment_conditions(hand_num_, dof_, num_classes_, condition_, menu_);
	game_.set_target(-1);

	// if still at main menu, wait for scene to be selected
	while (menu_ && !exit_program_ && !manual_stop_ && !auto_stop_) {

		// check if scene selected in Unity and set conditions based on selected scene
		game_.set_experiment_conditions(hand_num_, dof_, num_classes_, condition_, menu_);

		// check for program exit command from user
		check_exit_program();

		// wait for the next clock cycle
		timer_.wait();
	}

	// transition to next state from "WAIT FOR GUI"   
	if (auto_stop_ || manual_stop_) {
		event(ST_FAULT_STOP);
	}
	else if (exit_program_) {
		event(ST_STOP);
	}
	else if (!menu_) {
		event(ST_INIT);
	}
	else {
		LOG(Error) << "State transition undefined. Going to ST_FAULT_STOP.";
		event(ST_FAULT_STOP);
	}
}


//-----------------------------------------------------------------------------
// "INITIALIZATION" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRealTimeControlNoHardware::sf_init(const NoEventData* data) {
	LOG(Info) << "Initializing for " + str_conditions_long_[condition_] + " of " + str_dofs_long_[dof_];

	// reset global experiment variables
	end_of_label_sequence_ = true;
	class_label_sequence_.clear();
	pred_class_label_sequence_.clear();
	current_class_label_idx_ = -1;
	std::vector<double> init_flag = { 0.0 };
	lda_training_flag_.write_data(init_flag);

	// initialize classifier
	active_detector_.resize(num_classes_);
	dir_classifier_.set_class_count(num_classes_);

	// create subject and DoF specific folder for data logging
	subject_directory_ = project_directory_ + "\\EMG_S";
	if (subject_number_ < 10) {
		subject_directory_ += "0";
	}
	subject_directory_ += std::to_string(subject_number_);
	subject_dof_directory_ = subject_directory_ + "\\" + str_dofs_[dof_];

	// generate file names
	std::string str_subject_number = "S";
	if (subject_number_ < 10) {
		str_subject_number += "0";
	}
	str_subject_number += std::to_string(subject_number_);
	emg_active_classifier_filename_ = str_subject_number + "_" + str_dofs_[dof_] + "_emg_active_classifier";
	training_data_filename_ = str_subject_number + "_" + str_dofs_[dof_] + "_training_data";
	emg_dir_classifier_filename_ = str_subject_number + "_" + str_dofs_[dof_] + "_emg_dir_classifier";
	directory_share_.write_message(subject_dof_directory_);
	file_name_share_.write_message(training_data_filename_);

	// initialize data logging
	init_robot_log();
	init_emg_log();
	init_results_log();

	// confirm start of experiment
	LOG(Info) << "Running EMG Real-Time Control without Hardware.";


	// transition to next state from "INITIALIZATION"
	if (auto_stop_ || manual_stop_) {
		event(ST_FAULT_STOP);
	}
	else if (exit_program_) {
		event(ST_STOP);
	}
	else if (menu_) {
		event(ST_WAIT_FOR_GUI);
	}
	else {
		event(ST_BACKDRIVE);
	}
}


//-----------------------------------------------------------------------------
// "BACKDRIVE" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRealTimeControlNoHardware::sf_backdrive(const NoEventData* data) {
	LOG(Info) << "Robot Backdrivable";

	// initialize local state variables
	bool init_backdrive_time_reached = false;
	Time st_enter_time = timer_.get_elapsed_time();

	// enter the control loop
	while (!init_backdrive_time_reached && !menu_ && !exit_program_ && !manual_stop_ && !auto_stop_) {

		// check for init backdrive time reached
		init_backdrive_time_reached = check_wait_time_reached(init_backdrive_time_, st_enter_time, timer_.get_elapsed_time());

		// check if return to main menu selected in Unity
		game_.set_experiment_conditions(hand_num_, dof_, num_classes_, condition_, menu_);

		// check for program exit command from user
		check_exit_program();

		// wait for the next clock cycle
		timer_.wait();
	}

	// transition to next state from "BACKDRIVE"
	if (auto_stop_ || manual_stop_) {
		event(ST_FAULT_STOP);
	}
	else if (exit_program_) {
		event(ST_STOP);
	}
	else if (menu_) {
		event(ST_WAIT_FOR_GUI);
	}
	else if (init_backdrive_time_reached) {
		event(ST_INIT_RPS);
	}
	else {
		LOG(Error) << "State transition undefined. Going to ST_FAULT_STOP.";
		event(ST_FAULT_STOP);
	}
}


//-----------------------------------------------------------------------------
// "INITIALIZE RPS MECHANISM" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRealTimeControlNoHardware::sf_init_rps(const NoEventData* data) {
	LOG(Info) << "Initialize RPS Mechanism";
	print("Press 'Enter' to advance.");

	// initialize local state variables
	bool rps_init = false;


	// enter the control loop
	while (!rps_init && !menu_ && !exit_program_ && !manual_stop_ && !auto_stop_) {		

		// check for user advance
		if (Keyboard::is_key_pressed(Key::Enter)) {
			rps_init = true;
		}

		// check if return to main menu selected in Unity
		game_.set_experiment_conditions(hand_num_, dof_, num_classes_, condition_, menu_);

		// check for program exit command from user
		check_exit_program();

		// wait for the next clock cycle
		timer_.wait();
	}


	// transition to next state from "INITIALIZE RPS MECHANISM"
	if (auto_stop_ || manual_stop_) {
		event(ST_FAULT_STOP);
	}
	else if (exit_program_) {
		event(ST_STOP);
	}
	else if (menu_) {
		event(ST_WAIT_FOR_GUI);
	}
	else if (rps_init) {
		event(ST_TO_CENTER);
	}
	else {
		LOG(Error) << "State transition undefined. Going to ST_FAULT_STOP.";
		event(ST_FAULT_STOP);
	}
}



//-----------------------------------------------------------------------------
// "GO TO CENTER" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRealTimeControlNoHardware::sf_to_center(const NoEventData* data) {
	LOG(Info) << "Go to Center";
	print("Press 'Enter' to advance.");

	// initialize local state variables
	bool target_reached = false;

	// write to Unity
	game_.set_target(0);

	// enter the control loop
	while (!target_reached && !menu_ && !exit_program_ && !manual_stop_ && !auto_stop_) {

		// check for user advance
		if (Keyboard::is_key_pressed(Key::Enter)) {
			target_reached = true;
		}

		// check if return to main menu selected in Unity
		game_.set_experiment_conditions(hand_num_, dof_, num_classes_, condition_, menu_);
		

		// check for program exit command from user
		check_exit_program();

		// wait for the next clock cycle
		timer_.wait();
	}

	// transition to next state from "GOT TO CENTER"
	if (auto_stop_ || manual_stop_) {
		event(ST_FAULT_STOP);
	}
	else if (exit_program_) {
		event(ST_STOP);
	}
	else if (menu_) {
		event(ST_WAIT_FOR_GUI);
	}
	else if (target_reached) {
		if (is_cal()) {
			event(ST_CALIBRATION);
		}
		else if (is_training()) {
			event(ST_TRAINING);
		}
		else {
			++current_class_label_idx_;
			if (class_label_sequence_.empty() || (current_class_label_idx_ >= class_label_sequence_.size())) {
				end_of_label_sequence_ = true;
			}
			if (end_of_label_sequence_) {
				event(ST_HOLD_FOR_INPUT);
			}
			else {
				event(ST_HOLD_CENTER);
			}
		}
	}
	else {
		LOG(Error) << "State transition undefined. Going to ST_FAULT_STOP.";
		event(ST_FAULT_STOP);
	}
}


//-----------------------------------------------------------------------------
// "HOLD AT CENTER" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRealTimeControlNoHardware::sf_hold_center(const NoEventData* data) {
	LOG(Info) << "Hold at Center";


	hold_center_time_ = seconds(1.0);

	// initialize local state variables
	bool hold_center_time_reached = false;
	bool rest_state_reached = false;
	Time st_enter_time = timer_.get_elapsed_time();

	// write to Unity
	game_.set_target(0);

	// enter the control loop
	while (!rest_state_reached && !menu_ && !exit_program_ && !manual_stop_ && !auto_stop_) {

		// check for hold time reached
		hold_center_time_reached = check_wait_time_reached(hold_center_time_, st_enter_time, timer_.get_elapsed_time());
		

		// check if return to main menu selected in Unity
		game_.set_experiment_conditions(hand_num_, dof_, num_classes_, condition_, menu_);

		// check for program exit command from user
		check_exit_program();

		// wait for the next clock cycle
		timer_.wait();
	}

	// transition to next state from "HOLD AT CENTER"
	if (auto_stop_ || manual_stop_) {
		event(ST_FAULT_STOP);
	}
	else if (exit_program_) {
		event(ST_STOP);
	}
	else if (menu_) {
		event(ST_WAIT_FOR_GUI);
	}
	else if (rest_state_reached) {
		if (is_cal()) {
			log_results_row();
		}
		event(ST_PRESENT_TARGET);
	}
	else {
		LOG(Error) << "State transition undefined. Going to ST_FAULT_STOP.";
		event(ST_FAULT_STOP);
	}
}


//-----------------------------------------------------------------------------
// "HOLD FOR INPUT" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRealTimeControlNoHardware::sf_hold_for_input(const NoEventData* data) {
	LOG(Info) << "Hold for Input";

	// reset global variables
	if (class_label_sequence_.empty()) {
		end_of_label_sequence_ = false;
	}

	// initialize local state variables
	bool finished = false;
	bool present_more_targets = false;
	bool more_training_data = false;
	int num_observations_per_class;
	Key key;

	// write to Unity
	game_.set_target(0);

	// enter the control loop
	while (!present_more_targets && !finished && !menu_ && !exit_program_ && !manual_stop_ && !auto_stop_) {
		
		// check if return to main menu selected in Unity
		game_.set_experiment_conditions(hand_num_, dof_, num_classes_, condition_, menu_);

		// check for program exit command from user
		check_exit_program();

		// wait for the next clock cycle
		timer_.wait();
	}

	// transition to next state from "HOLD FOR INPUT"
	if (auto_stop_ || manual_stop_) {
		event(ST_FAULT_STOP);
	}
	else if (exit_program_) {
		event(ST_STOP);
	}
	else if (menu_) {
		event(ST_WAIT_FOR_GUI);
	}
	else if (finished) {
		event(ST_FINISH);
	}
	else if (present_more_targets) {
		end_of_label_sequence_ = false;
		if (!is_cal()) {
			std::vector<int> new_class_labels = rand_shuffle_class_labels(num_observations_per_class);
			for (int i = 0; i < new_class_labels.size(); ++i) {
				class_label_sequence_.push_back(new_class_labels[i]);
			}
		}
		event(ST_PRESENT_TARGET);
	}
	else {
		LOG(Error) << "State transition undefined. Going to ST_FAULT_STOP.";
		event(ST_FAULT_STOP);
	}
}

//-----------------------------------------------------------------------------
// "CALIBRATION" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRealTimeControlNoHardware::sf_calibration(const NoEventData* data) {
	LOG(Info) << "Begin Calibration of Active/Rest Detection";


	// methodological and control options - to be changed as desired
	Time mes_rest_capture_period = seconds(1);
	Time mes_active_capture_period = seconds(3);
	Time mes_active_period = milliseconds(200);

	// initialize methodological and control variables - not to be changed        
	std::size_t mes_rest_capture_window_size = (std::size_t)((unsigned)(mes_rest_capture_period.as_seconds() / Ts_.as_seconds()));
	std::size_t mes_active_capture_window_size = (std::size_t)((unsigned)(mes_active_capture_period.as_seconds() / Ts_.as_seconds()));
	mes_.resize_buffer(std::max(mes_rest_capture_window_size, mes_active_capture_window_size));
	std::size_t mes_active_window_size = (std::size_t)((unsigned)(mes_active_period.as_seconds() / Ts_.as_seconds()));
	bool active_detector_computed = false;
	std::size_t active_state = 0;
	std::vector<Key> active_keys = { Key::Num1, Key::Num2, Key::Num3, Key::Num4 };
	bool menu = false;
	bool menu_prev = true;
	int number_keypress;

	// construct clock to regulate interaction
	Clock training_refract_clock;
	Clock keypress_refract_clock;
	Time training_refract_time = seconds((double)((signed)mes_.get_buffer_capacity()) * Ts_.as_seconds());
	Time keypress_refract_time = seconds(0.5);

	// promt the user for input
	print("Select a mode from the Unity main menu.");
	print("Press 'A + 0' to add 'rest' state training data to all classifiers.");
	print("Press 'C + 0' to clear 'rest' state training data from all classifiers.");
	print("Press 'A + target #' to add 'active' state training data for one classifier.");
	print("Press 'C + target #' to clear 'active' state training data for one classifier.");
	print("Press 'T' to train classifier and begin real-time classification.");
	print("Press 'Escape' to exit.");

	while (!active_detector_computed_ && !menu_ && !exit_program_ && !manual_stop_ && !auto_stop_) {

		// predict state
		if (active_detector_.update(mes_.get_tkeo_envelope())) {
			active_state = active_detector_.get_class();
			game_.set_center(active_state == 1);
		}


		// check if return to main menu selected in Unity
		game_.set_experiment_conditions(hand_num_, dof_, num_classes_, condition_, menu_);

		// check for program exit command from user
		check_exit_program();

		// wait for the next clock cycle
		timer_.wait();
	}

	// transition to next state from "CALIBRATION"
	if (auto_stop_ || manual_stop_) {
		event(ST_FAULT_STOP);
	}
	else if (exit_program_) {
		event(ST_STOP);
	}
	else if (menu_) {
		event(ST_WAIT_FOR_GUI);
	}
	else if (active_detector_computed_)
		event(ST_FINISH);
	else {
		LOG(Error) << "State transition undefined. Going to ST_FAULT_STOP.";
		event(ST_FAULT_STOP);
	}
}

//-----------------------------------------------------------------------------
// "TRAINING" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRealTimeControlNoHardware::sf_training(const NoEventData* data) {
	LOG(Info) << "Begin Training of Directional Classifier";

	// initialize local state variables
	int num_observations_per_class = 5;
	bool checking_activity = false;
	bool collecting_data = false;
	//bool finished = false;
	//bool present_more_targets = false;
	//bool more_training_data = false;


	// reset global variables
	class_label_sequence_.clear();
	end_of_label_sequence_ = false;
	current_class_label_idx_ = 0;
	//emg_training_data_.clear();

	// generate new class labels
	std::vector<int> new_class_labels = rand_shuffle_class_labels(num_observations_per_class);
	for (int i = 0; i < new_class_labels.size(); ++i) {
		class_label_sequence_.push_back(new_class_labels[i]);
	}

	// initialize data containers
	int active_state = 0;
	int prev_active_state = 0;


	// make MelShares
	MelShare ms_mes_tkeo_env("melscope_mes_tkeo_env");
	MelShare ms_pred_state("melscope_pred_state");

	// construct clock to regulate keypresses
	Clock keypress_refract_clock;
	Time keypress_refract_time = seconds(1);

	// load active detector if it exists
	if (!load_emg_active_detector()) {
		LOG(Error) << "Failed to load active detector. Cannot run training.";
		auto_stop_ = true;
	}

	// load previous training data if it exists
	load_emg_dir_classifier();


	// prompt user for input
	print("Press 'Enter' to begin checking for activity.");

	while (!menu_ && !exit_program_ && !manual_stop_ && !auto_stop_) {

		// emg signal processing
		mes_.update();

		// write to emg data log
		log_emg_row();

		// update visual target
		game_.set_target(class_label_sequence_[current_class_label_idx_]);

		// predict active state
		active_detector_.update(mes_.get_tkeo_envelope());
		active_state = active_detector_.get_class();

		// check if active state has turned on
		if (checking_activity) {
			if (!prev_active_state && active_state) {
				collecting_data = true;

			}
		}


		

		// write to MelShares
		ms_mes_tkeo_env.write_data(mes_.get_tkeo_envelope());
		ms_pred_state.write_data({ (double)active_state });


		

		// check if user is ready to check for activity
		if (Keyboard::is_key_pressed(Key::Enter)) {
			if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
				checking_activity = !checking_activity; // toggle checking for activity
				if (checking_activity)
					LOG(Info) << "Checking for muscle activity to trigger data collection. Press 'Enter' to disable.";
				else
					LOG(Info) << "Disabled checking for muscle activity temporarily. Press 'Enter' to re-enable.";
				keypress_refract_clock.restart();
			}
		}

		// update variables
		prev_active_state = active_state;

		// check if return to main menu selected in Unity
		game_.set_experiment_conditions(hand_num_, dof_, num_classes_, condition_, menu_);

		// check for program exit command from user
		check_exit_program();

		// wait for the next clock cycle
		timer_.wait();
	}

	// transition to next state from "TRAINING"
	if (auto_stop_ || manual_stop_) {
		event(ST_FAULT_STOP);
	}
	else if (exit_program_) {
		event(ST_STOP);
	}
	else if (menu_) {
		event(ST_WAIT_FOR_GUI);
	}
	else {
		LOG(Error) << "State transition undefined. Going to ST_FAULT_STOP.";
		event(ST_FAULT_STOP);
	}
}


//-----------------------------------------------------------------------------
// "PRESENT TARGET" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRealTimeControlNoHardware::sf_present_target(const NoEventData* data) {
	LOG(Info) << "Present Target";



	// initialize local state variables
	bool active_state_reached = false;
	bool detector_expired = false;
	Time st_enter_time = timer_.get_elapsed_time();
	game_.set_effort_range(0.0, 1.0);


	// read from target sequence and write to Unity
	game_.set_target(class_label_sequence_[current_class_label_idx_]);


	print("Target class label: " + std::to_string(class_label_sequence_[current_class_label_idx_]));


	// enter the control loop
	while (!end_of_label_sequence_ && !active_state_reached && !detector_expired && !menu_ && !exit_program_ && !manual_stop_ && !auto_stop_) {

		

		// get measured emg voltages and log them
		mes_.update();
		log_emg_row();


		// check if return to main menu selected in Unity
		game_.set_experiment_conditions(hand_num_, dof_, num_classes_, condition_, menu_);

		// check for program exit command from user
		check_exit_program();

		// wait for the next clock cycle
		timer_.wait();
	}

	game_.set_effort(0.0);



	// transition to next state from "PRESENT TARGET"
	if (auto_stop_ || manual_stop_) {
		event(ST_FAULT_STOP);
	}
	else if (exit_program_) {
		event(ST_STOP);
	}
	else if (menu_) {
		event(ST_WAIT_FOR_GUI);
	}
	else if (active_state_reached) {
		//if (process_emg()) {
		//	if (is_training()) {
		//		log_trial_row();
		//		event(ST_TO_CENTER);
		//	}
		//	else {
		//		classify();
		//		if (is_blind()) {
		//			log_trial_row();
		//			event(ST_TO_CENTER);
		//		}
		//		else {
		//			log_trial_row();
		//			event(ST_TO_TARGET);
		//		}
		//	}
		//}
		//else {
		//	print("ERROR: EMG data unsuccessfully processed. Going to ST_FAULT_STOP.");
		//	event(ST_FAULT_STOP);
		//}
	}
	else if (detector_expired) {
		if (is_testing()) {
			pred_class_label_sequence_.push_back(0);
			if (is_blind()) {
				log_results_row();
				event(ST_TO_CENTER);
			}
			else {
				log_results_row();
				event(ST_TO_TARGET);
			}
		}
		else {
			print("ERROR: Boolean variable detector_expired should not be set to true outside of testing conditions.");
			event(ST_FAULT_STOP);
		}
	}
	else {
		LOG(Error) << "State transition undefined. Going to ST_FAULT_STOP.";
		event(ST_FAULT_STOP);
	}
}



//-----------------------------------------------------------------------------
// "GO TO TARGET" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRealTimeControlNoHardware::sf_to_target(const NoEventData* data) {
	LOG(Info) << "Go to Target";
	print("Press 'Enter' to advance.");

	// initialize local state variables
	bool target_reached = false;

	// set new reference position as predicted label
	std::vector<double> target_pos = get_target_position(pred_class_label_sequence_[current_class_label_idx_]);

	// write to Unity actual label
	game_.set_target(class_label_sequence_[current_class_label_idx_]);


	// enter the control loop
	while (!target_reached && !menu_ && !exit_program_ && !manual_stop_ && !auto_stop_) {

		// check for user advance
		if (Keyboard::is_key_pressed(Key::Enter)) {
			target_reached = true;
		}

		// check if return to main menu selected in Unity
		game_.set_experiment_conditions(hand_num_, dof_, num_classes_, condition_, menu_);

		// check for program exit command from user
		check_exit_program();

		// wait for the next clock cycle
		timer_.wait();
	}

	// transition to next state from "GO TO TARGET"
	if (auto_stop_ || manual_stop_) {
		event(ST_FAULT_STOP);
	}
	else if (exit_program_) {
		event(ST_STOP);
	}
	else if (menu_) {
		event(ST_WAIT_FOR_GUI);
	}
	else if (target_reached) {
		event(ST_HOLD_TARGET);
	}
	else {
		print("ERROR: State transition undefined. Going to ST_FAULT_STOP.");
		event(ST_FAULT_STOP);
	}
}


//-----------------------------------------------------------------------------
// "HOLD AT TARGET" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRealTimeControlNoHardware::sf_hold_target(const NoEventData* data) {
	LOG(Info) << "Hold at Target";

	// initialize local state variables
	bool hold_target_time_reached = false;
	Time st_enter_time = timer_.get_elapsed_time();

	// enter the control loop
	while (!hold_target_time_reached && !menu_ && !exit_program_ && !manual_stop_ && !auto_stop_) {
		
		// check for hold time reached
		hold_target_time_reached = check_wait_time_reached(hold_target_time_, st_enter_time, timer_.get_elapsed_time());	

		// check if return to main menu selected in Unity
		game_.set_experiment_conditions(hand_num_, dof_, num_classes_, condition_, menu_);

		// check for program exit command from user
		check_exit_program();

		// wait for the next clock cycle
		timer_.wait();
	}

	// transition to next state from "HOLD TARGET"
	if (auto_stop_ || manual_stop_) {
		event(ST_FAULT_STOP);
	}
	else if (exit_program_) {
		event(ST_STOP);
	}
	else if (menu_) {
		event(ST_WAIT_FOR_GUI);
	}
	else if (hold_target_time_reached) {
		event(ST_TO_CENTER);
	}
	else {
		LOG(Error) << "State transition undefined. Going to ST_FAULT_STOP.";
		event(ST_FAULT_STOP);
	}
}



//-----------------------------------------------------------------------------
// "FINISH EXPERIMENT" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRealTimeControlNoHardware::sf_finish(const NoEventData* data) {
	LOG(Info) << "Finishing Experiment";

	// intialize local state variables
	std::vector<double> lda_training_complete = { 0.0 };
	bool python_return = false;
	bool cv_score_achieved = true;

	// save data
	save_log_data();

	if (is_cal()) {
		save_emg_active_detector();
	}

	if (is_training()) {

		// send file location to python over melshare
		directory_share_.write_message(subject_dof_directory_);
		file_name_share_.write_message(training_data_filename_);

		// open LDA script in Python
		std::string system_command;
		system_command = "start " + program_directory_ + "\\" + "EMG_FS_LDA.py &";
		system(system_command.c_str());

		// wait for python to return results
		print("Waiting for Python to return training results...");

		while (!python_return && !menu_ && !exit_program_ && !manual_stop_ && !auto_stop_) {

			if (lda_training_complete[0] == 0.0) {
				lda_training_complete = lda_training_flag_.read_data();
			}
			else {

				if (!python_return) {
					python_return = true;

					// read in the results from the Cross-validation test in Python
					print("The cross validation results are:");
					std::vector<double> cross_eval_test(5);
					cross_eval_test = cv_results_.read_data();
					print(cross_eval_test);

					// check if results meet criteria for ending training
					for (int i = 0; i < cross_eval_test.size(); ++i) {
						if (cross_eval_test[i] < min_cv_score_) {
							cv_score_achieved = false;
						}
					}
					if (mean(cross_eval_test) < min_avg_cv_score_) {
						cv_score_achieved = false;
					}
					if (cv_score_achieved) {
						print("Training complete.");
					}
					else {
						print("Collect more training data.");
					}
				}
			}

			// check if return to main menu selected in Unity
			game_.set_experiment_conditions(hand_num_, dof_, num_classes_, condition_, menu_);

			// check for program exit command from user
			check_exit_program();

			// wait for the next clock cycle
			timer_.wait();
		}

		// wait to ensure data is logged
		sleep(seconds(3));
	}

	// wait for user input
	print("Press 'M' in Unity to return to the GUI main menu or press 'Escape' to stop the experiment");
	while (!menu_ && !exit_program_ && !manual_stop_ && !auto_stop_) {

		// read from Unity
		game_.set_experiment_conditions(hand_num_, dof_, num_classes_, condition_, menu_);

		// check for program exit command from user
		check_exit_program();

		// wait for the next clock cycle
		timer_.wait();
	}

	// transition to next state from "FINISH"
	if (auto_stop_ || manual_stop_) {
		event(ST_FAULT_STOP);
	}
	else if (exit_program_) {
		event(ST_STOP);
	}
	else if (menu_) {
		event(ST_WAIT_FOR_GUI);
	}
	else {
		LOG(Error) << "State transition undefined. Going to ST_FAULT_STOP.";
		event(ST_FAULT_STOP);
	}
}


//-----------------------------------------------------------------------------
// "STOP" STATE FUNCTION
//-----------------------------------------------------------------------------

void EmgRealTimeControlNoHardware::sf_stop(const NoEventData* data) {
	LOG(Info) << "Exiting Program";


}


//-----------------------------------------------------------------------------
// "FAULT STOP" STATE FUNCTION
//-----------------------------------------------------------------------------

void EmgRealTimeControlNoHardware::sf_fault_stop(const NoEventData* data) {
	LOG(Info) << "Program Stopped with Potential Fault";


	// ask user if want to save data
	print("The program was stopped manually or with a fault. Would you still like to save the data to the 'Error Report' directory (Y/N)?");
	Key key = Keyboard::wait_for_any_keys({ Key::Y, Key::N });
	switch (key) {
	case Key::Y:
		save_log_data();
		sleep(seconds(3)); // wait to ensure data is logged
		break;
	case Key::N:
		print("Data not saved.");
		sleep(seconds(0.5));
		break;
	}

	if (is_training()) {

		// ask user if want to save data
		print("The program was stopped manually or with a fault. Would you still like to save the training data to the subject directory (Y/N)?");
		Key key = Keyboard::wait_for_any_keys({ Key::Y, Key::N });
		switch (key) {
		case Key::Y:

			// write training data to file
			//if (!write_csv<double>(training_data_filename_, subject_dof_directory_, emg_training_data_)) {
			//}

			manual_stop_ = false;
			save_log_data();
			manual_stop_ = true;
			sleep(seconds(3)); // wait to ensure data is logged
			break;
		case Key::N:
			print("Data not saved.");
			sleep(seconds(0.5));
			break;
		}
	}
}



//-----------------------------------------------------------------------------
// EXPERIMENT SETUP/CONDITIONS UTILITY FUNCTIONS
//-----------------------------------------------------------------------------

std::vector<int> EmgRealTimeControlNoHardware::gen_rand_class_labels(int num_labels) const {
	std::random_device rd_seed; // random seed
	std::mt19937 rd_gen(rd_seed()); // random unsigned integer generator
	std::uniform_int_distribution<> class_labels_dist(1, num_classes_);
	std::vector<int> class_label_list(num_labels, 0);
	for (int i = 0; i < num_labels; ++i) {
		class_label_list[i] = class_labels_dist(rd_gen);
	}
	return class_label_list;
}

std::vector<int> EmgRealTimeControlNoHardware::rand_shuffle_class_labels(int num_labels_per_class) const {
	std::vector<int> class_label_list(num_labels_per_class * num_classes_, 0);
	for (int i = 0; i < num_labels_per_class * num_classes_; ++i) {
		class_label_list[i] = 1 + (i / num_labels_per_class);
	}
	std::srand(std::time(0));
	std::random_shuffle(class_label_list.begin(), class_label_list.end());
	return class_label_list;
}

bool EmgRealTimeControlNoHardware::is_single_dof() const {
	return dof_ < 4;
}

bool EmgRealTimeControlNoHardware::is_cal() const {
	return condition_ == 0;
}

bool EmgRealTimeControlNoHardware::is_training() const {
	return condition_ == 1;
}

bool EmgRealTimeControlNoHardware::is_testing() const {
	return condition_ == 2 || condition_ == 3;
}

bool EmgRealTimeControlNoHardware::is_blind() const {
	return condition_ == 2;
}

bool EmgRealTimeControlNoHardware::is_full() const {
	return condition_ == 3;
}

//-----------------------------------------------------------------------------
// EXPERIMENT CONTROL UTILITY FUNCTIONS
//-----------------------------------------------------------------------------

std::vector<double> EmgRealTimeControlNoHardware::get_target_position(int class_label) {
	std::vector<double> target_pos(MahiExoII::N_aj_, 0.0);
	try {
		if (is_single_dof()) {
			target_pos = single_dof_targets_.at(hand_num_).at(dof_).at(class_label - 1);
		}
		else {
			target_pos = multi_dof_targets_.at(hand_num_).at(dof_ - 4).at(class_label - 1);
		}
	}
	catch (const std::out_of_range& oor) {
		print("ERROR: Indexing out of range in function get_target_position.");
		auto_stop_ = true;
	}
	return target_pos;
}

bool EmgRealTimeControlNoHardware::check_wait_time_reached(Time wait_time, Time init_time, Time current_time) const {
	return (current_time - init_time) > wait_time;
}


//-----------------------------------------------------------------------------
// USER INPUT/OUTPUT UTILITY FUNCTIONS
//-----------------------------------------------------------------------------

void EmgRealTimeControlNoHardware::check_exit_program() {
	exit_program_ = (Keyboard::is_key_pressed(Key::Escape, false) | exit_program_);
}


//-----------------------------------------------------------------------------
// FILE READ/WRITE UTILITY FUNCTIONS
//-----------------------------------------------------------------------------

bool EmgRealTimeControlNoHardware::save_emg_active_detector() {
	bool write_status(false);

	// store in a csv-writable data type
	/*Eigen::MatrixXd calibration_data_eig(tkeo_W_t_.rows(), tkeo_W_t_.cols() + tkeo_w_0_.cols());
	calibration_data_eig << tkeo_W_t_, tkeo_w_0_;
	calibration_data_ = copy_eigmat_to_stdvecvec(calibration_data_eig);*/
	//if (active_detector_.size() != num_classes_) {
	//	LOG(Error) << "Mismatch between active detector and number of active classes expected.";
	//	auto_stop_ = true;
	//	return write_status;
	//}
	//std::vector<std::vector<double>> calibration_data;
	//for (std::size_t i = 0; i < active_detectors_.size(); ++i) {
	//	calibration_data.push_back(active_detectors_[i].get_bin_model());
	//}

	//// write to csv   
	//LOG(Info) << "Writing calibration data to " + subject_dof_directory_ + "\\" + emg_active_classifier_filename_;
	//write_status = write_csv<double>(emg_active_classifier_filename_, subject_dof_directory_, calibration_data);
	//if (!write_status)
	//	LOG(Warning) << "Failure to write calibration data.";
	return write_status;

	// write to csv with time stamp and additional identifying information
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}



bool EmgRealTimeControlNoHardware::load_emg_active_detector() {
	bool read_status(false);

	// read from csv
	//std::vector<std::vector<double>> calibration_data;
	//LOG(Info) << "Loading calibration data from " + subject_dof_directory_ + "\\" + emg_active_classifier_filename_;
	//read_status = read_csv<double>(emg_active_classifier_filename_, subject_dof_directory_, calibration_data);
	//if (!read_status) {
	//	LOG(Warning) << "Failure to read calibration data.";
	//	return read_status;
	//}

	//// parse csv data
	//if (calibration_data.size() != num_classes_ || calibration_data.size() != active_detectors_.size()) {
	//	LOG(Error) << "Mismatch between active detector and number of active classes expected.";
	//	auto_stop_ = true;
	//	return read_status;
	//}
	//for (std::size_t i = 0; i < calibration_data.size(); ++i) {
	//	active_detectors_[i].set_bin_classifier(calibration_data[i]);
	//}
	return read_status;
	//Eigen::MatrixXd calibration_data_eig = copy_stdvecvec_to_eigmat(calibration_data_);
	//tkeo_W_t_ = calibration_data_eig.block(0, 0, num_classes_ + 1, meii_.get_emg_channel_count());
	//tkeo_w_0_ = calibration_data_eig.block(0, meii_.get_emg_channel_count(), num_classes_ + 1, 1);
}

bool EmgRealTimeControlNoHardware::save_emg_dir_classifier() {
	bool write_status(false);

	//read_status = read_csv<double>(training_data_filename_, subject_dof_directory_, prev_emg_training_data_);
	//if (read_status)
	//	LOG(Info) << "Loading previous training data from " + subject_dof_directory_ + "\\" + training_data_filename_;
	return write_status;
}

bool EmgRealTimeControlNoHardware::load_emg_dir_classifier() {
	bool read_status(false);

	//std::vector<std::vector<double>> emg_dir_classifier;
	//LOG(Info) << "Loading classifier from " + subject_dof_directory_ + "\\" + emg_dir_classifier_filename_;
	//read_status = read_csv<double>(emg_dir_classifier_filename_, subject_dof_directory_, emg_dir_classifier);
	//if (!read_status) {
	//	LOG(Warning) << "Failure to read classifier data.";
	return read_status;
	//}

	//// convert classifier to Eigen type matrix
	//emg_dir_W_tt_ = copy_stdvecvec_to_eigmat(emg_dir_classifier);

}


void EmgRealTimeControlNoHardware::save_log_data() {

	std::string log_file_name;
	std::string directory;
	if (auto_stop_ || manual_stop_) {
		directory = project_directory_ + "\\" + "ErrorReport";
	}
	else {
		directory = subject_dof_directory_;
	}

	if (subject_number_ < 10) {
		log_file_name = "S0";
	}
	else {
		log_file_name = "S";
	}
	log_file_name += std::to_string(subject_number_) + "_" + str_dofs_[dof_] + "_" + str_conditions_[condition_];


	// save emg data log
	emg_log_.save_data("emg_data_log_" + log_file_name, directory, true);
	emg_log_.wait_for_save();
	emg_log_.clear_data();

	// save condition-specific data that was logged at each trial
	trial_log_.save_data(str_conditions_[condition_] + "_data_log_" + log_file_name, directory, true);
	trial_log_.wait_for_save();
	trial_log_.clear_data();

}

void EmgRealTimeControlNoHardware::init_robot_log() {
}

void EmgRealTimeControlNoHardware::init_emg_log() {
	std::vector<std::string> header;
	header.push_back("Time [s]");
	for (int i = 0; i < mes_.size(); ++i) {
		header.push_back("Ch. " + std::to_string(i));
	}
	for (int i = 0; i < mes_.size(); ++i) {
		header.push_back("Filt Ch. " + std::to_string(i));
	}
	header.push_back("State");
	emg_log_.set_header(header);
}

void EmgRealTimeControlNoHardware::init_results_log() {
	std::vector<std::string> header;
	header.push_back("Time [s]");
	if (is_cal()) {
		for (int i = 0; i < mes_.size(); ++i) {
			header.push_back("TK " + std::to_string(i));
		}
		header.push_back("Target");
	}
	else if (is_training()) {
		for (int i = 0; i < dir_classifier_.get_feature_dim(); ++i) {
			header.push_back("Feat. " + std::to_string(i));
		}
		header.push_back("Class Label");
	}
	else if (is_blind()) {
		for (int i = 0; i < dir_classifier_.get_feature_dim(); ++i) {
			header.push_back("Feat. " + std::to_string(i));
		}
		header.push_back("Class Label");
		header.push_back("Pred. Class Label");
		for (int k = 0; k < num_classes_; ++k) {
			header.push_back("Class " + std::to_string(k + 1) + "Post.");
		}
	}
	else if (is_full()) {
		for (int i = 0; i < dir_classifier_.get_feature_dim(); ++i) {
			header.push_back("Feat. " + std::to_string(i));
		}
		header.push_back("Class Label");
		header.push_back("Pred. Class Label");
		for (int k = 0; k < num_classes_; ++k) {
			header.push_back("Class " + std::to_string(k + 1) + "Post.");
		}
	}

	header.push_back("Subject");
	header.push_back("Hand");
	header.push_back("DoF");
	header.push_back("Condition");
	trial_log_.set_header(header);
}

void EmgRealTimeControlNoHardware::log_robot_row() {
}

void EmgRealTimeControlNoHardware::log_emg_row() {

	std::vector<double> row;
	row.push_back(timer_.get_elapsed_time().as_seconds());
	for (int i = 0; i < mes_.size(); ++i) {
		row.push_back(mes_.get_raw()[i]);
	}
	for (int i = 0; i < mes_.size(); ++i) {
		row.push_back(mes_.get_demean()[i]);
	}
	row.push_back((double)get_current_state());
	emg_log_.buffer(row);
}

void EmgRealTimeControlNoHardware::log_results_row() {
	print("Logging trial data.");

	std::vector<double> row;
	if (is_cal()) {
		//for (int j = 0; j < emg_calibration_data_buffer_.length_; ++j) {
		//    row.clear();
		//    row.push_back(timer_.get_elapsed_time().as_seconds());
		//    for (int i = 0; i < num_emg_channels_; ++i) {
		//        row.push_back(emg_calibration_data_buffer_.data_buffer_[i][j]);
		//    }
		//    row.push_back(class_label_sequence_[current_class_label_idx_]);
		//    row.push_back(subject_number_);
		//    row.push_back(hand_num_);
		//    row.push_back(dof_);
		//    row.push_back(condition_);
		//    trial_log_.buffer(row);
		//}
	}
	else if (is_training()) {
		//row.clear();
		//row.push_back(timer_.get_elapsed_time().as_seconds());
		//for (int i = 0; i < dir_classifier_.get_feature_dim(); ++i) {
		//	row.push_back(emg_feature_vec_[i]);
		//}
		//row.push_back(class_label_sequence_[current_class_label_idx_]);
		//row.push_back(subject_number_);
		//row.push_back(hand_num_);
		//row.push_back(dof_);
		//row.push_back(condition_);
		//trial_log_.buffer(row);
	}
	else if (is_blind()) {
		//row.clear();
		//row.push_back(timer_.get_elapsed_time().as_seconds());
		//for (int i = 0; i < mes_.size() * num_features_; ++i) {
		//	row.push_back(emg_feature_vec_[i]);
		//}
		//row.push_back(class_label_sequence_[current_class_label_idx_]);
		//row.push_back(pred_class_label_sequence_[current_class_label_idx_]);
		//for (int k = 0; k < num_classes_; ++k) {
		//	row.push_back(class_posteriors_[k]);
		//}
		//row.push_back(subject_number_);
		//row.push_back(hand_num_);
		//row.push_back(dof_);
		//row.push_back(condition_);
		//trial_log_.buffer(row);
	}
	else if (is_full()) {
		//row.clear();
		//row.push_back(timer_.get_elapsed_time().as_seconds());
		//for (int i = 0; i < mes_.size() * num_features_; ++i) {
		//	row.push_back(emg_feature_vec_[i]);
		//}
		//row.push_back(class_label_sequence_[current_class_label_idx_]);
		//row.push_back(pred_class_label_sequence_[current_class_label_idx_]);
		//for (int k = 0; k < num_classes_; ++k) {
		//	row.push_back(class_posteriors_[k]);
		//}
		//row.push_back(subject_number_);
		//row.push_back(hand_num_);
		//row.push_back(dof_);
		//row.push_back(condition_);
		//trial_log_.buffer(row);

	}
}


//-----------------------------------------------------------------------------
// DATA ANALYSIS UTILITY FUNCTIONS
//-----------------------------------------------------------------------------


Eigen::MatrixXd EmgRealTimeControlNoHardware::gen_confusion_mat(const std::vector<int>& actual_labels, const std::vector<int>& predicted_labels) const {
	size_t N = actual_labels.size();

	// assuming the class labels begin at 1
	int K = std::max(*std::max_element(actual_labels.begin(), actual_labels.end()), *std::max_element(predicted_labels.begin(), predicted_labels.end()));
	Eigen::MatrixXd conf_mat = Eigen::MatrixXd::Zero(K, K);

	if (predicted_labels.size() != N) {
		print("ERROR: Function gen_confusion_mat received input arguments with different lengths.");
		return conf_mat;
	}

	int min_val = std::min(*std::min_element(actual_labels.begin(), actual_labels.end()), *std::min_element(predicted_labels.begin(), predicted_labels.end()));
	if (min_val < 1) {
		print("ERROR: Function gen_confusion_mat received input arguments containing class labels less than 1.");
		return conf_mat;
	}

	for (size_t j = 0; j < K; ++j) {
		for (size_t i = 0; i < N; ++i) {
			if (actual_labels[i] == j + 1) {
				if (predicted_labels[i] == actual_labels[i]) {
					conf_mat(j, j) += 1;
				}
				else {
					conf_mat(j, predicted_labels[i] - 1) += 1;
				}
			}
		}
	}

	return conf_mat;

}

} // namespace meii