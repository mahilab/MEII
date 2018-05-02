#ifndef MEII_EMG_REAL_TIME_CONTROL_NO_HARDWARE_HPP
#define MEII_EMG_REAL_TIME_CONTROL_NO_HARDWARE_HPP

#include <MEII/MahiExoII/MahiExoII.hpp>
#include <MEII/Unity/UnityEmgRtc.hpp>
#include <MEL/Utility/StateMachine.hpp>
#include <MEL/Communications/Windows/MelShare.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Core/Time.hpp>
#include <MEL/Utility/RingBuffer.hpp>
#include <MEL/Logging/DataLogger.hpp>
#include <MEL/Daq/Daq.hpp>
#include <MEL/Utility/Console.hpp>
#include <MEII/EMG/MesArray.hpp>
#include <MEII/Classification/EmgActiveEnsClassifier.hpp>
#include <MEII/Classification/EmgDirClassifier.hpp>
#include <MEII/Utility/VirtualInput.hpp>
#include <Eigen/Dense>

using namespace mel;

namespace meii {

	class EmgRealTimeControlNoHardwareData : public EventData {

	public:

	};

	class EmgRealTimeControlNoHardware : public StateMachine {

	public:

		/// Constructor
		EmgRealTimeControlNoHardware(ctrl_bool& manual_stop, std::vector<uint32> input_channel_numbers);

	private:

		//-------------------------------------------------------------------------
		// STATE MACHINE SETUP
		//-------------------------------------------------------------------------

		// STATES
		enum States {
			ST_WAIT_FOR_GUI,
			ST_INIT,
			ST_BACKDRIVE,
			ST_INIT_RPS,
			ST_TO_CENTER,
			ST_HOLD_CENTER,
			ST_HOLD_FOR_INPUT,
			ST_CALIBRATION,
			ST_TRAINING,
			ST_PRESENT_TARGET,
			ST_TO_TARGET,
			ST_HOLD_TARGET,
			ST_FINISH,
			ST_STOP,
			ST_FAULT_STOP,
			ST_NUM_STATES
		};

		// STATE FUNCTIONS
		void sf_wait_for_gui(const NoEventData*);
		StateAction<EmgRealTimeControlNoHardware, NoEventData, &EmgRealTimeControlNoHardware::sf_wait_for_gui> sa_wait_for_gui;

		void sf_init(const NoEventData*);
		StateAction<EmgRealTimeControlNoHardware, NoEventData, &EmgRealTimeControlNoHardware::sf_init> sa_init;

		void sf_backdrive(const NoEventData*);
		StateAction<EmgRealTimeControlNoHardware, NoEventData, &EmgRealTimeControlNoHardware::sf_backdrive> sa_backdrive;

		void sf_init_rps(const NoEventData*);
		StateAction<EmgRealTimeControlNoHardware, NoEventData, &EmgRealTimeControlNoHardware::sf_init_rps> sa_init_rps;

		void sf_to_center(const NoEventData*);
		StateAction<EmgRealTimeControlNoHardware, NoEventData, &EmgRealTimeControlNoHardware::sf_to_center> sa_to_center;

		void sf_hold_center(const NoEventData*);
		StateAction<EmgRealTimeControlNoHardware, NoEventData, &EmgRealTimeControlNoHardware::sf_hold_center> sa_hold_center;

		void sf_hold_for_input(const NoEventData*);
		StateAction<EmgRealTimeControlNoHardware, NoEventData, &EmgRealTimeControlNoHardware::sf_hold_for_input> sa_hold_for_input;

		void sf_calibration(const NoEventData*);
		StateAction<EmgRealTimeControlNoHardware, NoEventData, &EmgRealTimeControlNoHardware::sf_calibration> sa_calibration;

		void sf_training(const NoEventData*);
		StateAction<EmgRealTimeControlNoHardware, NoEventData, &EmgRealTimeControlNoHardware::sf_training> sa_training;

		void sf_present_target(const NoEventData*);
		StateAction<EmgRealTimeControlNoHardware, NoEventData, &EmgRealTimeControlNoHardware::sf_present_target> sa_present_target;

		void sf_to_target(const NoEventData*);
		StateAction<EmgRealTimeControlNoHardware, NoEventData, &EmgRealTimeControlNoHardware::sf_to_target> sa_to_target;

		void sf_hold_target(const NoEventData*);
		StateAction<EmgRealTimeControlNoHardware, NoEventData, &EmgRealTimeControlNoHardware::sf_hold_target> sa_hold_target;

		void sf_finish(const NoEventData*);
		StateAction<EmgRealTimeControlNoHardware, NoEventData, &EmgRealTimeControlNoHardware::sf_finish> sa_finish;

		void sf_stop(const NoEventData*);
		StateAction<EmgRealTimeControlNoHardware, NoEventData, &EmgRealTimeControlNoHardware::sf_stop> sa_stop;

		void sf_fault_stop(const NoEventData*);
		StateAction<EmgRealTimeControlNoHardware, NoEventData, &EmgRealTimeControlNoHardware::sf_fault_stop> sa_fault_stop;

		// STATE MAP
		virtual const StateMapRow* get_state_map() {
			static const StateMapRow STATE_MAP[] = {
				&sa_wait_for_gui,
				&sa_init,
				&sa_backdrive,
				&sa_init_rps,
				&sa_to_center,
				&sa_hold_center,
				&sa_hold_for_input,
				&sa_calibration,
				&sa_training,
				&sa_present_target,
				&sa_to_target,
				&sa_hold_target,
				&sa_finish,
				&sa_stop,
				&sa_fault_stop,
			};
			return &STATE_MAP[0];
		}

	private:

		// EXPERIMENT SETUP/CONDITIONS UTILITY FUNCTIONS
		std::vector<int> gen_rand_class_labels(int num_labels) const;
		std::vector<int> rand_shuffle_class_labels(int num_labels_per_class) const;
		bool is_single_dof() const;
		bool is_cal() const;
		bool is_training() const;
		bool is_testing() const;
		bool is_blind() const;
		bool is_full() const;

		// EXPERIMENT CONTROL UTILITY FUNCTIONS
		std::vector<double> get_target_position(int class_label);
		bool check_wait_time_reached(Time wait_time, Time init_time, Time current_time) const;

		// USER INPUT/OUTPUT UTILITY FUNCTIONS
		void check_exit_program();

		// FILE READ/WRITE UTILITY FUNCTIONS
		bool save_emg_active_detector();
		bool load_emg_active_detector();
		bool save_emg_dir_classifier();
		bool load_emg_dir_classifier();
		void save_log_data();
		void init_robot_log();
		void init_emg_log();
		void init_results_log();
		void log_robot_row();
		void log_emg_row();
		void log_results_row();

		// DATA ANALYSIS UTILITY FUNCTIONS
		Eigen::MatrixXd gen_confusion_mat(const std::vector<int>& actual_labels, const std::vector<int>& predicted_labels) const;

	private:

		// SUBJECT/CONDITION
		std::size_t subject_number_ = 0; ///< subject number used for saving data, set to 0 when not running official experiment
		std::size_t hand_num_ = 1; ///< 0 or 1 for Left or Right arm of the user, respectively
		std::size_t dof_; ///< 0-3 is single-dof; 4-5 is multi-dof
		std::size_t num_classes_; ///< number of classes based on the dof, 2 for single, 4 for multi
		std::size_t condition_; ///< 0 = calibration; 1 = training; 2 = blind testing; 3 = full testing

		// TIMING 
		Time Ts_; ///< sample period for the DAQ
		Timer timer_; ///< timer to control the loop rate for closed-loop-control of the robot and signal processing

		// Virtual MES Array
		VirtualInput vi_module_;
		MesArray mes_;

		// UNITY GAME
		UnityEmgRtc game_; ///< instance of the custom Unity game made for the EmgRealTimeControl experiment 

		// STATE TRANSITION EVENT VARIABLES
		ctrl_bool& manual_stop_; ///< initialize to false; becomes true when user presses 'Ctrl+C' at the console
		bool auto_stop_; ///< initialize to false; becomes true when any robot limits are exceeded or any other programmed error catch occurs
		bool exit_program_; ///< initialize to false; becomes true when user presses 'Escape' at the console
		bool menu_; ///< initialize to true; remains true when GUI is at the main menu; otherwise false
		bool end_of_label_sequence_; ///< initialize to true; set to false when new sequence of labels is generated; returns to true when end of sequence reached
		bool active_detector_computed_; ///< initialize to false; becomes true when the calibration has been successfully completed, or active detector has been loaded

		// EXPERIMENT TIMING PARAMETERS
		Time init_backdrive_time_ = seconds(2.0); ///< time to be in backdrive mode initially
		Time hold_center_time_ = seconds(1.0); ///< time to hold at center target
		Time hold_target_time_ = seconds(1.0); ///< time to hold at target
		Time wait_mvc_time_ = seconds(0.5); ///< time to wait during MVC
		Time detection_expire_time_ = seconds(10.0); ///< time after which failure to detect active state is marked as misclassification

		// REAL-TIME CLASSIFIERS
		EmgActiveEnsClassifier active_detector_; ///< vector of real-time classifiers that act as one ensemble classifier for detecting when the user is active in a certain DoF
		EmgDirClassifier dir_classifier_;

		// STRING NAMES
		std::vector<std::string> str_conditions_long_ = { "Calibration", "Training", "Blind Testing", "Full Testing" };
		std::vector<std::string> str_conditions_ = { "cal", "trng", "blind", "full" };
		std::vector<std::string> str_dofs_long_ = { "Elbow F/E Single-DoF", "Forearm P/S Single-Dof", "Wrist F/E Single-DoF", "Wrist R/U Single-DoF", "Elbow F/E & Forearm P/S Multi-DoF", "Wrist F/E & Wrist R/U Multi-DoF" };
		std::vector<std::string> str_dofs_ = { "EFE", "FPS", "WFE", "WRU", "ELFM", "WMLT" };
		std::vector<std::string> str_states_ = { "ST_WAIT_FOR_GUI", "ST_INIT", "ST_BACKDRIVE", "ST_INIT_RPS", "ST_TO_CENTER", "ST_HOLD_CENTER", "ST_HOLD_FOR_INPUT", "ST_PRESENT_TARGET", "ST_TO_TARGET", "ST_HOLD_TARGET", "ST_FINISH", "ST_STOP", "ST_FAULT_STOP" };

		// FILE NAMES & DIRECTORIES
		std::string program_directory_ = "C:\\Git\\MEII\\python";
		std::string project_directory_ = "C:\\Git\\MEII\\EmgRTControl";
		std::string subject_directory_;
		std::string subject_dof_directory_;
		std::string emg_active_classifier_filename_;
		std::string training_data_filename_;
		std::string emg_dir_classifier_filename_;

		// PREDEFINED TARGETS
		const std::vector<double> center_pos_ = { -35 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD,  0.09 }; // anatomical joint positions
		const std::vector<std::vector<std::vector<std::vector<double>>>> single_dof_targets_ =     // left arm
		{ { { { -5 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 },{ -65 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 } }, // elbow f/e : up, down
		{ { -35 * DEG2RAD, -30 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 },{ -35 * DEG2RAD,  30 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 } }, // forearm p/s : right, left
		{ { -35 * DEG2RAD,   0 * DEG2RAD, -15 * DEG2RAD,   0 * DEG2RAD,  0.09 },{ -35 * DEG2RAD,   0 * DEG2RAD,  15 * DEG2RAD,   0 * DEG2RAD,  0.09 } }, // wrist f/e : right, left
		{ { -35 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  15 * DEG2RAD,  0.09 },{ -35 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD, -15 * DEG2RAD,  0.09 } } }, // wrist r/u : up, down

																																						   // right arm
		{ { { -5 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 },{ -65 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 } }, // elbow f/e : up, down
		{ { -35 * DEG2RAD,  30 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 },{ -35 * DEG2RAD, -30 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 } }, // forearm p/s : left, right
		{ { -35 * DEG2RAD,   0 * DEG2RAD,  15 * DEG2RAD,   0 * DEG2RAD,  0.09 },{ -35 * DEG2RAD,   0 * DEG2RAD, -15 * DEG2RAD,   0 * DEG2RAD,  0.09 } }, // wrist f/e : left, right
		{ { -35 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  15 * DEG2RAD,  0.09 },{ -35 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD, -15 * DEG2RAD,  0.09 } } } }; // wrist r/u : up, down

		const std::vector<std::vector<std::vector<std::vector<double>>>>  multi_dof_targets_ =     // left arm
		{ { { { -5 * DEG2RAD, -30 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 },{ -5 * DEG2RAD,  30 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 },{ -65 * DEG2RAD, -30 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 },{ -65 * DEG2RAD,  30 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 } }, // elbow f/e & forearm p/s
		{ { -35 * DEG2RAD,   0 * DEG2RAD, -15 * DEG2RAD,  15 * DEG2RAD,  0.09 },{ -35 * DEG2RAD,   0 * DEG2RAD,  15 * DEG2RAD,  15 * DEG2RAD,  0.09 },{ -35 * DEG2RAD,   0 * DEG2RAD, -15 * DEG2RAD, -15 * DEG2RAD,  0.09 },{ -35 * DEG2RAD,   0 * DEG2RAD,  15 * DEG2RAD, -15 * DEG2RAD,  0.09 } } }, // wrist f/e & wrist r/u

																																																																									   // right arm
		{ { { -5 * DEG2RAD,  30 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 },{ -5 * DEG2RAD, -30 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 },{ -65 * DEG2RAD,  30 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 },{ -65 * DEG2RAD, -30 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 } }, // elbow f/e & forearm p/s
		{ { -35 * DEG2RAD,   0 * DEG2RAD,  15 * DEG2RAD,  15 * DEG2RAD,  0.09 },{ -35 * DEG2RAD,   0 * DEG2RAD, -15 * DEG2RAD,  15 * DEG2RAD,  0.09 },{ -35 * DEG2RAD,   0 * DEG2RAD,  15 * DEG2RAD, -15 * DEG2RAD,  0.09 },{ -35 * DEG2RAD,   0 * DEG2RAD, -15 * DEG2RAD, -15 * DEG2RAD,  0.09 } } } }; // wrist f/e & wrist r/u



		// TRAINING PARAMETERS
		double min_cv_score_ = 0.85;
		double min_avg_cv_score_ = 0.95;

		// CLASSIFICATION
		std::vector<int> class_label_sequence_; ///< sequence of class labels to be presented: 1-2 for single-DoF and 1-4 for multi-DoF
		int current_class_label_idx_;
		std::vector<int> pred_class_label_sequence_;

		// TESTING PARAMETERS
		int num_blind_testing_trials_ = 10;
		int num_full_testing_trials_ = 5;

		// PYTHON COMMUNICATION
		MelShare directory_share_;
		MelShare file_name_share_;
		MelShare cv_results_;
		MelShare lda_training_flag_;

		// DATA LOG
		DataLogger emg_log_;
		DataLogger trial_log_;
		DataLogger debug_log_;


	};

} // namespace meii

#endif // MEII_EMG_REAL_TIME_CONTROL_NO_HARDWARE_HPP