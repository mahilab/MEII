#include "EmgRTControl/EmgRTControl.hpp"
#include "MEL/Utility/Windows/Keyboard.hpp"
#include "MEL/Utility/Console.hpp"
#include "MEL/Utility/System.hpp"
#include "MEL/Core/Motor.hpp"
#include "MEL/Math/Functions.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <random>
#include <numeric>
#include <stdexcept>



using namespace mel;

EmgRTControl::EmgRTControl(MahiExoIIEmg& meii, Daq& daq, Watchdog& watchdog) :
    StateMachine(13), 
    meii_(meii),
    daq_(daq),
    watchdog_(watchdog),
    timer_(milliseconds(1), Timer::Hybrid),
    scene_num_share_(50000,50001,"10.98.64.109",true),
    viz_target_num_share_(50002, 50003, "10.98.64.109", false),
    force_mag_share_(50004, 50005, "10.98.64.109", false),
    hand_select_(50006, 50007, "10.98.64.109", false),
    pos_net_(50008, 50009, "10.98.64.109", false),
    state_net_(50010, 50011, "10.98.64.109", false),
    directory_share_("file_path"),
    file_name_share_("file_name"),
    cv_results_("cv_results"),
    lda_training_flag_("lda_training_flag"),
    pos_share_("pos_share"),
    vel_share_("vel_share"),
    emg_share_("emg_share"),
    torque_share_("torque_share"),
    robot_log_({ "Time [s]", "MEII EFE Position [rad]", "MEII EFE Velocity [rad/s]", "MEII EFE Commanded Torque [Nm]",
        "MEII FPS Position [rad]", "MEII FPS Velocity [rad/s]", "MEII FPS Commanded Torque [Nm]",
        "MEII RPS L1 Position [m]", "MEII RPS L1 Velocity [m/s]", "MEII RPS L1 Force [N]",
        "MEII RPS L2 Position [m]", "MEII RPS L2 Velocity [m/s]", "MEII RPS L2 Force [N]",
        "MEII RPS L3 Position [m]", "MEII RPS L3 Velocity [m/s]", "MEII RPS L3 Force [N]", "State" }),
    emg_log_({"Time [s]", "Ch. 0", "Ch. 1", "Ch. 2", "Ch. 3", "Ch. 4", "Ch. 5", "Ch. 6", "Ch. 7",
        "Filt. Ch. 0", "Filt. Ch. 1", "Filt. Ch. 2", "Filt. Ch. 3", "Filt. Ch. 4", "Filt. Ch. 5", "Filt. Ch. 6", "Filt. Ch. 7", "State"}),
    //robot_log_({ "Time [s]", "MEII Joint 0 Encoder Count", "MEII Joint 0 Encoder Rate", "MEII Joint 0 Motor Commanded Current [A]", "MEII Joint 0 Motor Limited Current [A]",
    //    "MEII Joint 1 Encoder Count", "MEII Joint 1 Encoder Rate", "MEII Joint 1 Motor Commanded Current [A]", "MEII Joint 1 Motor Limited Current [A]",
    //    "MEII Joint 2 Encoder Count", "MEII Joint 2 Encoder Rate", "MEII Joint 2 Motor Commanded Current [A]", "MEII Joint 2 Motor Limited Current [A]",
    //    "MEII Joint 3 Encoder Count", "MEII Joint 3 Encoder Rate", "MEII Joint 3 Motor Commanded Current [A]", "MEII Joint 3 Motor Limited Current [A]",
    //    "MEII Joint 4 Encoder Count", "MEII Joint 4 Encoder Rate", "MEII Joint 4 Motor Commanded Current [A]", "MEII Joint 4 Motor Limited Current [A]",
    //    "MEII EFE Position [rad]", "MEII EFE Velocity [rad/s]", "MEII EFE Commanded Torque [Nm]",
    //    "MEII FPS Position [rad]", "MEII FPS Velocity [rad/s]", "MEII FPS Commanded Torque [Nm]",
    //    "MEII RPS Theta1 Position [rad]", "MEII RPS Theta2 Position [rad]", "MEII RPS Theta3 Position [rad]",
    //    "MEII RPS L1 Position [m]", "MEII RPS L2 Position [m]", "MEII RPS L3 Position [m]",
    //    "MEII RPS Alpha Position [rad]", "MEII RPS Beta Position [rad]", "MEII RPS Gamma Position [rad]",
    //    "MEII RPS X Position [m]", "MEII RPS Y Position [m]", "MEII RPS Z Position [m]",
    //    "MEII RPS Theta1 Velocity [rad/s]","MEII RPS Theta2 Velocity [rad/s]", "MEII RPS Theta3 Velocity [rad/s]",
    //    "MEII RPS L1 Velocity [m/s]", "MEII RPS L2 Velocity [m/s]", "MEII RPS L3 Velocity [m/s]",
    //    "MEII RPS Alpha Velocity [rad/s]", "MEII RPS Beta Velocity [rad/s]", "MEII RPS Gamma Velocity [rad/s]",
    //    "MEII RPS X Velocity [m/s]", "MEII RPS Y Velocity [m/s]", "MEII RPS Z Velocity [m/s]",
    //    "MEII RPS L1 Force [N]", "MEII RPS L2 Force [N]", "MEII RPS L3 Force [N]",
    //    "MEII RPS Alpha Torque [Nm]", "MEII RPS Beta Torque [Nm]", "MEII RPS X Force [N]" }),
    tkeo_stat_buffer_(tkeo_stat_buffer_size_)
{ }   


///-----------------------------------------------------------------------------
/// "WAIT FOR GUI" STATE FUNCTION
///-----------------------------------------------------------------------------
void EmgRTControl::sf_wait_for_gui(const NoEventData* data) {
    print("Waiting for Gui Input");

    /// start the clock
    timer_.restart();

    /// initialize local state variables
    bool scene_selected = false;
    bool exit_program = false;

    if (emg_signal_check_ || find_neutral_position_) {
        SCENE_NUM_[0] = 2;
        scene_selected = true;
    }
    else {
        /// launch game
        game.launch();

        /// prompt user for input
        print("Select a mode in Unity to begin the experiment, or press 'ENTER' to stop the experiment.");
    }  

    /// wait for gui input
    while (!scene_selected && !exit_program && !manual_stop_ && !auto_stop_) {
      

        /// check if scene selected in Unity
        if (scene_num_share_.receive_message() == "scene_change") {
            SCENE_NUM_ = scene_num_share_.receive_data();
            if (SCENE_NUM_[0] != 0) {
                scene_selected = true;
            }
        }


        /// check for end of experiment
        if (Keyboard::is_key_pressed(Key::Enter)) {
            exit_program = true;
        }

        /// check for manual stop input
        check_manual_stop();

        /// wait for the next clock cycle
        timer_.wait();
    }

    /// transition to next state from "WAIT FOR GUI"
    if (auto_stop_ || manual_stop_) {
        event(ST_FAULT_STOP);
    }
    else if (exit_program) {
        event(ST_STOP);
    }
    else if (scene_selected) {
        event(ST_INIT);
    }
    else {
        print("ERROR: State transition undefined. Going to ST_FAULT_STOP.");
        event(ST_FAULT_STOP);
    }
}


///-----------------------------------------------------------------------------
/// "INITIALIZATION" STATE FUNCTION
///-----------------------------------------------------------------------------
void EmgRTControl::sf_init(const NoEventData* data) {

    /// set experiment condition variables dof_ and condition_ based on scene number
    set_experiment_conditions(SCENE_NUM_[0]);
    print("Initializing for " + str_conditions_long_[condition_] + " of " + str_dofs_long_[dof_]);

    /// reset global experiment variables
    end_of_label_sequence_ = true;
    class_label_sequence_.clear();
    pred_class_label_sequence_.clear();
    current_class_label_idx_ = -1;
    viz_target_num_[0] = 0;
    emg_voltages_ = std::vector<double>(meii_.N_emg_, 0.0);
    filtered_emg_voltages_ = std::vector<double>(meii_.N_emg_, 0.0);
    class_posteriors_ = std::vector<double>(num_classes_, 0.0);
    emg_training_data_.clear();
    prev_emg_training_data_.clear();
    calibration_data_.clear();
    tkeo_rest_data_.clear();
    tkeo_active_data_.clear();
    tkeo_active_data_.resize(num_classes_);
    std::vector<double> init_flag = { 0.0 };
    lda_training_flag_.write_data(init_flag);

    /// create subject and DoF specific folder for data logging
    subject_directory_ = project_directory_ + "\\EMG_S";
    if (subject_number_ < 10) {
        subject_directory_ += "0";
    }
    subject_directory_ += std::to_string(subject_number_);
    subject_dof_directory_ = subject_directory_ + "\\" + str_dofs_[dof_];

    /// generate file names
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

    /// reset robot
    meii_.disable();

    /// initialize data loggers
    //meii_.init_robot_log();
    //init_emg_log();
    //init_trial_log();
    //init_debug_log();

    /// initialize MEII EMG signal monitors
    if (!virtual_emg_) {
        meii_.emg_signal_monitor_ = true;
        meii_.tko_.tkeo_signal_monitor_ = true;
    }
    else {
        meii_.emg_signal_monitor_ = false;
        meii_.tko_.tkeo_signal_monitor_ = false;
    }

    /// enable MEII
    meii_.enable();  

    /// confirm start of experiment
    print("\nRunning EMG Real-Time Control ... ");

    /// write to Unity
    hand_select_.send_message("hand_flag");
    hand_select_.send_message(hand_def_);

    /// start the watchdog
    watchdog_.start();

    /// transition to next state from "INITIALIZATION"
    if (auto_stop_ || manual_stop_) {
        event(ST_FAULT_STOP);
    }
    else {
        event(ST_BACKDRIVE);
    }
}


///-----------------------------------------------------------------------------
/// "BACKDRIVE" STATE FUNCTION
///-----------------------------------------------------------------------------
void EmgRTControl::sf_backdrive(const NoEventData* data) {
    print("Robot Backdrivable"); 

    /// initialize local state variables
    bool init_backdrive_time_reached = false;
    bool exit_program = false;
    bool neutral_position_selected = false;
    Time st_enter_time = timer_.get_elapsed_time();
    std::vector<double> command_torques(meii_.N_aj_, 0.0);
    std::vector<double> filtered_emg_voltages(meii_.N_emg_);
    Key key;

    if (emg_signal_check_) {
        /// prompt user for input
        print("Press 'ENTER' to end the EMG signal check mode.");
    }

    if (find_neutral_position_) {
        /// prompt user for input
        print("Press 'ENTER' to capture a new wrist neutral position.");
    }

    /// enter the control loop
    while (!init_backdrive_time_reached && !exit_program && !manual_stop_ && !auto_stop_) {

        /// read and reload DAQs
        watchdog_.kick();
        daq_.update_input();

        if (!virtual_exo_) {

            /// update robot kinematics
            meii_.update_kinematics();

            /// check joint limits
            if (meii_.check_all_joint_limits()) {
                auto_stop_ = true;
                break;
            }

            /// set zero torques
            meii_.set_joint_torques(command_torques);

            if (scope_mode_) {

                /// write kinematics and motor commands to MelScope
                pos_share_.write_data(meii_.get_anatomical_joint_positions());
                vel_share_.write_data(meii_.get_anatomical_joint_velocities());
                torque_share_.write_data(command_torques);
            }
        } 
    
        if (!emg_signal_check_ && !find_neutral_position_) {

            /// check for init transparent time reached
            init_backdrive_time_reached = check_wait_time_reached(init_backdrive_time_, st_enter_time, timer_.get_elapsed_time());
        }

        if (emg_signal_check_) {

            /// get measured emg voltages
            emg_voltages_ = meii_.get_emg_voltages();
            meii_.butter_hp_.filter(emg_voltages_, filtered_emg_voltages);
            emg_share_.write_data(filtered_emg_voltages);

            /// check for user input
            if (Keyboard::is_key_pressed(Key::Enter)) {
                exit_program = true;
            }
        }

        if (find_neutral_position_) {

            /// check for user input
            if (!neutral_position_selected) {
                if (Keyboard::is_key_pressed(Key::Enter)) {
                    neutral_position_selected = true;
                    print("Do you want to use this wrist position (Y/N)?");
                }
            }
            else {
                key = Keyboard::are_any_keys_pressed({ Key::Y, Key::N });
                switch (key) {
                case Key::Y:
                    print("Elbow F/E angle = " + std::to_string(meii_.get_anatomical_joint_position(0) * RAD2DEG) + " DEG");
                    print("Forearm P/S angle = " + std::to_string(meii_.get_anatomical_joint_position(1) * RAD2DEG) + " DEG");
                    print("Wrist F/E angle = " + std::to_string(meii_.get_anatomical_joint_position(2) * RAD2DEG) + " DEG");
                    print("Wrist R/U angle = " + std::to_string(meii_.get_anatomical_joint_position(3) * RAD2DEG) + " DEG");
                    print("Wrist platform height = " + std::to_string(meii_.get_anatomical_joint_position(4)) + " M");
                    exit_program = true;
                    break;
                case Key::N:
                    neutral_position_selected = false;
                    print("Press 'ENTER' to capture a new neutral wrist position.");
                    break;
                }
            }
        }

        /// check for manual stop input
        check_manual_stop();

        /// log data
        log_robot_row();

        if (!virtual_exo_) {

            /// write to daq
            daq_.update_output();
        }

        /// wait for the next clock cycle
        timer_.wait();
    }

    /// transition to next state from "BACKDRIVE"
    if (auto_stop_ || manual_stop_) {
        event(ST_FAULT_STOP);
    }
    else if (exit_program) {
        event(ST_STOP);
    }
    else if (init_backdrive_time_reached) {
        event(ST_INIT_RPS);
    }
    else {
        print("ERROR: State transition undefined. Going to ST_FAULT_STOP.");
        event(ST_FAULT_STOP);
    }
}


///-----------------------------------------------------------------------------
/// "INITIALIZE RPS MECHANISM" STATE FUNCTION
///-----------------------------------------------------------------------------
void EmgRTControl::sf_init_rps(const NoEventData* data) {
    print("Initialize RPS Mechanism");

    /// initialize local state variables
    bool rps_init = false;
    std::vector<double> rps_command_torques(meii_.N_qs_, 0.0);
    std::vector<double> command_torques(meii_.N_rj_, 0.0);

    /// initialize rps initialization position controller
    meii_.set_rps_control_mode(0);
    meii_.update_kinematics();
    meii_.rps_init_par_ref_.start(meii_.get_wrist_parallel_positions(), timer_.get_elapsed_time());

    /// enter the control loop
    while (!rps_init && !manual_stop_ && !auto_stop_) {

        /// read and reload DAQs
        watchdog_.kick();
        daq_.update_input();

        if (!virtual_exo_) {

            /// update robot kinematics
            meii_.update_kinematics();

            /// check joint limits
            if (meii_.check_all_joint_limits()) {
                auto_stop_ = true;
                break;
            }

            /// set zero torque for elbow and forearm joints (joints 0 and 1)           
            meii_[0].set_torque(0.0);
            meii_[1].set_torque(0.0);

            /// run rps position control
            rps_command_torques = meii_.set_rps_pos_ctrl_torques(meii_.rps_init_par_ref_, timer_.get_elapsed_time());
            std::copy(rps_command_torques.begin(), rps_command_torques.end(), command_torques.begin() + 2);

            if (scope_mode_) {

                /// write kinematics and motor commands to MelScope
                pos_share_.write_data(meii_.get_anatomical_joint_positions());
                vel_share_.write_data(meii_.get_anatomical_joint_velocities());
                torque_share_.write_data(command_torques);
            }
        }       

        if (!virtual_exo_) {

            /// check for rps mechanism in intialization position
            rps_init = meii_.check_rps_init();
        }
        else {
            rps_init = true;
        }

        /// log data
        log_robot_row();

        /// check for manual stop input
        check_manual_stop();

        if (!virtual_exo_) {

            /// write to daq
            daq_.update_output();
        }

        /// wait for the next clock cycle
        timer_.wait();
    }

    /// stop the rps intialization position controller
    meii_.rps_init_par_ref_.stop();

    /// transition to next state from "INITIALIZE RPS MECHANISM"
    if (auto_stop_ || manual_stop_) {
        event(ST_FAULT_STOP);
    }
    else if (rps_init) {
        event(ST_TO_CENTER);
    }
    else {
        print("ERROR: State transition undefined. Going to ST_FAULT_STOP.");
        event(ST_FAULT_STOP);
    }
}



///-----------------------------------------------------------------------------
/// "GO TO CENTER" STATE FUNCTION
///-----------------------------------------------------------------------------
void EmgRTControl::sf_to_center(const NoEventData* data) {
    print("Go to Center");
    
    /// initialize local state variables
    bool target_reached = false;
    std::vector<double> command_torques(meii_.N_aj_, 0.0);

    /// initialize rps position controller mode and reference
    meii_.set_rps_control_mode(1);
    if (meii_.anat_ref_.is_started()) {
        meii_.anat_ref_.set_ref(center_pos_, timer_.get_elapsed_time());
    }
    else {
        meii_.anat_ref_.start(center_pos_, meii_.get_anatomical_joint_positions(), timer_.get_elapsed_time());
    }

    /// write to Unity
    viz_target_num_ = { 0 };
    viz_target_num_share_.send_message("target_flag");
    viz_target_num_share_.send_data(viz_target_num_);

    /// enter the control loop
    while (!target_reached && !manual_stop_ && !auto_stop_) {

        /// read and reload DAQs
        watchdog_.kick();
        daq_.update_input();

        if (!virtual_exo_) {

            /// update robot kinematics
            meii_.update_kinematics();

            /// check joint limits
            if (meii_.check_all_joint_limits()) {
                auto_stop_ = true;
                break;
            }

            /// run position control
            command_torques = meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, timer_.get_elapsed_time());

            if (scope_mode_) {

                /// write kinematics and motor commands to MelScope
                pos_share_.write_data(meii_.get_anatomical_joint_positions());
                vel_share_.write_data(meii_.get_anatomical_joint_velocities());
                torque_share_.write_data(command_torques);
            }
        }

        if (!virtual_exo_) {

            /// check for target reached
            target_reached = meii_.check_neutral_anat_pos(center_pos_, { 1, 1, 1, 1, 0 });
        }
        else {
            target_reached = true;
        }

        /// log data
        log_robot_row();

        /// check for manual stop input
        check_manual_stop();

        if (!virtual_exo_) {

            /// write to daq
            daq_.update_output();
        }

        /// wait for the next clock cycle
        timer_.wait();
    }

    /// transition to next state from "GOT TO CENTER"
    if (auto_stop_ || manual_stop_) {
        event(ST_FAULT_STOP);
    }
    else if (target_reached) {
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
    else {
        print("ERROR: State transition undefined. Going to ST_FAULT_STOP.");
        event(ST_FAULT_STOP);
    }
}


//-----------------------------------------------------------------------------
/// "HOLD AT CENTER" STATE FUNCTION
///-----------------------------------------------------------------------------
void EmgRTControl::sf_hold_center(const NoEventData* data) {
    print("Hold at Center");

    /// initialize global variables   
    if (is_cal()) {
        hold_center_time_ = seconds(3.0);
        emg_calibration_data_buffer_.clear();
        meii_.butter_hp_.reset();   
        meii_.tko_.reset();
        meii_.tkeo_butter_lp_.reset();
    }
    else {
        hold_center_time_ = seconds(1.0);
        tkeo_stat_buffer_.clear();
        tkeo_stat_buffer_.resize(tkeo_stat_buffer_size_);
    }

    /// initialize local state variables
    bool hold_center_time_reached = false;
    bool rest_state_reached = false;
    Time st_enter_time = timer_.get_elapsed_time();
    std::vector<double> command_torques(meii_.N_aj_, 0.0);
    std::vector<double> tkeo_vec(meii_.N_emg_, 0.0);
    std::vector<double> filtered_tkeo_vec(meii_.N_emg_, 0.0);
    bool tkeo_buffer_full = false;
    emg_voltages_ = std::vector<double>(meii_.N_emg_, 0.0);
    filtered_emg_voltages_ = std::vector<double>(meii_.N_emg_, 0.0);

    /// write to Unity
    viz_target_num_ = { 0 };
    viz_target_num_share_.send_message("target_flag");
    viz_target_num_share_.send_data(viz_target_num_);

    /// enter the control loop
    while (!rest_state_reached && !manual_stop_ && !auto_stop_) {

        /// read and reload DAQs
        watchdog_.kick();
        daq_.update_input();

        if (!virtual_exo_) {

            /// update robot kinematics
            meii_.update_kinematics();

            /// check joint limits
            if (meii_.check_all_joint_limits()) {
                auto_stop_ = true;
                break;
            }

            /// run position control
            command_torques = meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, timer_.get_elapsed_time());

            if (scope_mode_) {

                /// write kinematics and motor commands to MelScope
                pos_share_.write_data(meii_.get_anatomical_joint_positions());
                vel_share_.write_data(meii_.get_anatomical_joint_velocities());
                torque_share_.write_data(command_torques);
            }
        }
      
        /// get measured emg voltages and log them
        emg_voltages_ = meii_.get_emg_voltages();
        meii_.butter_hp_.filter(emg_voltages_, filtered_emg_voltages_);
        log_emg_row();
        if (scope_mode_) {
            emg_share_.write_data(filtered_emg_voltages_);
        }

        /// compute the teager-kaiser metric online with rectification and filtering
        meii_.tko_.tkeo(filtered_emg_voltages_, tkeo_vec);
        std::for_each(tkeo_vec.begin(), tkeo_vec.end(), [](double x) { return std::abs(x); });
        meii_.tkeo_butter_lp_.filter(tkeo_vec, filtered_tkeo_vec);

        /// check for hold time reached
        hold_center_time_reached = check_wait_time_reached(hold_center_time_, st_enter_time, timer_.get_elapsed_time());

        if (is_cal()) {

            /// store calibration data in buffer
            emg_calibration_data_buffer_.push_back(filtered_tkeo_vec);

            if (hold_center_time_reached) {
                rest_state_reached = true;
            }
        }
        else {

            /// compute tkeo detector statistic and fill buffer
            tkeo_stat_ = tkeo_detector(filtered_tkeo_vec);

            if (tkeo_stat_ == 0 || tkeo_stat_ == 1) {
                //print("WARNING: TKEO_stat at 0 or 1");
            }
            //print(tkeo_stat_);

            if (hold_center_time_reached) {

                /// check for detection buffer full
                if (!tkeo_buffer_full) {
                    tkeo_buffer_full = check_wait_time_reached(tkeo_buffer_fill_time_, st_enter_time, timer_.get_elapsed_time());
                }
                else if (tkeo_buffer_full) {

                    if (!virtual_emg_) {
                        rest_state_reached = tkeo_stat_ > rest_tkeo_threshold_;
                        rest_state_reached = true;
                    }
                    else {
                        rest_state_reached = true;
                    }
                }
            }
        }
        
        /// log robot data
        log_robot_row();

        /// check for stop input
        check_manual_stop();

        if (!virtual_exo_) {

            /// write to daq
            daq_.update_output();
        }

        /// wait for the next clock cycle
        timer_.wait();
    }

    /// transition to next state from "HOLD AT CENTER"
    if (auto_stop_ || manual_stop_) {
        event(ST_FAULT_STOP);
    }
    else if (rest_state_reached) {
        if (is_cal()) {
            store_buffer(emg_calibration_data_buffer_, tkeo_rest_data_);
            //log_trial_row();
        }
        event(ST_PRESENT_TARGET);    
    }
    else {
        print("ERROR: State transition undefined. Going to ST_FAULT_STOP.");
        event(ST_FAULT_STOP);
    }
}


///-----------------------------------------------------------------------------
/// "HOLD FOR INPUT" STATE FUNCTION
///-----------------------------------------------------------------------------
void EmgRTControl::sf_hold_for_input(const NoEventData* data) {
    print("Hold for Input");

    /// reset global variables
    if (class_label_sequence_.empty()) {
        end_of_label_sequence_ = false;
    }

    /// initialize local state variables
    bool finished = false;
    bool present_more_targets = false;
    bool more_training_data = false;
    int num_observations_per_class;
    Key key;
    std::vector<double> command_torques(meii_.N_aj_, 0.0);

    /// initialization actions upon first entry to ST_HOLD_FOR_INPUT
    if (!end_of_label_sequence_) {
        if (is_cal()) {

            if (is_single_dof()) {
                class_label_sequence_ = { 1, 2 };
            }
            else {
                class_label_sequence_ = { 1,2,3,4 };
            }
            present_more_targets = true;
        }
        else if (is_training()) {

            /// read in emg calibration data from file
            load_emg_active_classifier();

            /// ask user if they want to use the existing training data
            print("Press Enter to collect new training data, or press L to Load previous training data...");
        }
        else if (is_testing()) {

            /// read in emg calibration data from file
            load_emg_active_classifier();

            /// read in classifier from file
            load_emg_dir_classifier();

            /// set pre-determined number of testing observations
            if (is_blind()) {
                num_observations_per_class = num_blind_testing_trials_;
            }
            else {
                num_observations_per_class = num_full_testing_trials_;
            }
            present_more_targets = true;
        }
        else {
            print("ERROR: Condition number was set improperly. Going to ST_FAULT_STOP.");
            auto_stop_ = true;
        }
    }

    /// initialization actions upon end of label sequence
    else { 
        if (is_cal()) {

            /// compute statistics from calibration data
            tkeo_model_estimation();

            /// write calibration data to file
            write_emg_active_classifier();
            finished = true;
            print("tkeo and file write done");
        }
        else if (is_training()) {

            /// write training data to file
            if (!write_csv<double>(training_data_filename_, subject_dof_directory_, emg_training_data_)) {
                auto_stop_ = true;
            }

            /// go directly to ST_FINISH
            finished = true;         
            
        }
        else if (is_testing()) {

            /// print classification performance report
            Eigen::MatrixXd confusion_mat = gen_confusion_mat(class_label_sequence_, pred_class_label_sequence_);
            print("Confusion Matrix for Last Round of Trials: ");
            std::cout << confusion_mat << std::endl;

            /// going directly to ST_FINISH
            finished = true;
        }
        else {
            print("ERROR: Condition number was set improperly. Going to ST_FAULT_STOP.");
            auto_stop_ = true;
        } 
    }
    
    /// write to Unity
    viz_target_num_[0] = 0;
    viz_target_num_share_.send_message("target_flag");
    viz_target_num_share_.send_data(viz_target_num_);

    /// enter the control loop
    while ( !present_more_targets && !finished && !manual_stop_ && !auto_stop_) {

        /// read and reload DAQs
        watchdog_.kick();
        daq_.update_input();

        if (!virtual_exo_) {

            /// update robot kinematics
            meii_.update_kinematics();

            /// check joint limits
            if (meii_.check_all_joint_limits()) {
                auto_stop_ = true;
                break;
            }

            /// run position control
            command_torques = meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, timer_.get_elapsed_time());

            if (scope_mode_) {

                /// write kinematics and motor commands to MelScope
                pos_share_.write_data(meii_.get_anatomical_joint_positions());
                vel_share_.write_data(meii_.get_anatomical_joint_velocities());
                torque_share_.write_data(command_torques);
            }
        }       

        /// check for external input upon first entry to ST_HOLD_FOR_INPUT
        if (!end_of_label_sequence_) {
            if (is_cal()) {
                print("ERROR: Should not be reached. Going to ST_FAULT_STOP.");
                auto_stop_ = true;
            }
            else if (is_training()) {

                if (!more_training_data) {
                    key = Keyboard::are_any_keys_pressed({ Key::Enter, Key::L });
                    switch (key) {
                    case Key::Enter:
                        more_training_data = true;
                        print("How many observations per class would you like to collect? (0-9)");
                        break;
                    case Key::L:
                        print("Loading previous training data from " + subject_dof_directory_ + "\\" + training_data_filename_);
                        if (!read_csv<double>(training_data_filename_, subject_dof_directory_, prev_emg_training_data_)) {
                            auto_stop_ = true;
                            break;
                        }
                        emg_training_data_ = prev_emg_training_data_;
                        more_training_data = true;
                        print("How many observations per class would you like to collect? (0-9)");
                        break;
                    }
                }
                else {

                    num_observations_per_class = is_any_num_key_pressed();
                    if (num_observations_per_class > 0) {
                        present_more_targets = true;
                    }
                    else if (num_observations_per_class == 0) {
                        more_training_data = false;
                        finished = true;
                    }
                }
            }
            else if (is_testing()) {
                print("ERROR: Should not be reached. Going to ST_FAULT_STOP.");
                auto_stop_ = true;
            }
            else {
                print("ERROR: Condition number was set improperly. Going to ST_FAULT_STOP.");
                auto_stop_ = true;
            }
        }

        /// check for external input upon end of label sequence
        else {
            if (is_cal()) {
                print("ERROR: Should not be reached. Going to ST_FAULT_STOP.");
                auto_stop_ = true;
            }
            else if (is_training()) {
                print("ERROR: Should not be reached. Going to ST_FAULT_STOP.");
                auto_stop_ = true;
            }
            else if (is_testing()) {
                print("ERROR: Should not be reached. Going to ST_FAULT_STOP.");
                auto_stop_ = true;
            }
            else {
                print("ERROR: Condition number was set improperly. Going to ST_FAULT_STOP.");
                auto_stop_ = true;
            }       
        }

        /// log data
        log_robot_row();

        /// check for manual stop input
        check_manual_stop();

        if (!virtual_exo_) {

            /// write to daq
            daq_.update_output();
        }

        /// wait for the next clock cycle
        timer_.wait();
    }

    /// transition to next state from "HOLD FOR INPUT"
    if (auto_stop_ || manual_stop_) {
        event(ST_FAULT_STOP);
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
        print("ERROR: State transition undefined. Going to ST_FAULT_STOP.");
        event(ST_FAULT_STOP);
    }
}



///-----------------------------------------------------------------------------
/// "PRESENT TARGET" STATE FUNCTION
///-----------------------------------------------------------------------------
void EmgRTControl::sf_present_target(const NoEventData* data) {
    print("Present Target");

    /// initialize global variables
    emg_calibration_data_buffer_.clear();
    emg_classification_data_buffer_.clear();
    meii_.butter_hp_.reset();
    meii_.tko_.reset();
    meii_.tkeo_butter_lp_.reset();
    tkeo_stat_buffer_.clear();
    tkeo_stat_buffer_.resize(tkeo_stat_buffer_size_);
    emg_voltages_ = std::vector<double>(meii_.N_emg_, 0.0);
    filtered_emg_voltages_ = std::vector<double>(meii_.N_emg_, 0.0);

    /// initialize local state variables
    bool force_mag_reached = false;
    bool active_state_reached = false;
    bool detector_expired = false;
    bool mvc_started = false;
    Time mvc_start_time = seconds(0.0);
    bool mvc_completed = false;
    bool keep_mvc = false;
    bool redo_mvc = false;
    std::vector<double> tkeo_vec(meii_.N_emg_, 0.0);
    std::vector<double> filtered_tkeo_vec(meii_.N_emg_, 0.0);
    std::vector<double> command_torques(meii_.N_aj_, 0.0);
    std::vector<double> force_mag = { 0.0 };
    std::vector<double> tkeo_active_mag = { 0.0 };
    bool tkeo_buffer_full = false;
    Time st_enter_time = timer_.get_elapsed_time();
    
    /// initialize magnitude force checking algorithm
    //force_mag_maintained_ = 0.0;
    //force_mag_time_now_ = clock_.get_current_time();
    //force_mag_time_last_ = clock_.get_current_time();

    /// read from target sequence and write to Unity
    set_viz_target_num(class_label_sequence_[current_class_label_idx_]);
    viz_target_num_share_.send_message("target_flag");
    viz_target_num_share_.send_data(viz_target_num_);

    print("Target class label: " + std::to_string(class_label_sequence_[current_class_label_idx_]));

    if (is_cal()) {

        /// ask user to indicate when MVC has started
        print("Press ENTER when MVC has started.");
    }

    /// send state flag to unity
    state_net_.send_message("state_flag");
    state_net_.send_data({ 1.0 });

    /// enter the control loop
    while (!end_of_label_sequence_ && !keep_mvc && !redo_mvc && !active_state_reached && !detector_expired && !manual_stop_ && !auto_stop_) {

        /// read and reload DAQs
        watchdog_.kick();
        daq_.update_input();

        if (!virtual_exo_) {

            /// update robot kinematics
            meii_.update_kinematics();

            /// check joint limits
            if (meii_.check_all_joint_limits()) {
                auto_stop_ = true;
                break;
            }

            /// run position control
            command_torques = meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, timer_.get_elapsed_time());

            if (scope_mode_) {

                /// write kinematics and motor commands to MelScope
                pos_share_.write_data(meii_.get_anatomical_joint_positions());
                vel_share_.write_data(meii_.get_anatomical_joint_velocities());
                torque_share_.write_data(command_torques);
            }
        }

        /// get measured emg voltages and log them
        emg_voltages_ = meii_.get_emg_voltages();
        meii_.butter_hp_.filter(emg_voltages_, filtered_emg_voltages_);
        log_emg_row();
        if (scope_mode_) {
            emg_share_.write_data(filtered_emg_voltages_);
        }

        /// compute the teager-kaiser metric online with rectification and filtering and store in calibration buffer       
        meii_.tko_.tkeo(filtered_emg_voltages_, tkeo_vec);
        std::for_each(tkeo_vec.begin(), tkeo_vec.end(), [](double x) { return std::abs(x); });
        meii_.tkeo_butter_lp_.filter(tkeo_vec, filtered_tkeo_vec);

        /// store emg data in appropriate buffer
        if (is_cal()) {
            if (mvc_started && !mvc_completed) {
                emg_calibration_data_buffer_.push_back(filtered_tkeo_vec);
            }
        }
        else {
            emg_classification_data_buffer_.push_back(filtered_emg_voltages_);
        }

        /// measure interaction force for specified dof(s)
        force_mag[0] = measure_task_force(command_torques, class_label_sequence_[current_class_label_idx_], dof_, condition_);

        /// check for break conditions
        if (is_cal()) {

            /// check for user input
            if (!mvc_started) {
                if (Keyboard::is_key_pressed(Key::Enter)) {
                    mvc_started = true;
                    mvc_start_time = timer_.get_elapsed_time();
                    print("Holding for contraction.");
                }
            }
            else {
                if (!mvc_completed) {
                    mvc_completed = check_wait_time_reached(wait_mvc_time_, mvc_start_time, timer_.get_elapsed_time());
                    if (mvc_completed) {
                        print("Do you want to keep the calibration data for this contraction (Y/N)?");
                    }
                }
                else {
                    Key key = Keyboard::are_any_keys_pressed({ Key::Y, Key::N });
                    switch (key) {
                    case Key::Y:
                        keep_mvc = true;
                        break;
                    case Key::N:
                        redo_mvc = true;
                        break;
                    }
                }
            }
        }
        else {

            /// compute tkeo detector statistic and fill buffer
            tkeo_stat_ = tkeo_detector(filtered_tkeo_vec);

            if (tkeo_stat_ == 0 || tkeo_stat_ == 1) {
                //print("WARNING: TKEO_stat at 0 or 1");
            }
            //print(tkeo_stat_);

            /// check for detection buffer full
            if (!tkeo_buffer_full) {
                tkeo_buffer_full = check_wait_time_reached(tkeo_buffer_fill_time_, st_enter_time, timer_.get_elapsed_time());
                tkeo_active_mag[0] = 0.0;
            }
            else {

                if (!virtual_emg_) {
                    active_state_reached = tkeo_stat_ < (1.0 - active_tkeo_threshold_);
                    tkeo_active_mag[0] = (1.0 - tkeo_stat_) * force_mag_goal_;

                    if (is_testing()) {

                        /// check for detector expiry
                        detector_expired = check_wait_time_reached(detection_expire_time_, st_enter_time, timer_.get_elapsed_time());
                    }
                }
                else {
                    active_state_reached = true;
                    tkeo_active_mag[0] = force_mag_goal_;
                }
            }
        }     

        /// write to unity
        if (force_mag_share_.receive_message() == "send_force") {
            if (is_cal()) {
                force_mag_share_.send_data(force_mag);
            }
            else {
                force_mag_share_.send_data(tkeo_active_mag);
            }
        }
            

        /// log data
        log_robot_row();

        /// check for stop input
        check_manual_stop();

        if (!virtual_exo_) {

            /// write to daq
            daq_.update_output();
        }

        /// wait for the next clock cycle
        timer_.wait();
    }

    tkeo_active_mag[0] = 0.0;
    if (force_mag_share_.receive_message() == "send_force") {
        force_mag_share_.send_data(tkeo_active_mag);
    }

    /// send state flag to unity
    state_net_.send_message("state_flag");
    state_net_.send_data({ 0.0 });

    /// transition to next state from "PRESENT TARGET"
    if (auto_stop_ || manual_stop_) {
        event(ST_FAULT_STOP);
    }
    else if (keep_mvc) {
        store_buffer(emg_calibration_data_buffer_, tkeo_active_data_[current_class_label_idx_]);
        //log_trial_row();
        event(ST_TO_CENTER);
    }
    else if (redo_mvc) {
        event(ST_PRESENT_TARGET);
    }
    else if (active_state_reached) {      
        if (process_emg()) {
            if (is_training()) {
                //log_trial_row();
                event(ST_TO_CENTER);
            }
            else {
                classify();
                if (is_blind()) {
                    //log_trial_row();
                    event(ST_TO_CENTER);
                }
                else {
                    //log_trial_row();
                    event(ST_TO_TARGET);
                }
            }
        }
        else {
            print("ERROR: EMG data unsuccessfully processed. Going to ST_FAULT_STOP.");
            event(ST_FAULT_STOP);
        }       
    }
    else if (detector_expired) {
        if (is_testing()) {
            pred_class_label_sequence_.push_back(0);
            emg_feature_vec_ = std::vector<double>(num_emg_channels_ * num_features_, 0.0);
            class_posteriors_ = std::vector<double>(num_classes_, 0.0);
            if (is_blind()) {
                //log_trial_row();
                event(ST_TO_CENTER);
            }
            else {
                //log_trial_row();
                event(ST_TO_TARGET);
            }           
        }
        else {
            print("ERROR: Boolean variable detector_expired should not be set to true outside of testing conditions.");
            event(ST_FAULT_STOP);
        }
    }
    else {
        print("ERROR: State transition undefined. Going to ST_FAULT_STOP.");
        event(ST_FAULT_STOP);
    }
}



///-----------------------------------------------------------------------------
/// "GO TO TARGET" STATE FUNCTION
///-----------------------------------------------------------------------------
void EmgRTControl::sf_to_target(const NoEventData* data) {
    print("Go to Target");

    /// initialize local state variables
    bool target_reached = false;
    std::vector<double> command_torques(meii_.N_aj_, 0.0);

    /// set new reference position as predicted label
    std::vector<double> target_pos = get_target_position(pred_class_label_sequence_[current_class_label_idx_]);
    meii_.anat_ref_.set_ref(target_pos, timer_.get_elapsed_time());

    /// write to Unity actual label
    set_viz_target_num(class_label_sequence_[current_class_label_idx_]);
    viz_target_num_share_.send_message("target_flag");
    viz_target_num_share_.send_data(viz_target_num_);

    /// enter the control loop
    while (!target_reached && !manual_stop_ && !auto_stop_) {

        /// read and reload DAQs
        watchdog_.kick();
        daq_.update_input();

        if (!virtual_exo_) {

            /// update robot kinematics
            meii_.update_kinematics();

            /// check joint limits
            if (meii_.check_all_joint_limits()) {
                auto_stop_ = true;
                break;
            }

            /// run position control
            command_torques = meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, timer_.get_elapsed_time());

            if (scope_mode_) {

                /// write kinematics and motor commands to MelScope
                pos_share_.write_data(meii_.get_anatomical_joint_positions());
                vel_share_.write_data(meii_.get_anatomical_joint_velocities());
                torque_share_.write_data(command_torques);
            }
        }

        if (!virtual_exo_) {

            /// check for target reached
            target_reached = meii_.check_goal_anat_pos(target_pos, { 1, 1, 1, 1, 0 });
        }
        else {
            target_reached = true;
        }

        /// log data
        log_robot_row();

        /// check for stop input
        check_manual_stop();

        if (!virtual_exo_) {

            /// write to daq
            daq_.update_output();
        }

        /// wait for the next clock cycle
        timer_.wait();
    }

    /// transition to next state from "GO TO TARGET"
    if (auto_stop_ || manual_stop_) {
        event(ST_FAULT_STOP);
    }
    else if (target_reached) {
        event(ST_HOLD_TARGET);
    }
    else {
        print("ERROR: State transition undefined. Going to ST_FAULT_STOP.");
        event(ST_FAULT_STOP);
    }
}


///-----------------------------------------------------------------------------
/// "HOLD AT TARGET" STATE FUNCTION
///-----------------------------------------------------------------------------
void EmgRTControl::sf_hold_target(const NoEventData* data) {
    print("Hold at Target"); 

    /// initialize local state variables
    bool hold_target_time_reached = false;
    Time st_enter_time = timer_.get_elapsed_time();
    std::vector<double> command_torques(meii_.N_aj_, 0.0);


    /// enter the control loop
    while (!hold_target_time_reached && !manual_stop_ && !auto_stop_) {

        /// read and reload DAQs
        watchdog_.kick();
        daq_.update_input();

        if (!virtual_exo_) {

            /// update robot kinematics
            meii_.update_kinematics();

            /// check joint limits
            if (meii_.check_all_joint_limits()) {
                auto_stop_ = true;
                break;
            }

            /// run position control
            command_torques = meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, timer_.get_elapsed_time());

            if (scope_mode_) {

                /// write kinematics and motor commands to MelScope
                pos_share_.write_data(meii_.get_anatomical_joint_positions());
                vel_share_.write_data(meii_.get_anatomical_joint_velocities());
                torque_share_.write_data(command_torques);
            }
        }

        /// check for hold time reached
        hold_target_time_reached = check_wait_time_reached(hold_target_time_, st_enter_time, timer_.get_elapsed_time());

        /// log data
        log_robot_row();

        /// check for stop input
        check_manual_stop();

        if (!virtual_exo_) {

            /// write to daq
            daq_.update_output();
        }

        /// wait for the next clock cycle
        timer_.wait();
    }

    /// transition to next state from "HOLD TARGET"
    if (auto_stop_ || manual_stop_) {
        event(ST_FAULT_STOP);
    }
    else if (hold_target_time_reached) {
        event(ST_TO_CENTER);
    }
    else {
        print("ERROR: State transition undefined. Going to ST_FAULT_STOP.");
        event(ST_FAULT_STOP);
    }
}



///-----------------------------------------------------------------------------
/// "FINISH EXPERIMENT" STATE FUNCTION
///-----------------------------------------------------------------------------
void EmgRTControl::sf_finish(const NoEventData* data) {
    print("Finishing Experiment");

    /// intialize local state variables
    bool menu_selected = false;
    bool exit_program = false;
    std::vector<double> lda_training_complete = { 0.0 };
    bool python_return = false;
    bool cv_score_achieved = true;

    /// disable robot
    if (meii_.is_enabled()) {
        meii_.disable();
    }

    /// disable watchdog
    if (daq_.is_enabled()) {
        watchdog_.stop();
    }

    /// save data
    save_log_data();

    if (is_training()) {

        /// send file location to python over melshare
        directory_share_.write_message(subject_dof_directory_);
        file_name_share_.write_message(training_data_filename_);

        /// open LDA script in Python
        std::string system_command;
        system_command = "start " + program_directory_ + "\\" + "EMG_FS_LDA.py &";
        system(system_command.c_str());

        /// wait for python to return results
        print("Waiting for Python to return training results...");

        while (!python_return && !manual_stop_ && !auto_stop_) {

            if (lda_training_complete[0] == 0.0) {
                lda_training_complete = lda_training_flag_.read_data();
            }
            else {

                if (!python_return) {
                    python_return = true;

                    /// read in the results from the Cross-validation test in Python
                    print("The cross validation results are:");
                    std::vector<double> cross_eval_test(5);
                    cross_eval_test = cv_results_.read_data();
                    print(cross_eval_test);

                    /// check if results meet criteria for ending training
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

            /// check for stop input
            check_manual_stop();

            /// wait for the next clock cycle
            timer_.wait();
        }

        /// wait to ensure data is logged
        sleep(seconds(3));
    }

    /// wait for user input
    print("Press 'm' in Unity to return to the GUI main menu or press 'ENTER' to stop the experiment");
    while (!menu_selected && !exit_program && !manual_stop_ && !auto_stop_) {

        /// read from Unity
        if (scene_num_share_.receive_message() == "scene_change") {
            SCENE_NUM_ = scene_num_share_.receive_data();
            if (SCENE_NUM_[0] == 0) {
                menu_selected = true;
            }
        }
        

        /// check for end of experiment
        if (Keyboard::is_key_pressed(Key::Enter)) {
            exit_program = true;
        }

        /// check for stop input
        check_manual_stop();

        /// wait for the next clock cycle
        timer_.wait();
    }

    /// transition to next state from "FINISH"
    if (auto_stop_ || manual_stop_) {
        event(ST_FAULT_STOP);
    }
    else if (menu_selected) {
        event(ST_WAIT_FOR_GUI);
    }
    else if (exit_program) {
        event(ST_STOP);
    }
    else {
        print("ERROR: State transition undefined. Going to ST_FAULT_STOP.");
        event(ST_FAULT_STOP);
    }
}


///-----------------------------------------------------------------------------
/// "STOP" STATE FUNCTION
///-----------------------------------------------------------------------------

void EmgRTControl::sf_stop(const NoEventData* data) {
    std::cout << "Exiting Program" << std::endl;

    /// disable robot and daq
    if (meii_.is_enabled()) {
        meii_.disable();
    }
    /*if (daq_.is_enabled()) {
        daq_.disable();
    }*/
}


///-----------------------------------------------------------------------------
/// "FAULT STOP" STATE FUNCTION
///-----------------------------------------------------------------------------

void EmgRTControl::sf_fault_stop(const NoEventData* data) {
    std::cout << "Program Stopped with Potential Fault" << std::endl;

    /// disable robot and daq
    if (meii_.is_enabled()) {
        meii_.disable();
    }
    /*if (daq_.is_enabled()) {
        if (!daq_.is_open)) {
            print("WATCHDOG HAS EXPIRED.");
        }
        daq_.disable();
    }*/

    /// ask user if want to save data
    print("The program was stopped manually or with a fault. Would you still like to save the data to the 'Error Report' directory (Y/N)?");
    Key key = Keyboard::wait_for_any_keys({ Key::Y, Key::N });
    switch (key) {
    case Key::Y:
        save_log_data();
        sleep(seconds(3)); /// wait to ensure data is logged
        break;
    case Key::N:
        print("Data not saved.");
        break;
    }

    if (is_training()) {

        /// ask user if want to save data
        print("The program was stopped manually or with a fault. Would you still like to save the training data to the subject directory (Y/N)?");
        Key key = Keyboard::wait_for_any_keys({ Key::Y, Key::N });
        switch (key) {
        case Key::Y:

            /// write training data to file
            if (!write_csv<double>(training_data_filename_, subject_dof_directory_, emg_training_data_)) {
            }

            manual_stop_ = false;
            save_log_data();
            manual_stop_ = true;
            sleep(seconds(3)); /// wait to ensure data is logged
            break;
        case Key::N:
            print("Data not saved.");
            break;
        }     
    }
}



///-----------------------------------------------------------------------------
/// EMG REAL-TIME CONTROL UTILITY FUNCTIONS
///-----------------------------------------------------------------------------

bool EmgRTControl::process_emg() {
    print("Process EMG Data");

    /// default return value to true; only write false if error encountered
    bool emg_data_processed = true; 

    /// extract features from EMG data
    emg_feature_vec_ = feature_extract(emg_classification_data_buffer_);
    
    /// check for successful data processing
    bool are_nan_features = false;
    std::vector<int> nan_idx;
    for (int i = 0; i < emg_feature_vec_.size(); ++i) {
        if (std::isnan(emg_feature_vec_[i])) {
            are_nan_features = true;
            emg_feature_vec_[i] = 1.0;
            nan_idx.push_back(i);
        }
    }
    if (are_nan_features) {
        print("ERROR: NaN feature(s) detected.");
        print(nan_idx);
        emg_data_processed = false;
    }

    /// store feature array in training data set
    if (is_training()) {
        emg_feature_vec_.push_back(class_label_sequence_[current_class_label_idx_]);
        emg_training_data_.push_back(emg_feature_vec_);
    }
    
    return emg_data_processed;
}

void EmgRTControl::classify() {
    print("Classifying EMG Activation");

    /// convert feature vector to Eigen type and add dummy feature
    Eigen::VectorXd emg_feature_vec_eig(emg_feature_vec_.size() + 1);
    for (int i = 0; i < emg_feature_vec_.size(); ++i) {
        emg_feature_vec_eig(i) = emg_feature_vec_[i];
    }
    emg_feature_vec_eig.tail(1) = Eigen::VectorXd::Ones(1);

    /// compute the output of the linear model
    Eigen::VectorXd y;
    y = emg_dir_W_tt_ * emg_feature_vec_eig;

    /// compute the posterior probabilities for each class    
    if (num_classes_ == 2) {       
        class_posteriors_[1] = sigmoid(y(0));
        class_posteriors_[0] = 1.0 - class_posteriors_[1];
    }
    else {
        for (int k = 0; k < num_classes_; ++k) {
            class_posteriors_[k] = softmax(y, k);
        }  
    }
    print("Class posterior probabilities: ");
    print(class_posteriors_);
    
    /// make movement direction decision
    pred_class_label_sequence_.push_back(std::distance(class_posteriors_.begin(), std::max_element(class_posteriors_.begin(), class_posteriors_.end())) + 1);
    
    print("Predicted class label: " + std::to_string(pred_class_label_sequence_[current_class_label_idx_]));
}

double EmgRTControl::tkeo_detector(const std::vector<double>& sample_vec) {
    Eigen::VectorXd sample = copy_stdvec_to_eigvec(sample_vec);
    Eigen::VectorXd y = tkeo_W_t_ * sample + tkeo_w_0_;
    double rest_posterior = softmax(y, 0);
    tkeo_stat_buffer_.push_back(rest_posterior);
    std::vector<double> tkeo_stat_vec(tkeo_stat_buffer_size_);
    for (size_t i = 0; i < tkeo_stat_buffer_size_; ++i) {
        tkeo_stat_vec[i] = tkeo_stat_buffer_[i];
    }
    return std::accumulate(tkeo_stat_vec.begin(), tkeo_stat_vec.end(), 0.0) / (double) tkeo_stat_buffer_size_;
}



///-----------------------------------------------------------------------------
/// EMG PATTERN RECOGNITION TRAINING UTILITY FUNCTIONS
///-----------------------------------------------------------------------------


void EmgRTControl::tkeo_model_estimation() {

    /// data dimension
    size_t D = meii_.N_emg_;
    Eigen::MatrixXd tkeo_sample_means(D, num_classes_ + 1);
    Eigen::MatrixXd tkeo_sample_cov = Eigen::MatrixXd::Zero(D, D);
    Eigen::MatrixXd tkeo_sample_cov_inv(D, D);

    Eigen::VectorXd rest_sample_mean(D);
    Eigen::MatrixXd rest_sample_cov(D, D);
    estimate_gaussian_params(tkeo_rest_data_, rest_sample_mean, rest_sample_cov);

    tkeo_sample_means.col(0) = rest_sample_mean;
    double scalar = static_cast<double>(tkeo_active_data_.size()) - 1.0;
    rest_sample_cov /= scalar;
    tkeo_sample_cov = rest_sample_cov;

    scalar = static_cast<double>(tkeo_active_data_.size());
    for (size_t k = 0; k < tkeo_active_data_.size(); ++k) {

        Eigen::VectorXd active_sample_mean(D);
        Eigen::MatrixXd active_sample_cov(D, D);
        estimate_gaussian_params(tkeo_active_data_[k], active_sample_mean, active_sample_cov);

        tkeo_sample_means.col(k + 1) = active_sample_mean;
        active_sample_cov /= scalar;
        tkeo_sample_cov += active_sample_cov;
    }


    if (tkeo_sample_cov.fullPivLu().isInvertible())
        tkeo_sample_cov_inv = tkeo_sample_cov.fullPivLu().inverse();
    else {
        print("ERROR: TKEO sample covariance matrix cannot be inverted.");
        auto_stop_ = true;
        return;
    }   

    /// compute linear model
    tkeo_W_t_.resize(num_classes_ + 1, D);
    tkeo_w_0_.resize(num_classes_ + 1);
    for (int k = 0; k < num_classes_ + 1; ++k) {
        tkeo_W_t_.row(k) = tkeo_sample_means.col(k).transpose() * tkeo_sample_cov_inv;
        tkeo_w_0_(k) = -tkeo_sample_means.col(k).transpose() * tkeo_sample_cov_inv * tkeo_sample_means.col(k);
    }

}


void EmgRTControl::estimate_gaussian_params(const std::vector<std::vector<double>>& sample_data, Eigen::VectorXd& sample_mean, Eigen::MatrixXd& sample_cov) {
    size_t n_rows = sample_data.size();
    size_t n_cols = sample_data[0].size();

    sample_mean.resize(n_rows);
    sample_cov.resize(n_rows, n_rows);

    sample_mean.setConstant(0.0);
    sample_cov.setConstant(0.0);

    for (size_t i = 0; i < n_rows; ++i) {
        if (sample_data[i].size() != n_cols) {
            print("ERROR: Function ml_covariance_estimate was given argument sample_data with incorrect dimensions.");
            return;
        }
    }

    for (size_t i = 0; i < n_rows; ++i) {
        sample_mean(i) = std::accumulate(sample_data[i].begin(), sample_data[i].end(), 0.0) / static_cast<double>(n_cols);
    }

    Eigen::MatrixXd sample(n_rows, 1);
    for (size_t j = 0; j < n_cols; ++j) {
        for (size_t i = 0; i < n_rows; ++i) {
            sample(i, 0) = sample_data[i][j];
        }
        sample -= sample_mean;
        sample_cov += sample * sample.transpose().eval();
    }
    sample_cov = sample_cov / static_cast<double>(n_cols);
}

///-----------------------------------------------------------------------------
/// EMG FEATURE EXTRACTION FUNCTIONS
///-----------------------------------------------------------------------------

std::vector<double> EmgRTControl::feature_extract(const MahiExoIIEmg::EmgDataBuffer& emg_data_buffer) const {

    std::vector<double> feature_vec;
    std::vector<double> nrms_vec(meii_.N_emg_, 0.0);
    std::vector<double> nmav_vec(meii_.N_emg_, 0.0);
    std::vector<double> nwl_vec(meii_.N_emg_, 0.0);
    std::vector<double> nzc_vec(meii_.N_emg_, 0.0);
    std::vector<double> nssc_vec(meii_.N_emg_, 0.0);
    std::vector<double> ar_vec(4, 0.0);
    std::vector<double> ar1_vec(meii_.N_emg_, 0.0);
    std::vector<double> ar2_vec(meii_.N_emg_, 0.0);
    std::vector<double> ar3_vec(meii_.N_emg_, 0.0);
    std::vector<double> ar4_vec(meii_.N_emg_, 0.0);

    /// extract unnormalized features
    for (int i = 0; i < meii_.N_emg_; ++i) {
        nrms_vec[i] = rms_feature_extract(emg_data_buffer.get_channel(i));
        nmav_vec[i] = mav_feature_extract(emg_data_buffer.get_channel(i));
        nwl_vec[i] = wl_feature_extract(emg_data_buffer.get_channel(i));
        nzc_vec[i] = zc_feature_extract(emg_data_buffer.get_channel(i));
        nssc_vec[i] = ssc_feature_extract(emg_data_buffer.get_channel(i));
        ar_4_feature_extract(ar_vec, emg_data_buffer.get_channel(i));
        ar1_vec[i] = ar_vec[0];
        ar2_vec[i] = ar_vec[1];
        ar3_vec[i] = ar_vec[2];
        ar4_vec[i] = ar_vec[3];
    }

    /// normalize time-domain features
    double rms_mean = mean(nrms_vec);
    double mav_mean = mean(nmav_vec);
    double wl_mean = mean(nwl_vec);
    double zc_mean = mean(nzc_vec);
    double ssc_mean = mean(nssc_vec);
    for (int i = 0; i < meii_.N_emg_; ++i) {
        if (rms_mean > 0) {
            nrms_vec[i] = nrms_vec[i] / rms_mean;
        }
        if (mav_mean > 0) {
            nmav_vec[i] = nmav_vec[i] / mav_mean;
        }
        if (wl_mean > 0) {
            nwl_vec[i] = nwl_vec[i] / wl_mean;
        }
        if (zc_mean > 0) {
            nzc_vec[i] = nzc_vec[i] / zc_mean;
        }
        if (ssc_mean > 0) {
            nssc_vec[i] = nssc_vec[i] / ssc_mean;
        }
    }

    /// check for validity of feature computations
    if (!virtual_emg_) {
        if (std::all_of(nrms_vec.begin(), nrms_vec.end(), [](double x) { return x == 1.0; })) {
            print("WARNING: All RMS features have same value.");
        }
        if (std::all_of(nmav_vec.begin(), nmav_vec.end(), [](double x) { return x == 1.0; })) {
            print("WARNING: All MAV features have same value.");
        }
        if (std::all_of(nwl_vec.begin(), nwl_vec.end(), [](double x) { return x == 1.0; })) {
            print("WARNING: All WL features have same value.");
        }
        if (std::all_of(nzc_vec.begin(), nzc_vec.end(), [](double x) { return x == 1.0; })) {
            print("WARNING: All ZC features have same value.");
        }
        if (std::all_of(nssc_vec.begin(), nssc_vec.end(), [](double x) { return x == 1.0; })) {
            print("WARNING: All SSC features have same value.");
        }
    }

    /// copy features into one vector
    feature_vec.insert(feature_vec.end(), nrms_vec.begin(), nrms_vec.end());
    feature_vec.insert(feature_vec.end(), nmav_vec.begin(), nmav_vec.end());
    feature_vec.insert(feature_vec.end(), nwl_vec.begin(), nwl_vec.end());
    feature_vec.insert(feature_vec.end(), nzc_vec.begin(), nzc_vec.end());
    feature_vec.insert(feature_vec.end(), nssc_vec.begin(), nssc_vec.end());
    feature_vec.insert(feature_vec.end(), ar1_vec.begin(), ar1_vec.end());
    feature_vec.insert(feature_vec.end(), ar2_vec.begin(), ar2_vec.end());
    feature_vec.insert(feature_vec.end(), ar3_vec.begin(), ar3_vec.end());
    feature_vec.insert(feature_vec.end(), ar4_vec.begin(), ar4_vec.end());

    return feature_vec;

}

double EmgRTControl::rms_feature_extract(const std::vector<double>& emg_channel_buffer) const {
    double sum_squares = 0;
    for (int i = 0; i < emg_channel_buffer.size(); ++i) {
        sum_squares += std::pow(emg_channel_buffer[i], 2);
    }
    return std::sqrt(sum_squares / emg_channel_buffer.size());
}

double EmgRTControl::mav_feature_extract(const std::vector<double>& emg_channel_buffer) const {
    double sum_abs = 0;
    for (int i = 0; i < emg_channel_buffer.size(); ++i) {
        sum_abs += std::abs(emg_channel_buffer[i]);
    }
    return sum_abs / emg_channel_buffer.size();
}

double EmgRTControl::wl_feature_extract(const std::vector<double>& emg_channel_buffer) const {
    double sum_abs_diff = 0.0;
    for (int i = 0; i < emg_channel_buffer.size() - 1; ++i) {
        sum_abs_diff += std::abs(emg_channel_buffer[i + 1] - emg_channel_buffer[i]);
    }
    return sum_abs_diff;
}

double EmgRTControl::zc_feature_extract(const std::vector<double>& emg_channel_buffer) const {
    double sum_abs_diff_sign = 0;
    for (int i = 0; i < emg_channel_buffer.size() - 1; ++i) {
        sum_abs_diff_sign += std::abs(std::copysign(1.0, emg_channel_buffer[i + 1]) - std::copysign(1.0, emg_channel_buffer[i]));
    }
    return sum_abs_diff_sign / 2;
}

double EmgRTControl::ssc_feature_extract(const std::vector<double>& emg_channel_buffer) const {
    double sum_abs_diff_sign_diff = 0;
    for (int i = 0; i < emg_channel_buffer.size() - 2; ++i) {
        sum_abs_diff_sign_diff += std::abs(std::copysign(1.0, (emg_channel_buffer[i + 2] - emg_channel_buffer[i + 1])) - std::copysign(1.0, (emg_channel_buffer[i + 1] - emg_channel_buffer[i])));
    }
    return sum_abs_diff_sign_diff / 2;
}

void EmgRTControl::ar_4_feature_extract(std::vector<double>& coeffs, const std::vector<double>& emg_channel_buffer) const {

    /// Algorithm taken from Cedrick Collomb, "Burg's Method, Algorithm and Recursion," November 8, 2009

    /// initialize
    size_t N = emg_channel_buffer.size() - 1;
    size_t m = coeffs.size();
    std::vector<double> A_k(m + 1, 0.0);
    A_k[0] = 1.0;
    std::vector<double> f = emg_channel_buffer;
    std::vector<double> b = emg_channel_buffer;

    double D_k = 0;
    for (size_t j = 0; j <= N; ++j) {
        D_k += 2.0 * f[j] * f[j];
    }
    D_k -= f[0] * f[0] + b[N] * b[N];


    /// Burg recursion
    for (size_t k = 0; k < m; ++k) {

        /// compute mu
        double mu = 0.0;
        for (size_t n = 0; n <= N - k - 1; ++n) {
            mu += f[n + k + 1] * b[n];
        }
        mu *= -2.0 / D_k;

        /// update A_k
        for (size_t n = 0; n <= (k + 1) / 2; ++n) {
            double t1 = A_k[n] + mu * A_k[k + 1 - n];
            double t2 = A_k[k + 1 - n] + mu * A_k[n];
            A_k[n] = t1;
            A_k[k + 1 - n] = t2;
        }

        /// update f and b
        for (size_t n = 0; n <= N - k - 1; ++n) {
            double t1 = f[n + k + 1] + mu * b[n];
            double t2 = b[n] + mu * f[n + k + 1];
            f[n + k + 1] = t1;
            b[n] = t2;
        }

        /// update D_k
        D_k = (1.0 - mu * mu) * D_k - f[k + 1] * f[k + 1] - b[N - k - 1] * b[N - k - 1];
    }

    /// assign coefficients
    coeffs.assign(++A_k.begin(), A_k.end());
}

///-----------------------------------------------------------------------------
/// EXPERIMENT SETUP/CONDITIONS UTILITY FUNCTIONS
///-----------------------------------------------------------------------------

void EmgRTControl::set_experiment_conditions(int scene_num) {
    dof_ = (scene_num - 2) / 4;
    condition_ = (scene_num - 2) % 4;
    if (is_single_dof()) {
        num_classes_ = 2;
    }
    else {
        num_classes_ = 4;
    }
}

std::vector<int> EmgRTControl::gen_rand_class_labels(int num_labels) const {
    std::random_device rd_seed; /// random seed
    std::mt19937 rd_gen(rd_seed()); /// random unsigned integer generator
    std::uniform_int_distribution<> class_labels_dist(1, num_classes_);
    std::vector<int> class_label_list(num_labels, 0);
    for (int i = 0; i < num_labels; ++i) {
        class_label_list[i] = class_labels_dist(rd_gen);
    }
    return class_label_list;
}

std::vector<int> EmgRTControl::rand_shuffle_class_labels(int num_labels_per_class) const {
    std::vector<int> class_label_list(num_labels_per_class * num_classes_, 0);
    for (int i = 0; i < num_labels_per_class * num_classes_; ++i) {
        class_label_list[i] = 1 + (i / num_labels_per_class);
    }
    std::srand(std::time(0));
    std::random_shuffle(class_label_list.begin(), class_label_list.end());
    return class_label_list;
}

bool EmgRTControl::is_single_dof() const {
    return dof_ < 4;
}

bool EmgRTControl::is_cal() const {
    return condition_ == 0;
}

bool EmgRTControl::is_training() const {
    return condition_ == 1;
}

bool EmgRTControl::is_testing() const {
    return condition_ == 2 || condition_ == 3;
}

bool EmgRTControl::is_blind() const {
    return condition_ == 2;
}

bool EmgRTControl::is_full() const {
    return condition_ == 3;
}

///-----------------------------------------------------------------------------
/// EXPERIMENT VISUALIZATION UTILITY FUNCTIONS
///-----------------------------------------------------------------------------

void EmgRTControl::set_viz_target_num(int class_label) {
    viz_target_num_[0] = (double) class_label;
}

std::vector<double> EmgRTControl::get_target_position(int class_label) {
    std::vector<double> target_pos(meii_.N_aj_, 0.0);
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

///-----------------------------------------------------------------------------
/// EXPERIMENT CONTROL UTILITY FUNCTIONS
///-----------------------------------------------------------------------------

bool EmgRTControl::check_wait_time_reached(Time wait_time, Time init_time, Time current_time) const {
    return (current_time - init_time) > wait_time;
}

double EmgRTControl::measure_task_force(std::vector<double> commanded_torques, int target_num, int dof, int condition) const {

    double task_force;
    std::vector<std::vector<int>> target_dir;

    switch (task_force_measurement_mode_) {
    case 0:
        if (hand_def_ == "R") {
            target_dir = target_dir_R_;
        }
        else {
            target_dir = target_dir_L_;
        }

        if (dof < 4) {
            task_force = (-1.0) * force_mag_goal_ * (commanded_torques[dof] + gravity_offsets_[dof]) / force_dof_scale_[dof][target_num - 1][dof] * target_dir[dof][target_num - 1];
        }
        else if (dof == 4) {
            task_force = (-1.0) * force_mag_goal_ * ((commanded_torques[0] + gravity_offsets_[0]) / force_dof_scale_[dof][target_num - 1][0] * target_dir[dof][target_num - 1] + (commanded_torques[1] + gravity_offsets_[1]) / force_dof_scale_[dof][target_num - 1][1] * target_dir[dof + 1][target_num - 1]) / 2.0;
        }
        else if (dof == 5) {
            task_force = (-1.0) * force_mag_goal_ * ((commanded_torques[2] + gravity_offsets_[2]) / force_dof_scale_[dof][target_num - 1][2] * target_dir[dof][target_num - 1] + (commanded_torques[3] + gravity_offsets_[3]) / force_dof_scale_[dof][target_num - 1][3] * target_dir[dof + 1][target_num - 1]) / 2.0;
        }
        return std::max(0.0, task_force);
        break;
    case 1:
        if (dof < 4) {
            task_force = force_mag_goal_ * (commanded_torques[dof] + gravity_offsets_[dof]) / force_dof_scale_[dof][target_num - 1][dof];
        }
        else if (dof == 4) {
            task_force = force_mag_goal_ * (std::abs((commanded_torques[0] + gravity_offsets_[0])) / force_dof_scale_[dof][target_num - 1][0] + std::abs((commanded_torques[1] + gravity_offsets_[1])) / force_dof_scale_[dof][target_num - 1][1]) / 2.0;
        }
        else if (dof == 5) {
            task_force = force_mag_goal_ * (std::abs((commanded_torques[2] + gravity_offsets_[2])) / force_dof_scale_[dof][target_num - 1][2] + std::abs((commanded_torques[3] + gravity_offsets_[3])) / force_dof_scale_[dof][target_num - 1][3]) / 2.0;
        }
        return std::abs(task_force);
        break;
    case 2:
        task_force = force_mag_goal_ * (std::abs((commanded_torques[0] + gravity_offsets_[0])) / force_dof_scale_[dof][target_num - 1][0] + std::abs((commanded_torques[1] + gravity_offsets_[1])) / force_dof_scale_[dof][target_num - 1][1] + std::abs((commanded_torques[2] + gravity_offsets_[2])) / force_dof_scale_[dof][target_num - 1][2] + std::abs((commanded_torques[3] + gravity_offsets_[3])) / force_dof_scale_[dof][target_num - 1][3]) / 4.0;
        return task_force;
        break;
    default:
        print("ERROR: Invalid choice for task_force_measurement_mode_. Must be 0 to 2.");
        return NAN;
    }
}


//bool EmgRTControl::check_force_mag_reached(double force_mag_goal, double force_mag) {
//    force_mag_time_now_ = clock_.get_current_time();
//    force_mag_maintained_ = std::abs(force_mag_maintained_ + std::copysign(1.0, force_mag_tol_ - std::abs(force_mag_goal - force_mag)) * (force_mag_time_now_ - force_mag_time_last_));
//    force_mag_time_last_ = force_mag_time_now_;
//    return force_mag_maintained_ > force_mag_dwell_time_;
//}

///-----------------------------------------------------------------------------
/// USER INPUT/OUTPUT UTILITY FUNCTIONS
///-----------------------------------------------------------------------------

void EmgRTControl::check_manual_stop() {
    std::vector<Key> keys{ Key::LControl, Key::C };
    manual_stop_ = (Keyboard::are_all_keys_pressed(keys, false) | manual_stop_);
}

int EmgRTControl::is_any_num_key_pressed() const {
    std::vector<Key> keys = { Key::Num0, Key::Num1, Key::Num2, Key::Num3, Key::Num4, Key::Num5, Key::Num6, Key::Num7, Key::Num8, Key::Num9 };
    Key key = Keyboard::are_any_keys_pressed(keys);
    switch (key) {
    case Key::Num0:
        return 0;
    case Key::Num1:
        return 1;
    case Key::Num2:
        return 2;
    case Key::Num3:
        return 3;
    case Key::Num4:
        return 4;
    case Key::Num5:
        return 5;
    case Key::Num6:
        return 6;
    case Key::Num7:
        return 7;
    case Key::Num8:
        return 8;
    case Key::Num9:
        return 9;
    default: return -1;
    }
}

///-----------------------------------------------------------------------------
/// FILE READ/WRITE UTILITY FUNCTIONS
///-----------------------------------------------------------------------------

template <typename T>
bool EmgRTControl::read_csv(std::string filename, std::string directory, std::vector<std::vector<T>>& output) {
    output.clear();
    std::string full_filename = directory + "\\" + filename + ".csv";
    std::ifstream input(full_filename);
    input.precision(12);
    if (input.is_open()) {
        std::string csv_line;
        while (std::getline(input, csv_line)) {
            std::istringstream csv_stream(csv_line);
            std::vector<T> row;
            std::string number;
            T data;
            while (std::getline(csv_stream, number, ',')) {
                std::istringstream number_stream(number);
                number_stream >> data;
                row.push_back(data);
            }
            output.push_back(row);
        }
        return true;
    }
    else {
        print("ERROR: File not found.");
        return false;
    }
}
template bool EmgRTControl::read_csv<int>(std::string filename, std::string directory, std::vector<std::vector<int>>& output);
template bool EmgRTControl::read_csv<double>(std::string filename, std::string directory, std::vector<std::vector<double>>& output);

template <typename T>
bool EmgRTControl::write_csv(std::string filename, std::string directory, const std::vector<std::vector<T>>& input) const {
    std::string full_filename = directory + "\\" + filename + ".csv";
    //boost::filesystem::path dir(directory.c_str());
    //boost::filesystem::create_directories(dir);
    create_directory(directory);
    std::ofstream output(full_filename);
    output.precision(12);
    if (output.is_open()) {
        for (int row = 0; row < input.size(); ++row) {
            for (int col = 0; col + 1 < input[row].size(); ++col) {
                output << input[row][col] << ",";
            }
            output << input[row][input[row].size()-1] << std::endl;
        }
        output.close();
        return true;
    }
    else {
        print("ERROR: File not found.");
        return false;
    }
}
template bool EmgRTControl::write_csv<int>(std::string filename, std::string directory, const std::vector<std::vector<int>>& input) const;
template bool EmgRTControl::write_csv<double>(std::string filename, std::string directory, const std::vector<std::vector<double>>& input) const;




void EmgRTControl::store_buffer(const MahiExoIIEmg::EmgDataBuffer& data_buffer, std::vector<std::vector<double>>& data_matrix) {
    if (data_matrix.empty()) {
        for (int i = 0; i < data_buffer.num_channels_; ++i) {
            data_matrix.push_back(std::vector<double>());
        }
    }
    
    if (data_matrix.size() != data_buffer.num_channels_) {
        print("ERROR: Number of channels in data buffer does not match number of rows in data matrix.");
        return;
    }
    std::vector<double> data_buffer_channel;
    for (int i = 0; i < data_buffer.num_channels_; ++i) {
        data_buffer_channel = data_buffer.get_channel(i);
        data_matrix[i].insert(data_matrix[i].end(), data_buffer_channel.begin(), data_buffer_channel.end());
    }
}



void EmgRTControl::write_emg_active_classifier() {

    /// store in a csv-writable data type
    Eigen::MatrixXd calibration_data_eig(tkeo_W_t_.rows(), tkeo_W_t_.cols() + tkeo_w_0_.cols());
    calibration_data_eig << tkeo_W_t_, tkeo_w_0_;
    calibration_data_ = copy_eigmat_to_stdvecvec(calibration_data_eig);

    /// write to csv
    print("Writing calibration data to " + subject_dof_directory_ + "\\" + emg_active_classifier_filename_);
    if (!write_csv<double>(emg_active_classifier_filename_, subject_dof_directory_, calibration_data_)) {
        auto_stop_ = true;
    }

    /// write to csv with time stamp and additional identifying information
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}



void EmgRTControl::load_emg_active_classifier() {

    /// read from csv
    print("Loading calibration data from " + subject_dof_directory_ + "\\" + emg_active_classifier_filename_);
    if (!read_csv<double>(emg_active_classifier_filename_, subject_dof_directory_, calibration_data_)) {
        auto_stop_ = true;
    }

    /// parse csv data into separate variables
    Eigen::MatrixXd calibration_data_eig = copy_stdvecvec_to_eigmat(calibration_data_);
    tkeo_W_t_ = calibration_data_eig.block(0, 0, num_classes_ + 1, meii_.N_emg_);
    tkeo_w_0_ = calibration_data_eig.block(0, meii_.N_emg_, num_classes_ + 1, 1);
}

void EmgRTControl::load_emg_dir_classifier() {

    std::vector<std::vector<double>> emg_dir_classifier;
    print("Loading classifier from " + subject_dof_directory_ + "\\" + emg_dir_classifier_filename_);
    if (!read_csv<double>(emg_dir_classifier_filename_, subject_dof_directory_, emg_dir_classifier)) {
        auto_stop_ = true;
    }

    /// convert classifier to Eigen type matrix
    emg_dir_W_tt_ = copy_stdvecvec_to_eigmat(emg_dir_classifier);

}


void EmgRTControl::save_log_data() {

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

    // save standard robot data log
    //meii_.save_and_clear_robot_log("robot_data_log_" + log_file_name, directory, true);
    robot_log_.save_data("robot_data_log_" + log_file_name, directory, true);
    robot_log_.wait_for_save();
    robot_log_.clear_data();
    //robot_log_ = DataLog("robot_log", false);

    // save emg data log
    emg_log_.save_data("emg_data_log_" + log_file_name, directory, true);
    emg_log_.wait_for_save();
    emg_log_.clear_data();
    //emg_log_ = DataLog("emg_log", false);
    
    //// save condition-specific data that was logged at each trial
    //trial_log_.save_and_clear_data(str_conditions_[condition_] + "_data_log_" + log_file_name, directory, true);
    //trial_log_ = DataLog("trial_log", false);

    //// save debugging data log if used
    //if (debug_log_.get_col_count() > 0 && debug_log_.get_row_count() > 0) {
    //    debug_log_.save_and_clear_data("debug_data_log_" + log_file_name, directory, true);
    //    debug_log_ = DataLog("debug_log", false);
    //}


}

/*void EmgRTControl::init_robot_log() {
    robot_log_.add_col("Time [s]")
        .add_col("MEII EFE Position [rad]").add_col("MEII EFE Velocity [rad/s]").add_col("MEII EFE Commanded Torque [Nm]")
        .add_col("MEII FPS Position [rad]").add_col("MEII FPS Velocity [rad/s]").add_col("MEII FPS Commanded Torque [Nm]")
        .add_col("MEII RPS1 Position [m]").add_col("MEII RPS1 Velocity [m/s]").add_col("MEII RPS1 Commanded Force [N]")
        .add_col("MEII RPS2 Position [m]").add_col("MEII RPS2 Velocity [m/s]").add_col("MEII RPS2 Commanded Force [N]")
        .add_col("MEII RPS3 Position [m]").add_col("MEII RPS3 Velocity [m/s]").add_col("MEII RPS3 Commanded Force [N]")
        .add_col("State");
}*/

//void EmgRTControl::init_emg_log() {
//    emg_log_.add_col("Time [s]");
//        for (int i = 0; i < meii_.N_emg_; ++i) {
//            emg_log_.add_col("Ch. " + std::to_string(i));
//        }
//        for (int i = 0; i < meii_.N_emg_; ++i) {
//            emg_log_.add_col("Filt Ch. " + std::to_string(i));
//        }
//        emg_log_.add_col("State");
//}

//void EmgRTControl::init_trial_log() {
//
//    trial_log_.add_col("Time [s]");
//
//    if (is_cal()) {
//        for (int i = 0; i < num_emg_channels_; ++i) {
//            trial_log_.add_col("TK " + std::to_string(i));
//        }
//        trial_log_.add_col("Target");
//    }
//    else if (is_training()) {        
//        for (int i = 0; i < num_emg_channels_ * num_features_; ++i) {
//            trial_log_.add_col("Feat. " + std::to_string(i));
//        }
//        trial_log_.add_col("Class Label");
//    }
//    else if (is_blind()) {
//        for (int i = 0; i < num_emg_channels_ * num_features_; ++i) {
//            trial_log_.add_col("Feat. " + std::to_string(i));
//        }
//        trial_log_.add_col("Class Label").add_col("Pred. Class Label");
//        for (int k = 0; k < num_classes_; ++k) {
//            trial_log_.add_col("Class " + std::to_string(k + 1) + "Post.");
//        }
//    }
//    else if (is_full()) {
//        for (int i = 0; i < num_emg_channels_ * num_features_; ++i) {
//            trial_log_.add_col("Feat. " + std::to_string(i));
//        }
//        trial_log_.add_col("Class Label").add_col("Pred. Class Label");
//        for (int k = 0; k < num_classes_; ++k) {
//            trial_log_.add_col("Class " + std::to_string(k + 1) + "Post.");
//        }
//    }
//  
//    trial_log_.add_col("Subject").add_col("Hand").add_col("DoF").add_col("Condition");
//
//}

//void EmgRTControl::init_debug_log() {
//    int N = 16;
//    debug_log_.add_col("Time [s]");
//        for (int i = 0; i < N; ++i) {
//            debug_log_.add_col("Sig. " + std::to_string(i));
//        }
//    debug_log_.add_col("Posterior");
//}

void EmgRTControl::log_robot_row() {
    robot_log_.add_row({ timer_.get_elapsed_time().as_seconds(), meii_[0].get_position(), meii_[0].get_velocity(), meii_[0].get_torque(),
        meii_[1].get_position(), meii_[1].get_velocity(), meii_[1].get_torque(),
        meii_[2].get_position(), meii_[2].get_velocity(), meii_[2].get_torque(),
        meii_[3].get_position(), meii_[3].get_velocity(), meii_[3].get_torque(),
        meii_[4].get_position(), meii_[4].get_velocity(), meii_[4].get_torque(), get_current_state() });
}


void EmgRTControl::log_emg_row() {

    //std::vector<double> row;
    //row.push_back(timer_.get_elapsed_time().as_seconds());   
    //for (int i = 0; i < meii_.N_emg_; ++i) {
    //    row.push_back(emg_voltages_[i]);
    //}
    //for (int i = 0; i < meii_.N_emg_; ++i) {
    //    row.push_back(filtered_emg_voltages_[i]);
    //}
    //row.push_back(get_current_state());
    emg_log_.add_row({ timer_.get_elapsed_time().as_seconds(), emg_voltages_[0], emg_voltages_[1], emg_voltages_[2], emg_voltages_[3], emg_voltages_[4], emg_voltages_[5], emg_voltages_[6], emg_voltages_[7],
        filtered_emg_voltages_[0], filtered_emg_voltages_[1], filtered_emg_voltages_[2], filtered_emg_voltages_[3], filtered_emg_voltages_[4], filtered_emg_voltages_[5], filtered_emg_voltages_[6], filtered_emg_voltages_[7], get_current_state() });
}

//void EmgRTControl::log_trial_row() {
//    print("Logging trial data.");
//
//    std::vector<double> row;
//    if (is_cal()) {       
//        for (int j = 0; j < emg_calibration_data_buffer_.length_; ++j) {
//            row.clear();
//            row.push_back(clock_.time());
//            for (int i = 0; i < num_emg_channels_; ++i) {
//                row.push_back(emg_calibration_data_buffer_.data_buffer_[i][j]);
//            }
//            row.push_back(viz_target_num_);
//            row.push_back(subject_number_);
//            row.push_back(hand_num_);
//            row.push_back(dof_);
//            row.push_back(condition_);
//            trial_log_.add_row(row);
//        }   
//    }
//    else if (is_training()) { 
//        row.clear();
//        row.push_back(clock_.get_current_time());
//        for (int i = 0; i < meii_.N_emg_ * num_features_; ++i) {
//            row.push_back(emg_feature_vec_[i]);
//        }
//        row.push_back(class_label_sequence_[current_class_label_idx_]);  
//        row.push_back(subject_number_);
//        row.push_back(hand_num_);
//        row.push_back(dof_);
//        row.push_back(condition_);
//        trial_log_.add_row(row);
//    }
//    else if (is_blind()) {
//        row.clear();
//        row.push_back(clock_.get_current_time());
//        for (int i = 0; i < meii_.N_emg_ * num_features_; ++i) {
//            row.push_back(emg_feature_vec_[i]);
//        }
//        row.push_back(class_label_sequence_[current_class_label_idx_]);
//        row.push_back(pred_class_label_sequence_[current_class_label_idx_]);
//        for (int k = 0; k < num_classes_; ++k) {
//            row.push_back(class_posteriors_[k]);
//        }
//        row.push_back(subject_number_);
//        row.push_back(hand_num_);
//        row.push_back(dof_);
//        row.push_back(condition_);
//        trial_log_.add_row(row);
//    }
//    else if (is_full()) {
//        row.clear();
//        row.push_back(clock_.get_current_time());
//        for (int i = 0; i < meii_.N_emg_ * num_features_; ++i) {
//            row.push_back(emg_feature_vec_[i]);
//        }
//        row.push_back(class_label_sequence_[current_class_label_idx_]);
//        row.push_back(pred_class_label_sequence_[current_class_label_idx_]);
//        for (int k = 0; k < num_classes_; ++k) {
//            row.push_back(class_posteriors_[k]);
//        }
//        row.push_back(subject_number_);
//        row.push_back(hand_num_);
//        row.push_back(dof_);
//        row.push_back(condition_);
//        trial_log_.add_row(row);
//    }
//}
//void EmgRTControl::log_debug_row( std::vector<double> debug_row) {
//    std::vector<double> row;
//    row.push_back(clock_.get_current_time());
//    for (int i = 0; i < debug_row.size(); ++i) {
//        row.push_back(debug_row[i]);
//    }
//    debug_log_.add_row(row);
//}

///-----------------------------------------------------------------------------
/// DATA ANALYSIS UTILITY FUNCTIONS
///-----------------------------------------------------------------------------


Eigen::MatrixXd EmgRTControl::gen_confusion_mat( const std::vector<int>& actual_labels, const std::vector<int>& predicted_labels) const {
    size_t N = actual_labels.size();

    /// assuming the class labels begin at 1
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