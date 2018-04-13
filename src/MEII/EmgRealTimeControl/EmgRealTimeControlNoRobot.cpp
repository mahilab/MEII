#include "EmgRealTimeControl/EmgRealTimeControl.hpp"
#include "MEL/Utility/Windows/Keyboard.hpp"
#include "MEL/Utility/Console.hpp"
#include "MEL/Utility/System.hpp"
#include "MEL/Core/Motor.hpp"
#include "MEL/Math/Functions.hpp"
#include "MEL/Logging/Log.hpp"
#include "EMG/EmgDataCapture.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <random>
#include <numeric>
#include <stdexcept>
#include <ctime>

using namespace mel;

EmgRealTimeControl::EmgRealTimeControl(Daq& daq, Watchdog& watchdog, MesArray& mes, MahiExoII& meii, ctrl_bool& manual_stop, bool virtual_hardware) :
    StateMachine(15),
    virtual_exo_(virtual_hardware),
    virtual_emg_(virtual_hardware),    
    daq_(daq),
    watchdog_(watchdog),
    mes_(mes),
    meii_(meii),
    Ts_(milliseconds(1)),
    timer_(Ts_, Timer::Hybrid),    
    manual_stop_(manual_stop),
    auto_stop_(false),
    exit_program_(false),
    menu_(true),
    end_of_label_sequence_(true),
    active_detector_computed_(false),
    active_detector_(mes_.size(), Ts_),
    current_class_label_idx_(-1),
    directory_share_("file_path"),
    file_name_share_("file_name"),
    cv_results_("cv_results"),
    lda_training_flag_("lda_training_flag"),
    pos_share_("pos_share"),
    vel_share_("vel_share"),
    emg_share_("emg_share"),
    torque_share_("torque_share")
    //tkeo_stat_buffer_(tkeo_stat_buffer_size_)
{ }   


//-----------------------------------------------------------------------------
// "WAIT FOR GUI" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRealTimeControl::sf_wait_for_gui(const NoEventData* data) {
    LOG(Info) << "Waiting for Gui Input";

    // disable robot
    if (meii_.is_enabled()) {
        meii_.disable();
    }

    // disable watchdog
    if (daq_.is_enabled()) {
        watchdog_.stop();
    }

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
void EmgRealTimeControl::sf_init(const NoEventData* data) {
    LOG(Info) << "Initializing for " + str_conditions_long_[condition_] + " of " + str_dofs_long_[dof_];

    // reset global experiment variables
    end_of_label_sequence_ = true;
    class_label_sequence_.clear();
    pred_class_label_sequence_.clear();
    current_class_label_idx_ = -1;
    class_posteriors_ = std::vector<double>(num_classes_, 0.0);
    emg_training_data_.clear();
    prev_emg_training_data_.clear();
    //calibration_data_.clear();
    //tkeo_rest_data_.clear();
    //tkeo_active_data_.clear();
    //tkeo_active_data_.resize(num_classes_);
    std::vector<double> init_flag = { 0.0 };
    lda_training_flag_.write_data(init_flag);

    // initialize classifier
    active_detector_.resize(num_classes_);

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

    // initialize data loggers
    init_robot_log();
    init_emg_log();
    init_trial_log();

    // enable MEII
    if (!virtual_exo_) {
        meii_.enable();
    }

    // confirm start of experiment
    LOG(Info) << "Running EMG Real-Time Control.";

    // start the watchdog
    watchdog_.start();

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
void EmgRealTimeControl::sf_backdrive(const NoEventData* data) {
    LOG(Info) << "Robot Backdrivable"; 

    // initialize local state variables
    bool init_backdrive_time_reached = false;
    Time st_enter_time = timer_.get_elapsed_time();
    std::vector<double> command_torques(meii_.N_aj_, 0.0);
    Key key;

    // enter the control loop
    while (!init_backdrive_time_reached && !menu_ && !exit_program_ && !manual_stop_ && !auto_stop_) {

        // read and reload DAQs
        watchdog_.kick();
        daq_.update_input();

        if (!virtual_exo_) {

            // update robot kinematics
            meii_.update_kinematics();

            // check joint limits
            if (meii_.any_limit_exceeded()) {
                auto_stop_ = true;
                break;
            }

            // set zero torques
            meii_.set_joint_torques(command_torques);

            if (scope_mode_) {

                // write kinematics and motor commands to MelScope
                pos_share_.write_data(meii_.get_anatomical_joint_positions());
                vel_share_.write_data(meii_.get_anatomical_joint_velocities());
                torque_share_.write_data(command_torques);
            }
        }        

        // log data
        log_robot_row();

        if (!virtual_exo_) {

            // write to daq
            daq_.update_output();
        }

        // check for init transparent time reached
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
void EmgRealTimeControl::sf_init_rps(const NoEventData* data) {
    LOG(Info) << "Initialize RPS Mechanism";

    // initialize local state variables
    bool rps_init = false;
    std::vector<double> rps_command_torques(meii_.N_qs_, 0.0);
    std::vector<double> command_torques(meii_.N_rj_, 0.0);

    // initialize rps initialization position controller
    meii_.set_rps_control_mode(0);
    meii_.update_kinematics();
    meii_.rps_init_par_ref_.start(meii_.get_wrist_parallel_positions(), timer_.get_elapsed_time());

    // enter the control loop
    while (!rps_init && !menu_ && !exit_program_ && !manual_stop_ && !auto_stop_) {

        // read and reload DAQs
        watchdog_.kick();
        daq_.update_input();

        if (!virtual_exo_) {

            // update robot kinematics
            meii_.update_kinematics();

            // check joint limits
            if (meii_.any_limit_exceeded()) {
                auto_stop_ = true;
                break;
            }

            // set zero torque for elbow and forearm joints (joints 0 and 1)           
            meii_[0].set_torque(0.0);
            meii_[1].set_torque(0.0);

            // run rps position control
            rps_command_torques = meii_.set_rps_pos_ctrl_torques(meii_.rps_init_par_ref_, timer_.get_elapsed_time());
            std::copy(rps_command_torques.begin(), rps_command_torques.end(), command_torques.begin() + 2);

            if (scope_mode_) {

                // write kinematics and motor commands to MelScope
                pos_share_.write_data(meii_.get_anatomical_joint_positions());
                vel_share_.write_data(meii_.get_anatomical_joint_velocities());
                torque_share_.write_data(command_torques);
            }
        }       

        if (!virtual_exo_) {

            // check for rps mechanism in intialization position
            rps_init = meii_.check_rps_init();
        }
        else {
            rps_init = true;
        }

        // log data
        log_robot_row();
       

        if (!virtual_exo_) {

            // write to daq
            daq_.update_output();
        }

        // check if return to main menu selected in Unity
        game_.set_experiment_conditions(hand_num_, dof_, num_classes_, condition_, menu_);

        // check for program exit command from user
        check_exit_program();

        // wait for the next clock cycle
        timer_.wait();
    }

    // stop the rps intialization position controller
    meii_.rps_init_par_ref_.stop();

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
void EmgRealTimeControl::sf_to_center(const NoEventData* data) {
    LOG(Info) << "Go to Center";
    
    // initialize local state variables
    bool target_reached = false;
    std::vector<double> command_torques(meii_.N_aj_, 0.0);

    // initialize rps position controller mode and reference
    meii_.set_rps_control_mode(1);
    if (meii_.anat_ref_.is_started()) {
        meii_.anat_ref_.set_ref(center_pos_, timer_.get_elapsed_time());
    }
    else {
        meii_.anat_ref_.start(center_pos_, meii_.get_anatomical_joint_positions(), timer_.get_elapsed_time());
    }

    // write to Unity
    game_.set_target(0);

    // enter the control loop
    while (!target_reached && !menu_ && !exit_program_ && !manual_stop_ && !auto_stop_) {

        // read and reload DAQs
        watchdog_.kick();
        daq_.update_input();

        if (!virtual_exo_) {

            // update robot kinematics
            meii_.update_kinematics();

            // check joint limits
            if (meii_.any_limit_exceeded()) {
                auto_stop_ = true;
                break;
            }

            // run position control
            command_torques = meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, timer_.get_elapsed_time());

            if (scope_mode_) {

                // write kinematics and motor commands to MelScope
                pos_share_.write_data(meii_.get_anatomical_joint_positions());
                vel_share_.write_data(meii_.get_anatomical_joint_velocities());
                torque_share_.write_data(command_torques);
            }
        }

        if (!virtual_exo_) {

            // check for target reached
            target_reached = meii_.check_neutral_anat_pos(center_pos_, { 1, 1, 1, 1, 0 });
        }
        else {
            target_reached = true;
        }

        // log robot data
        log_robot_row();        

        if (!virtual_exo_) {

            // write to daq
            daq_.update_output();
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
void EmgRealTimeControl::sf_hold_center(const NoEventData* data) {
    LOG(Info) << "Hold at Center";

    // initialize global variables   
    if (is_cal()) {
        hold_center_time_ = seconds(3.0);
        //emg_calibration_data_buffer_.clear();
        //meii_.reset_emg_signal_processing();
    }
    else {
        hold_center_time_ = seconds(1.0);
        //tkeo_stat_buffer_.clear();
        //tkeo_stat_buffer_.resize(tkeo_stat_buffer_size_);
    }

    // initialize local state variables
    bool hold_center_time_reached = false;
    bool rest_state_reached = false;
    Time st_enter_time = timer_.get_elapsed_time();
    std::vector<double> command_torques(meii_.N_aj_, 0.0);
    //std::vector<double> tkeo_vec(num_emg_channels_, 0.0);
    //std::vector<double> filtered_tkeo_vec(num_emg_channels_, 0.0);
    bool tkeo_buffer_full = false;
    //emg_voltages_ = std::vector<double>(num_emg_channels_, 0.0);
    //filtered_emg_voltages_ = std::vector<double>(num_emg_channels_, 0.0);
    //mes_tkeo_env_ = std::vector<double>(num_emg_channels_, 0.0);

    // write to Unity
    game_.set_target(0);

    // enter the control loop
    while (!rest_state_reached && !menu_ && !exit_program_ && !manual_stop_ && !auto_stop_) {

        // read and reload DAQs
        watchdog_.kick();
        daq_.update_input();

        if (!virtual_exo_) {

            // update robot kinematics
            meii_.update_kinematics();

            // check joint limits
            if (meii_.any_limit_exceeded()) {
                auto_stop_ = true;
                break;
            }

            // run position control
            command_torques = meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, timer_.get_elapsed_time());

            if (scope_mode_) {

                // write kinematics and motor commands to MelScope
                pos_share_.write_data(meii_.get_anatomical_joint_positions());
                vel_share_.write_data(meii_.get_anatomical_joint_velocities());
                torque_share_.write_data(command_torques);
            }
        }
      
        // get measured emg voltages and log them
        //meii_.update_emg();
        //emg_voltages_ = meii_.get_mes_raw();
        //filtered_emg_voltages_ = meii_.get_mes_dm();
        log_emg_row();
        if (scope_mode_) {

            //emg_share_.write_data(filtered_emg_voltages_);
        }

        // compute the teager-kaiser metric online with rectification and filtering
        //meii_.tko_.tkeo(filtered_emg_voltages_, tkeo_vec);
        //std::for_each(tkeo_vec.begin(), tkeo_vec.end(), [](double x) { return std::abs(x); });
        //meii_.tkeo_butter_lp_.filter(tkeo_vec, filtered_tkeo_vec);
        //mes_tkeo_env_ = meii_.get_mes_tkeo_env();

        // check for hold time reached
        hold_center_time_reached = check_wait_time_reached(hold_center_time_, st_enter_time, timer_.get_elapsed_time());

        //if (is_cal()) {

        //    // store calibration data in buffer
        //    //emg_calibration_data_buffer_.push_back(mes_tkeo_env_);

        //    if (hold_center_time_reached) {
        //        rest_state_reached = true;
        //    }
        //}
        //else {

        //    // compute tkeo detector statistic and fill buffer
        //    //tkeo_stat_ = tkeo_detector(mes_tkeo_env_);


        //    if (hold_center_time_reached) {

        //        // check for detection buffer full
        //        if (!tkeo_buffer_full) {
        //            tkeo_buffer_full = check_wait_time_reached(tkeo_buffer_fill_time_, st_enter_time, timer_.get_elapsed_time());
        //        }
        //        else if (tkeo_buffer_full) {

        //            if (!virtual_emg_) {
        //                rest_state_reached = tkeo_stat_ > rest_tkeo_threshold_;
        //                rest_state_reached = true;
        //            }
        //            else {
        //                rest_state_reached = true;
        //            }
        //        }
        //    }
        //}
        
        // log robot data
        log_robot_row();

        if (!virtual_exo_) {

            // write to daq
            daq_.update_output();
        }

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
            //store_buffer(emg_calibration_data_buffer_, tkeo_rest_data_);
            log_trial_row();
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
void EmgRealTimeControl::sf_hold_for_input(const NoEventData* data) {
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
    std::vector<double> command_torques(meii_.N_aj_, 0.0);

    // initialization actions upon first entry to ST_HOLD_FOR_INPUT
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

            // read in emg calibration data from file
            load_emg_active_classifier();

            // ask user if they want to use the existing training data
            print("Press Enter to collect new training data, or press L to Load previous training data...");
        }
        else if (is_testing()) {

            // read in emg calibration data from file
            load_emg_active_classifier();

            // read in classifier from file
            load_emg_dir_classifier();

            // set pre-determined number of testing observations
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

    // initialization actions upon end of label sequence
    else { 
        if (is_cal()) {

            // compute statistics from calibration data
            //tkeo_model_estimation();

            // write calibration data to file
            //write_emg_active_classifier();
            //finished = true;
            //print("tkeo and file write done");
        }
        else if (is_training()) {

            // write training data to file
            if (!write_csv<double>(training_data_filename_, subject_dof_directory_, emg_training_data_)) {
                auto_stop_ = true;
            }

            // go directly to ST_FINISH
            finished = true;         
            
        }
        else if (is_testing()) {

            // print classification performance report
            Eigen::MatrixXd confusion_mat = gen_confusion_mat(class_label_sequence_, pred_class_label_sequence_);
            print("Confusion Matrix for Last Round of Trials: ");
            std::cout << confusion_mat << std::endl;

            // going directly to ST_FINISH
            finished = true;
        }
        else {
            LOG(Error) << "State transition undefined. Going to ST_FAULT_STOP.";
            auto_stop_ = true;
        } 
    }
    
    // write to Unity
    game_.set_target(0);

    // enter the control loop
    while ( !present_more_targets && !finished && !menu_ && !exit_program_ && !manual_stop_ && !auto_stop_) {

        // read and reload DAQs
        watchdog_.kick();
        daq_.update_input();

        if (!virtual_exo_) {

            // update robot kinematics
            meii_.update_kinematics();

            // check joint limits
            if (meii_.any_limit_exceeded()) {
                auto_stop_ = true;
                break;
            }

            // run position control
            command_torques = meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, timer_.get_elapsed_time());

            if (scope_mode_) {

                // write kinematics and motor commands to MelScope
                pos_share_.write_data(meii_.get_anatomical_joint_positions());
                vel_share_.write_data(meii_.get_anatomical_joint_velocities());
                torque_share_.write_data(command_torques);
            }
        }       

        // check for external input upon first entry to ST_HOLD_FOR_INPUT
        if (!end_of_label_sequence_) {
            if (is_cal()) {
                LOG(Error) << "Should not be reached. Going to ST_FAULT_STOP.";
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
                LOG(Error) << "Should not be reached. Going to ST_FAULT_STOP.";
                auto_stop_ = true;
            }
            else {
                LOG(Error) << "Condition number was set improperly. Going to ST_FAULT_STOP.";
                auto_stop_ = true;
            }
        }

        // check for external input upon end of label sequence
        else {
            if (is_cal()) {
                LOG(Error) << "Should not be reached. Going to ST_FAULT_STOP.";
                auto_stop_ = true;
            }
            else if (is_training()) {
                LOG(Error) << "Should not be reached. Going to ST_FAULT_STOP.";
                auto_stop_ = true;
            }
            else if (is_testing()) {
                LOG(Error) << "Should not be reached. Going to ST_FAULT_STOP.";
                auto_stop_ = true;
            }
            else {
                LOG(Error) << "Condition number was set improperly. Going to ST_FAULT_STOP.";
                auto_stop_ = true;
            }       
        }

        // log data
        log_robot_row();

        if (!virtual_exo_) {

            // write to daq
            daq_.update_output();
        }

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
void EmgRealTimeControl::sf_calibration(const NoEventData* data) {
    LOG(Info) << "Begin Calibration of Active/Rest Detection";
      

    // methodological and control options - to be changed as desired
    Time mes_rest_capture_period = seconds(1);
    Time mes_active_capture_period = seconds(3);
    Time mes_active_period = milliseconds(200);

    // initialize methodological and control variables - not to be changed        
    std::vector<Key> active_keys = { Key::Num1, Key::Num2, Key::Num3, Key::Num4 };
    std::size_t mes_rest_capture_window_size = mes_rest_capture_period.as_seconds() / Ts_.as_seconds();
    std::size_t mes_active_capture_window_size = mes_active_capture_period.as_seconds() / Ts_.as_seconds();
    mes_.resize_buffer(std::max(mes_rest_capture_window_size, mes_active_capture_window_size));
    std::size_t mes_active_window_size = mes_active_period.as_seconds() / Ts_.as_seconds();

    // initialize data containers   
    std::vector<double> command_torques(meii_.N_aj_, 0.0);
    int active_state = 0;
    int number_keypress = -1;

    // make MelShares
    MelShare ms_mes_tkeo_env("melscope_mes_tkeo_env");
    MelShare ms_active_prob("melscope_active_prob");
    MelShare ms_pred_state("melscope_pred_state");

    // construct clock to regulate keypresses
    Clock keypress_refract_clock;
    Time keypress_refract_time = seconds(1);  

    // promt the user for input
    print("Press 'R' to capture rest data. Press 'A + class label #' to capture active data.");
    print("Press 'Space + R' to clear rest data. Press 'Space + A + class label #' to clear active data.");
    print("Press 'C' to compute classification model and begin detection. Press 'Enter' to save classifier and exit.");

    while (!active_detector_computed_ && !menu_ && !exit_program_ && !manual_stop_ && !auto_stop_) {

        // read and reload DAQs
        watchdog_.kick();
        daq_.update_input();

        if (!virtual_exo_) {

            // update robot kinematics
            meii_.update_kinematics();

            // check joint limits
            if (meii_.any_limit_exceeded()) {
                auto_stop_ = true;
                break;
            }

            // run position control
            command_torques = meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, timer_.get_elapsed_time());

            if (scope_mode_) {

                // write kinematics and motor commands to MelScope
                pos_share_.write_data(meii_.get_anatomical_joint_positions());
                vel_share_.write_data(meii_.get_anatomical_joint_velocities());
                torque_share_.write_data(command_torques);
            }
        }

        // emg signal processing
        mes_.update();

        // write to emg data log
        log_emg_row();

        // predict state
        active_detector_.update(mes_.get_tkeo_envelope());
        active_state = active_detector_.get_class();
        
        // clear rest data
        if (Keyboard::are_all_keys_pressed({ Key::Space, Key::R })) {
            if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                for (std::size_t k = 0; k < num_classes_; ++k) {
                    active_detector_.clear_training_data(k, 0);
                }
                LOG(Info) << "Cleared rest data.";
                keypress_refract_clock.restart();
            }
        }

        // capture rest data
        if (Keyboard::is_key_pressed(Key::R)) {
            if (mes_.is_buffer_full()) {
                if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                    for (std::size_t k = 0; k < num_classes_; ++k) {
                        active_detector_.add_training_data(k, 0, mes_.get_tkeo_env_buffer_data(mes_rest_capture_window_size));
                    }
                    LOG(Info) << "Collected rest data.";
                    keypress_refract_clock.restart();
                }
            }
        }

        // clear active data
        for (std::size_t k = 0; k < num_classes_; ++k) {
            if (Keyboard::are_all_keys_pressed({ Key::Space, Key::A, active_keys[k] })) {
                if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                    active_detector_.clear_training_data(k, 1);
                    LOG(Info) << "Cleared active data " + stringify(k + 1) + ".";
                    keypress_refract_clock.restart();
                }
            }
        }

        // capture active data
        for (std::size_t k = 0; k < num_classes_; ++k) {
            if (Keyboard::are_all_keys_pressed({ Key::A, active_keys[k] })) {
                if (mes_.is_buffer_full()) {
                    if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                        active_detector_.add_training_data(k, 1, find_sum_max_window(mes_.get_tkeo_env_buffer_data(mes_active_capture_window_size), mes_active_window_size));
                        LOG(Info) << "Collected active data " + stringify(k + 1) + ".";
                        keypress_refract_clock.restart();
                    }
                }
            }
        }

        // compute the active/rest classifier
        if (Keyboard::is_key_pressed(Key::C)) {
            if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                if (active_detector_.train());
                LOG(Info) << "Computed new active/rest classifier.";
                keypress_refract_clock.restart();
            }
        }

        // update visualization target
        number_keypress = is_any_num_key_pressed();
        if (number_keypress >= 0) {
            if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                if (number_keypress > num_classes_)
                    number_keypress = -1;
                game_.set_target(number_keypress);
                keypress_refract_clock.restart();
            }
        }

        // exit calibration and save the computed classifier
        if (Keyboard::is_key_pressed(Key::Enter)) {
            active_detector_computed_ = true;
        }

        // write to MelShares
        ms_mes_tkeo_env.write_data(mes_.get_tkeo_envelope());
        ms_pred_state.write_data({ (double)active_state });
       

        if (!virtual_exo_) {

            // write to daq
            daq_.update_output();
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
void EmgRealTimeControl::sf_training(const NoEventData* data) {
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
    emg_training_data_.clear();

    // generate new class labels
    std::vector<int> new_class_labels = rand_shuffle_class_labels(num_observations_per_class);
    for (int i = 0; i < new_class_labels.size(); ++i) {
        class_label_sequence_.push_back(new_class_labels[i]);
    }

    // initialize data containers
    std::vector<double> command_torques(meii_.N_aj_, 0.0);
    //std::vector<int> pred_class(num_classes_);
    int active_state = 0;
    int prev_active_state = 0;


    
    //Key key;

    // make MelShares
    MelShare ms_mes_tkeo_env("melscope_mes_tkeo_env");
    MelShare ms_pred_state("melscope_pred_state");

    // construct clock to regulate keypresses
    Clock keypress_refract_clock;
    Time keypress_refract_time = seconds(1);   

    // load active detector if it exists
    if (!load_emg_active_classifier()) {
        LOG(Error) << "Failed to load active detector. Cannot run training.";
        auto_stop_ = true;
    }

    // load previous training data if it exists
    load_emg_dir_training_data();


    // write training data to file
    //if (!write_csv<double>(training_data_filename_, subject_dof_directory_, emg_training_data_)) {
    //    auto_stop_ = true;
    //}

    // go directly to ST_FINISH
    //finished = true;

    // prompt user for input
    print("Press 'Enter' to begin checking for activity.");

    while (!menu_ && !exit_program_ && !manual_stop_ && !auto_stop_) {

        // read and reload DAQs
        watchdog_.kick();
        daq_.update_input();

        if (!virtual_exo_) {

            // update robot kinematics
            meii_.update_kinematics();

            // check joint limits
            if (meii_.any_limit_exceeded()) {
                auto_stop_ = true;
                break;
            }

            // run position control
            command_torques = meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, timer_.get_elapsed_time());

            if (scope_mode_) {

                // write kinematics and motor commands to MelScope
                pos_share_.write_data(meii_.get_anatomical_joint_positions());
                vel_share_.write_data(meii_.get_anatomical_joint_velocities());
                torque_share_.write_data(command_torques);
            }
        }

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
                collect_emg_dir_sample();
            }
        }


        //if (!more_training_data) {
        //    key = Keyboard::are_any_keys_pressed({ Key::Enter, Key::L });
        //    switch (key) {
        //    case Key::Enter:
        //        more_training_data = true;
        //        print("How many observations per class would you like to collect? (0-9)");
        //        break;
        //    case Key::L:
        //        print("Loading previous training data from " + subject_dof_directory_ + "\\" + training_data_filename_);
        //        if (!read_csv<double>(training_data_filename_, subject_dof_directory_, prev_emg_training_data_)) {
        //            auto_stop_ = true;
        //            break;
        //        }
        //        emg_training_data_ = prev_emg_training_data_;
        //        more_training_data = true;
        //        print("How many observations per class would you like to collect? (0-9)");
        //        break;
        //    }
        //}
        //else {

        //    num_observations_per_class = is_any_num_key_pressed();
        //    if (num_observations_per_class > 0) {
        //        present_more_targets = true;
        //    }
        //    else if (num_observations_per_class == 0) {
        //        more_training_data = false;
        //        finished = true;
        //    }
        //}

        // write to MelShares
        ms_mes_tkeo_env.write_data(mes_.get_tkeo_envelope());
        ms_pred_state.write_data({ (double)active_state });


        if (!virtual_exo_) {

            // write to daq
            daq_.update_output();
        }

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
    //else if (present_more_targets) {
    //    end_of_label_sequence_ = false;
    //    if (!is_cal()) {
    //        std::vector<int> new_class_labels = rand_shuffle_class_labels(num_observations_per_class);
    //        for (int i = 0; i < new_class_labels.size(); ++i) {
    //            class_label_sequence_.push_back(new_class_labels[i]);
    //        }
    //    }
    //    event(ST_PRESENT_TARGET);
    //}
    else {
        LOG(Error) << "State transition undefined. Going to ST_FAULT_STOP.";
        event(ST_FAULT_STOP);
    }
}


//-----------------------------------------------------------------------------
// "PRESENT TARGET" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRealTimeControl::sf_present_target(const NoEventData* data) {
    LOG(Info) << "Present Target";

    // initialize global variables
    //emg_calibration_data_buffer_.clear();
    //emg_classification_data_buffer_.clear();
    meii_.reset_emg_signal_processing();
    //tkeo_stat_buffer_.clear();
    //tkeo_stat_buffer_.resize(tkeo_stat_buffer_size_);
    //emg_voltages_ = std::vector<double>(num_emg_channels_, 0.0);
    //filtered_emg_voltages_ = std::vector<double>(num_emg_channels_, 0.0);
    //mes_tkeo_env_ = std::vector<double>(num_emg_channels_, 0.0);

    // initialize local state variables
    bool force_mag_reached = false;
    bool active_state_reached = false;
    bool detector_expired = false;
    bool mvc_started = false;
    Time mvc_start_time = seconds(0.0);
    bool mvc_completed = false;
    bool keep_mvc = false;
    bool redo_mvc = false;
    //std::vector<double> tkeo_vec(num_emg_channels_, 0.0);
    //std::vector<double> filtered_tkeo_vec(num_emg_channels_, 0.0);
    std::vector<double> command_torques(meii_.N_aj_, 0.0);
    double force_mag = 0.0;
    double tkeo_active_mag = 0.0;
    bool tkeo_buffer_full = false;
    Time st_enter_time = timer_.get_elapsed_time();
    
    if (is_cal()) {
        game_.set_effort_range(0.0, force_mag_goal_);
    }
    else {
        game_.set_effort_range(0.0, 1.0);
    }

    // initialize magnitude force checking algorithm
    //force_mag_maintained_ = 0.0;
    //force_mag_time_now_ = clock_.get_current_time();
    //force_mag_time_last_ = clock_.get_current_time();

    // read from target sequence and write to Unity
    game_.set_target(class_label_sequence_[current_class_label_idx_]);
    //set_viz_target_num(class_label_sequence_[current_class_label_idx_]);
    //ms_target_.write_data(viz_target_num_);

    print("Target class label: " + std::to_string(class_label_sequence_[current_class_label_idx_]));

    if (is_cal()) {

        // ask user to indicate when MVC has started
        print("Press ENTER when MVC has started.");
    }

    // enter the control loop
    while (!end_of_label_sequence_ && !keep_mvc && !redo_mvc && !active_state_reached && !detector_expired && !menu_ && !exit_program_ && !manual_stop_ && !auto_stop_) {

        // read and reload DAQs
        watchdog_.kick();
        daq_.update_input();

        if (!virtual_exo_) {

            // update robot kinematics
            meii_.update_kinematics();

            // check joint limits
            if (meii_.any_limit_exceeded()) {
                auto_stop_ = true;
                break;
            }

            // run position control
            command_torques = meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, timer_.get_elapsed_time());

            if (scope_mode_) {

                // write kinematics and motor commands to MelScope
                pos_share_.write_data(meii_.get_anatomical_joint_positions());
                vel_share_.write_data(meii_.get_anatomical_joint_velocities());
                torque_share_.write_data(command_torques);
            }
        }

        // get measured emg voltages and log them
        meii_.update_emg();
        //emg_voltages_ = meii_.get_mes_raw();
        //filtered_emg_voltages_ = meii_.get_mes_dm();
        log_emg_row();
        if (scope_mode_) {
            //emg_share_.write_data(filtered_emg_voltages_);
        }

        // compute the teager-kaiser metric online with rectification and filtering and store in calibration buffer       
        //meii_.tko_.tkeo(filtered_emg_voltages_, tkeo_vec);
        //std::for_each(tkeo_vec.begin(), tkeo_vec.end(), [](double x) { return std::abs(x); });
        //meii_.tkeo_butter_lp_.filter(tkeo_vec, filtered_tkeo_vec);
        //mes_tkeo_env_ = meii_.get_mes_tkeo_env();

        // store emg data in appropriate buffer
        //if (is_cal()) {
        //    if (mvc_started && !mvc_completed) {
        //        //emg_calibration_data_buffer_.push_back(mes_tkeo_env_);
        //    }
        //}
        //else {
        //    //emg_classification_data_buffer_.push_back(filtered_emg_voltages_);
        //}

        // measure interaction force for specified dof(s)
        force_mag = measure_task_force(command_torques, class_label_sequence_[current_class_label_idx_], dof_, condition_);

        // check for break conditions
        if (is_cal()) {

            //// check for user input
            //if (!mvc_started) {
            //    if (Keyboard::is_key_pressed(Key::Enter)) {
            //        mvc_started = true;
            //        mvc_start_time = timer_.get_elapsed_time();
            //        print("Holding for contraction.");
            //    }
            //}
            //else {
            //    if (!mvc_completed) {
            //        mvc_completed = check_wait_time_reached(wait_mvc_time_, mvc_start_time, timer_.get_elapsed_time());
            //        if (mvc_completed) {
            //            print("Do you want to keep the calibration data for this contraction (Y/N)?");
            //        }
            //    }
            //    else {
            //        Key key = Keyboard::are_any_keys_pressed({ Key::Y, Key::N });
            //        switch (key) {
            //        case Key::Y:
            //            keep_mvc = true;
            //            break;
            //        case Key::N:
            //            redo_mvc = true;
            //            break;
            //        }
            //    }
            //}
        }
        else {

            //// compute tkeo detector statistic and fill buffer
            //tkeo_stat_ = tkeo_detector(mes_tkeo_env_);

            //// check for detection buffer full
            //if (!tkeo_buffer_full) {
            //    tkeo_buffer_full = check_wait_time_reached(tkeo_buffer_fill_time_, st_enter_time, timer_.get_elapsed_time());
            //    tkeo_active_mag = 0.0;
            //}
            //else {

            //    if (!virtual_emg_) {
            //        active_state_reached = tkeo_stat_ < (1.0 - active_tkeo_threshold_);
            //        tkeo_active_mag = (1.0 - tkeo_stat_) * force_mag_goal_;

            //        if (is_testing()) {

            //            // check for detector expiry
            //            detector_expired = check_wait_time_reached(detection_expire_time_, st_enter_time, timer_.get_elapsed_time());
            //        }
            //    }
            //    else {
            //        active_state_reached = true;
            //        tkeo_active_mag = force_mag_goal_;
            //    }
            //}
        }     

        // write to unity
        if (is_cal()) {
            //game_.set_effort(force_mag);
            //ms_effort_.write_data(force_mag);
        }
        else {
            //ms_effort_.write_data(tkeo_active_mag);
        }            

        // log data
        log_robot_row();

        if (!virtual_exo_) {

            // write to daq
            daq_.update_output();
        }

        // check if return to main menu selected in Unity
        game_.set_experiment_conditions(hand_num_, dof_, num_classes_, condition_, menu_);

        // check for program exit command from user
        check_exit_program();

        // wait for the next clock cycle
        timer_.wait();
    }

    game_.set_effort(0.0);
    //tkeo_active_mag = 0.0;    
    //ms_effort_.write_data(tkeo_active_mag);


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
    else if (keep_mvc) {
        //store_buffer(emg_calibration_data_buffer_, tkeo_active_data_[current_class_label_idx_]);
        log_trial_row();
        event(ST_TO_CENTER);
    }
    else if (redo_mvc) {
        event(ST_PRESENT_TARGET);
    }
    else if (active_state_reached) {      
        if (process_emg()) {
            if (is_training()) {
                log_trial_row();
                event(ST_TO_CENTER);
            }
            else {
                classify();
                if (is_blind()) {
                    log_trial_row();
                    event(ST_TO_CENTER);
                }
                else {
                    log_trial_row();
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
            //emg_feature_vec_ = std::vector<double>(num_emg_channels_ * num_features_, 0.0);
            emg_feature_vec_ = std::vector<double>(meii_.get_emg_channel_count() * num_features_, 0.0);
            class_posteriors_ = std::vector<double>(num_classes_, 0.0);
            if (is_blind()) {
                log_trial_row();
                event(ST_TO_CENTER);
            }
            else {
                log_trial_row();
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
void EmgRealTimeControl::sf_to_target(const NoEventData* data) {
    LOG(Info) << "Go to Target";

    // initialize local state variables
    bool target_reached = false;
    std::vector<double> command_torques(meii_.N_aj_, 0.0);

    // set new reference position as predicted label
    std::vector<double> target_pos = get_target_position(pred_class_label_sequence_[current_class_label_idx_]);
    meii_.anat_ref_.set_ref(target_pos, timer_.get_elapsed_time());

    // write to Unity actual label
    game_.set_target(class_label_sequence_[current_class_label_idx_]);
    //set_viz_target_num(class_label_sequence_[current_class_label_idx_]);
    //ms_target_.write_data(viz_target_num_);

    // enter the control loop
    while (!target_reached && !menu_ && !exit_program_ && !manual_stop_ && !auto_stop_) {

        // read and reload DAQs
        watchdog_.kick();
        daq_.update_input();

        if (!virtual_exo_) {

            // update robot kinematics
            meii_.update_kinematics();

            // check joint limits
            if (meii_.any_limit_exceeded()) {
                auto_stop_ = true;
                break;
            }

            // run position control
            command_torques = meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, timer_.get_elapsed_time());

            if (scope_mode_) {

                // write kinematics and motor commands to MelScope
                pos_share_.write_data(meii_.get_anatomical_joint_positions());
                vel_share_.write_data(meii_.get_anatomical_joint_velocities());
                torque_share_.write_data(command_torques);
            }
        }

        if (!virtual_exo_) {

            // check for target reached
            target_reached = meii_.check_goal_anat_pos(target_pos, { 1, 1, 1, 1, 0 });
        }
        else {
            target_reached = true;
        }

        // log data
        log_robot_row();

        if (!virtual_exo_) {

            // write to daq
            daq_.update_output();
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
void EmgRealTimeControl::sf_hold_target(const NoEventData* data) {
    LOG(Info) << "Hold at Target"; 

    // initialize local state variables
    bool hold_target_time_reached = false;
    Time st_enter_time = timer_.get_elapsed_time();
    std::vector<double> command_torques(meii_.N_aj_, 0.0);


    // enter the control loop
    while (!hold_target_time_reached && !menu_ && !exit_program_ && !manual_stop_ && !auto_stop_) {

        // read and reload DAQs
        watchdog_.kick();
        daq_.update_input();

        if (!virtual_exo_) {

            // update robot kinematics
            meii_.update_kinematics();

            // check joint limits
            if (meii_.any_limit_exceeded()) {
                auto_stop_ = true;
                break;
            }

            // run position control
            command_torques = meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, timer_.get_elapsed_time());

            if (scope_mode_) {

                // write kinematics and motor commands to MelScope
                pos_share_.write_data(meii_.get_anatomical_joint_positions());
                vel_share_.write_data(meii_.get_anatomical_joint_velocities());
                torque_share_.write_data(command_torques);
            }
        }

        // check for hold time reached
        hold_target_time_reached = check_wait_time_reached(hold_target_time_, st_enter_time, timer_.get_elapsed_time());

        // log data
        log_robot_row();

        if (!virtual_exo_) {

            // write to daq
            daq_.update_output();
        }

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
void EmgRealTimeControl::sf_finish(const NoEventData* data) {
    LOG(Info) << "Finishing Experiment";

    // intialize local state variables
    //bool menu_selected = false;
    std::vector<double> lda_training_complete = { 0.0 };
    bool python_return = false;
    bool cv_score_achieved = true;

    // disable robot
    if (meii_.is_enabled()) {
        meii_.disable();
    }

    // disable watchdog
    if (daq_.is_enabled()) {
        watchdog_.stop();
    }

    // save data
    save_log_data();

    if (is_cal()) {
        write_emg_active_classifier();
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

void EmgRealTimeControl::sf_stop(const NoEventData* data) {
    LOG(Info) << "Exiting Program";

    // disable robot
    if (meii_.is_enabled()) {
        meii_.disable();
    }

}


//-----------------------------------------------------------------------------
// "FAULT STOP" STATE FUNCTION
//-----------------------------------------------------------------------------

void EmgRealTimeControl::sf_fault_stop(const NoEventData* data) {
    LOG(Info) << "Program Stopped with Potential Fault";

    // disable robot
    if (meii_.is_enabled()) {
        meii_.disable();
    }

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
            if (!write_csv<double>(training_data_filename_, subject_dof_directory_, emg_training_data_)) {
            }

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
// EMG REAL-TIME CONTROL UTILITY FUNCTIONS
//-----------------------------------------------------------------------------

bool EmgRealTimeControl::collect_emg_dir_sample() {
    LOG(Info) << "Collect EMG Direction Classifier Sample with Label";

    // default return value to true; only write false if error encountered
    bool emg_data_processed = true; 

    // extract features from EMG data
    emg_training_data_row_ = feature_extract();
    
    // check for successful data processing
    bool are_nan_features = false;
    std::vector<int> nan_idx;
    for (int i = 0; i < emg_training_data_row_.size(); ++i) {
        if (std::isnan(emg_training_data_row_[i])) {
            are_nan_features = true;
            emg_training_data_row_[i] = 1.0;
            nan_idx.push_back(i);
        }
    }
    if (are_nan_features) {
        LOG(Error) << "NaN feature(s) detected.";
        print(nan_idx);
        emg_data_processed = false;
    }

    // store feature array in training data set
    if (is_training()) {
        emg_training_data_row_.push_back(class_label_sequence_[current_class_label_idx_]);
        emg_training_data_.push_back(emg_training_data_row_);
    }
    
    return emg_data_processed;
}

void EmgRealTimeControl::classify() {
    print("Classifying EMG Activation");

    // convert feature vector to Eigen type and add dummy feature
    Eigen::VectorXd emg_feature_vec_eig(emg_feature_vec_.size() + 1);
    for (int i = 0; i < emg_feature_vec_.size(); ++i) {
        emg_feature_vec_eig(i) = emg_feature_vec_[i];
    }
    emg_feature_vec_eig.tail(1) = Eigen::VectorXd::Ones(1);

    // compute the output of the linear model
    Eigen::VectorXd y;
    y = emg_dir_W_tt_ * emg_feature_vec_eig;

    // compute the posterior probabilities for each class    
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
    
    // make movement direction decision
    pred_class_label_sequence_.push_back(std::distance(class_posteriors_.begin(), std::max_element(class_posteriors_.begin(), class_posteriors_.end())) + 1);
    
    print("Predicted class label: " + std::to_string(pred_class_label_sequence_[current_class_label_idx_]));
}

//double EmgRealTimeControl::tkeo_detector(const std::vector<double>& sample_vec) {
//    Eigen::VectorXd sample = copy_stdvec_to_eigvec(sample_vec);
//    Eigen::VectorXd y = tkeo_W_t_ * sample + tkeo_w_0_;
//    double rest_posterior = softmax(y, 0);
//    tkeo_stat_buffer_.push_back(rest_posterior);
//    std::vector<double> tkeo_stat_vec(tkeo_stat_buffer_size_);
//    for (size_t i = 0; i < tkeo_stat_buffer_size_; ++i) {
//        tkeo_stat_vec[i] = tkeo_stat_buffer_[i];
//    }
//    return std::accumulate(tkeo_stat_vec.begin(), tkeo_stat_vec.end(), 0.0) / (double) tkeo_stat_buffer_size_;
//}



//-----------------------------------------------------------------------------
// EMG PATTERN RECOGNITION TRAINING UTILITY FUNCTIONS
//-----------------------------------------------------------------------------


//void EmgRealTimeControl::tkeo_model_estimation() {
//
//    // data dimension
//    size_t D = num_emg_channels_;
//    Eigen::MatrixXd tkeo_sample_means(D, num_classes_ + 1);
//    Eigen::MatrixXd tkeo_sample_cov = Eigen::MatrixXd::Zero(D, D);
//    Eigen::MatrixXd tkeo_sample_cov_inv(D, D);
//
//    Eigen::VectorXd rest_sample_mean(D);
//    Eigen::MatrixXd rest_sample_cov(D, D);
//    estimate_gaussian_params(tkeo_rest_data_, rest_sample_mean, rest_sample_cov);
//
//    tkeo_sample_means.col(0) = rest_sample_mean;
//    double scalar = static_cast<double>(tkeo_active_data_.size()) - 1.0;
//    rest_sample_cov /= scalar;
//    tkeo_sample_cov = rest_sample_cov;
//
//    scalar = static_cast<double>(tkeo_active_data_.size());
//    for (size_t k = 0; k < tkeo_active_data_.size(); ++k) {
//
//        Eigen::VectorXd active_sample_mean(D);
//        Eigen::MatrixXd active_sample_cov(D, D);
//        estimate_gaussian_params(tkeo_active_data_[k], active_sample_mean, active_sample_cov);
//
//        tkeo_sample_means.col(k + 1) = active_sample_mean;
//        active_sample_cov /= scalar;
//        tkeo_sample_cov += active_sample_cov;
//    }
//
//
//    if (tkeo_sample_cov.fullPivLu().isInvertible())
//        tkeo_sample_cov_inv = tkeo_sample_cov.fullPivLu().inverse();
//    else {
//        print("ERROR: TKEO sample covariance matrix cannot be inverted.");
//        auto_stop_ = true;
//        return;
//    }   
//
//    // compute linear model
//    tkeo_W_t_.resize(num_classes_ + 1, D);
//    tkeo_w_0_.resize(num_classes_ + 1);
//    for (int k = 0; k < num_classes_ + 1; ++k) {
//        tkeo_W_t_.row(k) = tkeo_sample_means.col(k).transpose() * tkeo_sample_cov_inv;
//        tkeo_w_0_(k) = -tkeo_sample_means.col(k).transpose() * tkeo_sample_cov_inv * tkeo_sample_means.col(k);
//    }
//
//}


//void EmgRealTimeControl::estimate_gaussian_params(const std::vector<std::vector<double>>& sample_data, Eigen::VectorXd& sample_mean, Eigen::MatrixXd& sample_cov) {
//    size_t n_rows = sample_data.size();
//    size_t n_cols = sample_data[0].size();
//
//    sample_mean.resize(n_rows);
//    sample_cov.resize(n_rows, n_rows);
//
//    sample_mean.setConstant(0.0);
//    sample_cov.setConstant(0.0);
//
//    for (size_t i = 0; i < n_rows; ++i) {
//        if (sample_data[i].size() != n_cols) {
//            print("ERROR: Function ml_covariance_estimate was given argument sample_data with incorrect dimensions.");
//            return;
//        }
//    }
//
//    for (size_t i = 0; i < n_rows; ++i) {
//        sample_mean(i) = std::accumulate(sample_data[i].begin(), sample_data[i].end(), 0.0) / static_cast<double>(n_cols);
//    }
//
//    Eigen::MatrixXd sample(n_rows, 1);
//    for (size_t j = 0; j < n_cols; ++j) {
//        for (size_t i = 0; i < n_rows; ++i) {
//            sample(i, 0) = sample_data[i][j];
//        }
//        sample -= sample_mean;
//        sample_cov += sample * sample.transpose().eval();
//    }
//    sample_cov = sample_cov / static_cast<double>(n_cols);
//}

//-----------------------------------------------------------------------------
// EMG FEATURE EXTRACTION FUNCTIONS
//-----------------------------------------------------------------------------

std::vector<double> EmgRealTimeControl::feature_extract() {

    std::size_t emg_channel_count = meii_.get_emg_channel_count();
    std::vector<double> feature_vec;
    std::vector<double> nrms_vec(emg_channel_count, 0.0);
    std::vector<double> nmav_vec(emg_channel_count, 0.0);
    std::vector<double> nwl_vec(emg_channel_count, 0.0);
    std::vector<double> nzc_vec(emg_channel_count, 0.0);
    std::vector<double> nssc_vec(emg_channel_count, 0.0);
    std::vector<double> ar_vec(4, 0.0);
    std::vector<double> ar1_vec(emg_channel_count, 0.0);
    std::vector<double> ar2_vec(emg_channel_count, 0.0);
    std::vector<double> ar3_vec(emg_channel_count, 0.0);
    std::vector<double> ar4_vec(emg_channel_count, 0.0);

    // extract unnormalized features
    int it_offset = 0;
    feature_vec = meii_.get_all_features();
    std::copy(feature_vec.begin() + it_offset, feature_vec.begin() + it_offset + emg_channel_count, nrms_vec.begin());
    it_offset += emg_channel_count;
    std::copy(feature_vec.begin() + it_offset, feature_vec.begin() + it_offset + emg_channel_count, nmav_vec.begin());
    it_offset += emg_channel_count;
    std::copy(feature_vec.begin() + it_offset, feature_vec.begin() + it_offset + emg_channel_count, nwl_vec.begin());
    it_offset += emg_channel_count;
    std::copy(feature_vec.begin() + it_offset, feature_vec.begin() + it_offset + emg_channel_count, nzc_vec.begin());
    it_offset += emg_channel_count;
    std::copy(feature_vec.begin() + it_offset, feature_vec.begin() + it_offset + emg_channel_count, nssc_vec.begin());
    it_offset += emg_channel_count;
    std::copy(feature_vec.begin() + it_offset, feature_vec.begin() + it_offset + emg_channel_count, ar1_vec.begin());
    it_offset += emg_channel_count;
    std::copy(feature_vec.begin() + it_offset, feature_vec.begin() + it_offset + emg_channel_count, ar2_vec.begin());
    it_offset += emg_channel_count;
    std::copy(feature_vec.begin() + it_offset, feature_vec.begin() + it_offset + emg_channel_count, ar3_vec.begin());
    it_offset += emg_channel_count;
    std::copy(feature_vec.begin() + it_offset, feature_vec.begin() + it_offset + emg_channel_count, ar4_vec.begin());

    //for (int i = 0; i < num_emg_channels_; ++i) {
    //    nrms_vec[i] = rms_feature_extract(emg_data_buffer.get_channel(i));
    //    nmav_vec[i] = mav_feature_extract(emg_data_buffer.get_channel(i));
    //    nwl_vec[i] = wl_feature_extract(emg_data_buffer.get_channel(i));
    //    nzc_vec[i] = zc_feature_extract(emg_data_buffer.get_channel(i));
    //    nssc_vec[i] = ssc_feature_extract(emg_data_buffer.get_channel(i));
    //    ar_4_feature_extract(ar_vec, emg_data_buffer.get_channel(i));
    //    ar1_vec[i] = ar_vec[0];
    //    ar2_vec[i] = ar_vec[1];
    //    ar3_vec[i] = ar_vec[2];
    //    ar4_vec[i] = ar_vec[3];
    //}


    // normalize time-domain features
    double rms_mean = mean(nrms_vec);
    double mav_mean = mean(nmav_vec);
    double wl_mean = mean(nwl_vec);
    double zc_mean = mean(nzc_vec);
    double ssc_mean = mean(nssc_vec);
    for (int i = 0; i < emg_channel_count; ++i) {
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

    // check for validity of feature computations
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

    // copy features into one vector
    feature_vec.clear();
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




//-----------------------------------------------------------------------------
// EXPERIMENT SETUP/CONDITIONS UTILITY FUNCTIONS
//-----------------------------------------------------------------------------

//void EmgRealTimeControl::set_experiment_conditions(int scene_num) {
//    if (scene_num > 0) {
//        dof_ = (scene_num - 1) / 4;
//        condition_ = (scene_num - 1) % 4;
//        if (is_single_dof()) {
//            num_classes_ = 2;
//        }
//        else {
//            num_classes_ = 4;
//        }
//        game_.set_variables(hand_num_, dof_, condition_);
//    }    
//}

std::vector<int> EmgRealTimeControl::gen_rand_class_labels(int num_labels) const {
    std::random_device rd_seed; // random seed
    std::mt19937 rd_gen(rd_seed()); // random unsigned integer generator
    std::uniform_int_distribution<> class_labels_dist(1, num_classes_);
    std::vector<int> class_label_list(num_labels, 0);
    for (int i = 0; i < num_labels; ++i) {
        class_label_list[i] = class_labels_dist(rd_gen);
    }
    return class_label_list;
}

std::vector<int> EmgRealTimeControl::rand_shuffle_class_labels(int num_labels_per_class) const {
    std::vector<int> class_label_list(num_labels_per_class * num_classes_, 0);
    for (int i = 0; i < num_labels_per_class * num_classes_; ++i) {
        class_label_list[i] = 1 + (i / num_labels_per_class);
    }
    std::srand(std::time(0));
    std::random_shuffle(class_label_list.begin(), class_label_list.end());
    return class_label_list;
}

bool EmgRealTimeControl::is_single_dof() const {
    return dof_ < 4;
}

bool EmgRealTimeControl::is_cal() const {
    return condition_ == 0;
}

bool EmgRealTimeControl::is_training() const {
    return condition_ == 1;
}

bool EmgRealTimeControl::is_testing() const {
    return condition_ == 2 || condition_ == 3;
}

bool EmgRealTimeControl::is_blind() const {
    return condition_ == 2;
}

bool EmgRealTimeControl::is_full() const {
    return condition_ == 3;
}

//-----------------------------------------------------------------------------
// EXPERIMENT CONTROL UTILITY FUNCTIONS
//-----------------------------------------------------------------------------

std::vector<double> EmgRealTimeControl::get_target_position(int class_label) {
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

bool EmgRealTimeControl::check_wait_time_reached(Time wait_time, Time init_time, Time current_time) const {
    return (current_time - init_time) > wait_time;
}

double EmgRealTimeControl::measure_task_force(std::vector<double> commanded_torques, int target_num, int dof, int condition) const {

    double task_force;
    std::vector<std::vector<int>> target_dir;

    switch (task_force_measurement_mode_) {
    case 0:
        if (hand_num_ == 1) {
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

//-----------------------------------------------------------------------------
// USER INPUT/OUTPUT UTILITY FUNCTIONS
//-----------------------------------------------------------------------------

void EmgRealTimeControl::check_exit_program() {
    exit_program_ = (Keyboard::is_key_pressed(Key::Escape, false) | exit_program_);
}

int EmgRealTimeControl::is_any_num_key_pressed() const {
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

//-----------------------------------------------------------------------------
// FILE READ/WRITE UTILITY FUNCTIONS
//-----------------------------------------------------------------------------

template <typename T>
bool EmgRealTimeControl::read_csv(std::string filename, std::string directory, std::vector<std::vector<T>>& output) {
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
        LOG(Warning) << "File not found for read_csv().";
        return false;
    }
}
template bool EmgRealTimeControl::read_csv<int>(std::string filename, std::string directory, std::vector<std::vector<int>>& output);
template bool EmgRealTimeControl::read_csv<double>(std::string filename, std::string directory, std::vector<std::vector<double>>& output);

template <typename T>
bool EmgRealTimeControl::write_csv(std::string filename, std::string directory, const std::vector<std::vector<T>>& input) const {
    std::string full_filename = directory + "\\" + filename + ".csv";
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
        LOG(Warning) << "File not found for write_csv().";
        return false;
    }
}
template bool EmgRealTimeControl::write_csv<int>(std::string filename, std::string directory, const std::vector<std::vector<int>>& input) const;
template bool EmgRealTimeControl::write_csv<double>(std::string filename, std::string directory, const std::vector<std::vector<double>>& input) const;




//void EmgRTControl::store_buffer(const MahiExoIIEmg::EmgDataBuffer& data_buffer, std::vector<std::vector<double>>& data_matrix) {
//    if (data_matrix.empty()) {
//        for (int i = 0; i < data_buffer.num_channels_; ++i) {
//            data_matrix.push_back(std::vector<double>());
//        }
//    }
//    
//    if (data_matrix.size() != data_buffer.num_channels_) {
//        print("ERROR: Number of channels in data buffer does not match number of rows in data matrix.");
//        return;
//    }
//    std::vector<double> data_buffer_channel;
//    for (int i = 0; i < data_buffer.num_channels_; ++i) {
//        data_buffer_channel = data_buffer.get_channel(i);
//        data_matrix[i].insert(data_matrix[i].end(), data_buffer_channel.begin(), data_buffer_channel.end());
//    }
//}



bool EmgRealTimeControl::write_emg_active_classifier() {
    bool write_status(false);

    // store in a csv-writable data type
    /*Eigen::MatrixXd calibration_data_eig(tkeo_W_t_.rows(), tkeo_W_t_.cols() + tkeo_w_0_.cols());
    calibration_data_eig << tkeo_W_t_, tkeo_w_0_;
    calibration_data_ = copy_eigmat_to_stdvecvec(calibration_data_eig);*/
    if (active_detectors_.size() != num_classes_) {
        LOG(Error) << "Mismatch between active detector and number of active classes expected.";
        auto_stop_ = true;
        return write_status;
    }
    std::vector<std::vector<double>> calibration_data;
    for (std::size_t i = 0; i < active_detectors_.size(); ++i) {
        calibration_data.push_back(active_detectors_[i].get_bin_model());
    }

    // write to csv   
    LOG(Info) << "Writing calibration data to " + subject_dof_directory_ + "\\" + emg_active_classifier_filename_;
    write_status = write_csv<double>(emg_active_classifier_filename_, subject_dof_directory_, calibration_data);
    if (!write_status)
        LOG(Warning) << "Failure to write calibration data.";
    return write_status;

    // write to csv with time stamp and additional identifying information
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}



bool EmgRealTimeControl::load_emg_active_classifier() {
    bool read_status(false);

    // read from csv
    std::vector<std::vector<double>> calibration_data;
    LOG(Info) << "Loading calibration data from " + subject_dof_directory_ + "\\" + emg_active_classifier_filename_;
    read_status = read_csv<double>(emg_active_classifier_filename_, subject_dof_directory_, calibration_data);
    if (!read_status) {
        LOG(Warning) << "Failure to read calibration data.";
        return read_status;
    }

    // parse csv data
    if (calibration_data.size() != num_classes_ || calibration_data.size() != active_detectors_.size()) {
        LOG(Error) << "Mismatch between active detector and number of active classes expected.";
        auto_stop_ = true;
        return read_status;
    }
    for (std::size_t i = 0; i < calibration_data.size(); ++i) {
        active_detectors_[i].set_bin_classifier(calibration_data[i]);
    }
    return read_status;
    //Eigen::MatrixXd calibration_data_eig = copy_stdvecvec_to_eigmat(calibration_data_);
    //tkeo_W_t_ = calibration_data_eig.block(0, 0, num_classes_ + 1, meii_.get_emg_channel_count());
    //tkeo_w_0_ = calibration_data_eig.block(0, meii_.get_emg_channel_count(), num_classes_ + 1, 1);
}

bool EmgRealTimeControl::load_emg_dir_training_data() {
    bool read_status(false);
    
    read_status = read_csv<double>(training_data_filename_, subject_dof_directory_, prev_emg_training_data_);
    if (read_status)
        LOG(Info) << "Loading previous training data from " + subject_dof_directory_ + "\\" + training_data_filename_;
    return read_status;
}

bool EmgRealTimeControl::load_emg_dir_classifier() {
    bool read_status(false);

    std::vector<std::vector<double>> emg_dir_classifier;
    LOG(Info) << "Loading classifier from " + subject_dof_directory_ + "\\" + emg_dir_classifier_filename_;
    read_status = read_csv<double>(emg_dir_classifier_filename_, subject_dof_directory_, emg_dir_classifier);
    if (!read_status) {
        LOG(Warning) << "Failure to read classifier data.";
        return read_status;
    }

    // convert classifier to Eigen type matrix
    emg_dir_W_tt_ = copy_stdvecvec_to_eigmat(emg_dir_classifier);

}


void EmgRealTimeControl::save_log_data() {

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
    robot_log_.save_data("robot_data_log_" + log_file_name, directory, true);
    robot_log_.wait_for_save();
    robot_log_.clear_data();

    // save emg data log
    emg_log_.save_data("emg_data_log_" + log_file_name, directory, true);
    emg_log_.wait_for_save();
    emg_log_.clear_data();
    
    // save condition-specific data that was logged at each trial
    trial_log_.save_data(str_conditions_[condition_] + "_data_log_" + log_file_name, directory, true);
    trial_log_.wait_for_save();
    trial_log_.clear_data();

    //// save debugging data log if used
    //if (debug_log_.get_col_count() > 0 && debug_log_.get_row_count() > 0) {
    //    debug_log_.save_and_clear_data("debug_data_log_" + log_file_name, directory, true);
    //    debug_log_ = DataLog("debug_log", false);
    //}


}

void EmgRealTimeControl::init_robot_log() {
    robot_log_.set_header({ "Time [s]", "MEII EFE Position [rad]", "MEII EFE Velocity [rad/s]", "MEII EFE Commanded Torque [Nm]",
        "MEII FPS Position [rad]", "MEII FPS Velocity [rad/s]", "MEII FPS Commanded Torque [Nm]",
        "MEII RPS L1 Position [m]", "MEII RPS L1 Velocity [m/s]", "MEII RPS L1 Commanded Force [N]",
        "MEII RPS L2 Position [m]", "MEII RPS L2 Velocity [m/s]", "MEII RPS L2 Commanded Force [N]",
        "MEII RPS L3 Position [m]", "MEII RPS L3 Velocity [m/s]", "MEII RPS L3 Commanded Force [N]", "State" });
}

void EmgRealTimeControl::init_emg_log() {
    std::vector<std::string> header;
    header.push_back("Time [s]");
    for (int i = 0; i < meii_.get_emg_channel_count(); ++i) {
        header.push_back("Ch. " + std::to_string(i));
    }
    for (int i = 0; i < meii_.get_emg_channel_count(); ++i) {
        header.push_back("Filt Ch. " + std::to_string(i));
    }
    header.push_back("State");
    emg_log_.set_header(header);
}

void EmgRealTimeControl::init_trial_log() {
    std::vector<std::string> header;
    header.push_back("Time [s]");
    if (is_cal()) {
        for (int i = 0; i < meii_.get_emg_channel_count(); ++i) {
            header.push_back("TK " + std::to_string(i));
        }
        header.push_back("Target");
    }
    else if (is_training()) {        
        for (int i = 0; i < meii_.get_emg_channel_count() * num_features_; ++i) {
            header.push_back("Feat. " + std::to_string(i));
        }
        header.push_back("Class Label");
    }
    else if (is_blind()) {
        for (int i = 0; i < meii_.get_emg_channel_count() * num_features_; ++i) {
            header.push_back("Feat. " + std::to_string(i));
        }
        header.push_back("Class Label");
        header.push_back("Pred. Class Label");
        for (int k = 0; k < num_classes_; ++k) {
            header.push_back("Class " + std::to_string(k + 1) + "Post.");
        }
    }
    else if (is_full()) {
        for (int i = 0; i < meii_.get_emg_channel_count() * num_features_; ++i) {
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

//void EmgRTControl::init_debug_log() {
//    int N = 16;
//    debug_log_.add_col("Time [s]");
//        for (int i = 0; i < N; ++i) {
//            debug_log_.add_col("Sig. " + std::to_string(i));
//        }
//    debug_log_.add_col("Posterior");
//}

void EmgRealTimeControl::log_robot_row() {
    robot_log_.buffer({ timer_.get_elapsed_time().as_seconds(), meii_[0].get_position(), meii_[0].get_velocity(), meii_[0].get_torque(),
        meii_[1].get_position(), meii_[1].get_velocity(), meii_[1].get_torque(),
        meii_[2].get_position(), meii_[2].get_velocity(), meii_[2].get_torque(),
        meii_[3].get_position(), meii_[3].get_velocity(), meii_[3].get_torque(),
        meii_[4].get_position(), meii_[4].get_velocity(), meii_[4].get_torque(), (double) get_current_state() });
}


void EmgRealTimeControl::log_emg_row() {

    std::vector<double> row;
    row.push_back(timer_.get_elapsed_time().as_seconds());
    for (int i = 0; i < mes_.size(); ++i) {
        //row.push_back(emg_voltages_[i]);
        row.push_back(mes_.get_raw()[i]);
    }
    for (int i = 0; i < mes_.size(); ++i) {
        //row.push_back(filtered_emg_voltages_[i]);
        row.push_back(mes_.get_demean()[i]);
    }
    row.push_back((double)get_current_state());
    emg_log_.buffer(row);
}

void EmgRealTimeControl::log_trial_row() {
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
        row.clear();
        row.push_back(timer_.get_elapsed_time().as_seconds());
        for (int i = 0; i < mes_.size() * num_features_; ++i) {
            row.push_back(emg_feature_vec_[i]);
        }
        row.push_back(class_label_sequence_[current_class_label_idx_]);  
        row.push_back(subject_number_);
        row.push_back(hand_num_);
        row.push_back(dof_);
        row.push_back(condition_);
        trial_log_.buffer(row);
    }
    else if (is_blind()) {
        row.clear();
        row.push_back(timer_.get_elapsed_time().as_seconds());
        for (int i = 0; i < mes_.size() * num_features_; ++i) {
            row.push_back(emg_feature_vec_[i]);
        }
        row.push_back(class_label_sequence_[current_class_label_idx_]);
        row.push_back(pred_class_label_sequence_[current_class_label_idx_]);
        for (int k = 0; k < num_classes_; ++k) {
            row.push_back(class_posteriors_[k]);
        }
        row.push_back(subject_number_);
        row.push_back(hand_num_);
        row.push_back(dof_);
        row.push_back(condition_);
        trial_log_.buffer(row);
    }
    else if (is_full()) {
        row.clear();
        row.push_back(timer_.get_elapsed_time().as_seconds());
        for (int i = 0; i < mes_.size() * num_features_; ++i) {
            row.push_back(emg_feature_vec_[i]);
        }
        row.push_back(class_label_sequence_[current_class_label_idx_]);
        row.push_back(pred_class_label_sequence_[current_class_label_idx_]);
        for (int k = 0; k < num_classes_; ++k) {
            row.push_back(class_posteriors_[k]);
        }
        row.push_back(subject_number_);
        row.push_back(hand_num_);
        row.push_back(dof_);
        row.push_back(condition_);
        trial_log_.buffer(row);
        
    }
}

//void EmgRTControl::log_debug_row( std::vector<double> debug_row) {
//    std::vector<double> row;
//    row.push_back(clock_.get_current_time());
//    for (int i = 0; i < debug_row.size(); ++i) {
//        row.push_back(debug_row[i]);
//    }
//    debug_log_.add_row(row);
//}

//-----------------------------------------------------------------------------
// DATA ANALYSIS UTILITY FUNCTIONS
//-----------------------------------------------------------------------------


Eigen::MatrixXd EmgRealTimeControl::gen_confusion_mat( const std::vector<int>& actual_labels, const std::vector<int>& predicted_labels) const {
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

