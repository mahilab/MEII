#include "EmgRTControl.h"
#include "Input.h"
#include "Motor.h"
#include <iostream>
#include <fstream>
#include <string>
#include <random>
#include <numeric>
#include <boost/filesystem.hpp>


using namespace mel;

EmgRTControl::EmgRTControl(util::Clock& clock, core::Daq* daq, exo::MahiExoIIEmg& meii) :
    StateMachine(12),
    clock_(clock),
    daq_(daq),
    meii_(meii)
{
    // create subject folder for data logging
    subject_directory_ = "C:\\Users\\Ted\\GitHub\\MEII\\EmgRTControl\\EMG_S";
    if (subject_number_ < 10) {
        subject_directory_ += "0";
    }
    subject_directory_ += std::to_string(subject_number_);

}   


//-----------------------------------------------------------------------------
// "WAIT FOR GUI" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRTControl::sf_wait_for_gui(const util::NoEventData* data) {
    util::print("Waiting for Gui Input");

    // start the clock
    clock_.start();

    // initialize local state variables
    bool scene_selected = false;

    // launch game
    game.launch();

    // wait for gui input
    while (!scene_selected && !stop_) {

        // read from Unity
        scene_num_share_.read(SCENE_NUM_);

        // check if scene selected in Unity
        if (SCENE_NUM_ > 0) {
            scene_selected = true;
        }

        // check for stop input
        stop_ = check_stop();

        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    // transition to next state from "WAIT FOR GUI"
    if (stop_) {
        // stop if user provided input
        event(ST_STOP);
    }
    else if (scene_selected) {
        event(ST_INIT);
    }
    else {
        util::print("ERROR: State transition undefined. Going to ST_STOP.");
        event(ST_STOP);
    }
}


//-----------------------------------------------------------------------------
// "INITIALIZATION" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRTControl::sf_init(const util::NoEventData* data) {

    // set experiment condition variables dof_ and condition_ based on scene number
    set_experiment_conditions(SCENE_NUM_);
    util::print("Initializing for " + str_conditions_long_[condition_] + " of " + str_dofs_long_[dof_]);

    // reset global experiment variables
    end_of_label_sequence_ = true;
    class_label_sequence_.clear();
    current_class_label_idx_ = -1;
    viz_target_num_ = 0;
    pred_class_label_ = 0;
    lda_classifier_.clear();
    emg_training_data_.clear();
    prev_emg_training_data_.clear();
    calibration_data_.clear();
    tkeo_rest_data_.clear();
    tkeo_active_data_.clear();

    // create subject and DoF specific folder for data logging
    subject_dof_directory_ = subject_directory_ + "\\" + str_dofs_[dof_];

    // generate file names
    std::string str_subject_number = "S";
    if (subject_number_ < 10) {
        str_subject_number += "0";
    }
    str_subject_number += std::to_string(subject_number_);
    calibration_data_filename_ = str_subject_number + "_" + str_dofs_[dof_] + "_calibration_data";
    training_data_filename_ = str_subject_number + "_" + str_dofs_[dof_] + "_training_data";
    lda_classifier_filename_ = str_subject_number + "_" + str_dofs_[dof_] + "_lda_classifier";
    directory_share_.write_message(subject_dof_directory_);
    file_name_share_.write_message(training_data_filename_);

    // reset robot
    meii_.disable();

    // Add columns to training logger
    training_log_.add_col("Time [s]").add_col("DoF").add_col("Condition");
    for (int i = 0; i < num_emg_channels_ * num_features_; ++i) {
        training_log_.add_col("Feat. " + std::to_string(i));
    }
    training_log_.add_col("Target");
    if (is_testing()) {
        training_log_.add_col("LDA Pred");
        training_log_.add_col("LDA Dist1");
            for (int i = 0; i < 3; ++i) {
                training_log_.add_col("LDA Dist" + std::to_string(i + 2));
            }
    }   

    // Add columns to robot logger
    robot_log_.add_col("Time [s]").add_col("State")
              .add_col("MEII EFE Position [deg]").add_col("MEII EFE Velocity [deg/s]").add_col("MEII EFE Commanded Torque [Nm]")
              .add_col("MEII FPS Position [deg]").add_col("MEII FPS Velocity [deg/s]").add_col("MEII FPS Commanded Torque [Nm]")
              .add_col("MEII RPS1 Position [m]").add_col("MEII RPS1 Velocity [m/s]").add_col("MEII RPS1 Commanded Force [N]")
              .add_col("MEII RPS2 Position [m]").add_col("MEII RPS2 Velocity [m/s]").add_col("MEII RPS2 Commanded Force [N]")
              .add_col("MEII RPS3 Position [m]").add_col("MEII RPS3 Velocity [m/s]").add_col("MEII RPS3 Commanded Force [N]");


    // enable MEII EMG DAQ
    daq_->enable();
    if (!daq_->is_enabled()) {
        event(ST_STOP);
        return;
    }

    // check DAQ behavior for safety
    daq_->read_all();
    meii_.update_kinematics();
    if (meii_.check_all_joint_limits()) {
        event(ST_STOP);
        return;
    }
    if (!dev::Q8Usb::check_digital_loopback(0, 7)) {
        event(ST_STOP);
        return;
    }

    // enable MEII
    meii_.enable();
    if (!meii_.is_enabled()) {
        event(ST_STOP);
        return;
    }   

    // confirm start of experiment
    util::print("\nRunning EMG Real-Time Control ... ");

    // write to Unity
    hand_select_.write_message(hand_def_);

    // start the watchdog
    daq_->start_watchdog(0.1);

    // transition to next state from "INITIALIZATION"
    if (stop_) {
        event(ST_STOP);
    }
    else {
        event(ST_BACKDRIVE);
    }
}


//-----------------------------------------------------------------------------
// "BACKDRIVE" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRTControl::sf_backdrive(const util::NoEventData* data) {
    util::print("Robot Backdrivable"); 

    // initialize local state variables
    bool init_backdrive_time_reached = false;
    double st_enter_time = clock_.time();
    const double_vec command_torques(meii_.N_aj_, 0.0);

    // enter the control loop
    while (!init_backdrive_time_reached && !stop_) {

        // read and reload DAQs
        daq_->reload_watchdog();
        daq_->read_all();

        // update robot kinematics
        meii_.update_kinematics();

        // write kinematics to MelScope
        pos_share_.write(meii_.get_anatomical_joint_positions());
        vel_share_.write(meii_.get_anatomical_joint_velocities());

        // check joint limits
        if (meii_.check_all_joint_limits()) {
            stop_ = true;
            break;
        }

        // set zero torques
        meii_.set_joint_torques(command_torques);

        // write to daq
        daq_->write_all();

        // check for init transparent time reached
        init_backdrive_time_reached = check_wait_time_reached(init_backdrive_time_, st_enter_time, clock_.time());

        // check for stop input
        stop_ = check_stop();

        // log data
        log_robot_row();

        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    // transition to next state from "BACKDRIVE"
    if (stop_) {
        // stop if user provided input
        event(ST_STOP);
    }
    else if (init_backdrive_time_reached) {
        event(ST_INIT_RPS);
    }
    else {
        util::print("ERROR: State transition undefined. Going to ST_STOP.");
        event(ST_STOP);
    }
}


//-----------------------------------------------------------------------------
// "INITIALIZE RPS MECHANISM" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRTControl::sf_init_rps(const util::NoEventData* data) {
    util::print("Initialize RPS Mechanism");

    // initialize local state variables
    bool rps_init = false;

    // initialize rps initialization position controller
    meii_.set_rps_control_mode(0);
    meii_.update_kinematics();
    meii_.rps_init_par_ref_.start(meii_.get_wrist_parallel_positions(), clock_.time());

    // enter the control loop
    while (!rps_init && !stop_) {

        // read and reload DAQs
        daq_->reload_watchdog();
        daq_->read_all();

        // update robot kinematics
        meii_.update_kinematics();

        // check joint limits
        if (meii_.check_all_joint_limits()) {
            stop_ = true;
            break;
        }

        // write kinematics to MelScope
        pos_share_.write(meii_.get_anatomical_joint_positions());
        vel_share_.write(meii_.get_anatomical_joint_velocities());

        // set zero torque for elbow and forearm joints (joints 0 and 1)
        meii_.joints_[0]->set_torque(0.0);
        meii_.joints_[1]->set_torque(0.0);

        // run rps position control
        meii_.set_rps_pos_ctrl_torques(meii_.rps_init_par_ref_, clock_.time());

        // write to daq
        daq_->write_all();

        // check for rps mechanism in intialization position
        rps_init = meii_.check_rps_init();

        // check for stop input
        stop_ = check_stop();

        // log data
        log_robot_row();

        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    // stop the rps intialization position controller
    meii_.rps_init_par_ref_.stop();

    // transition to next state from "INITIALIZE RPS MECHANISM"
    if (stop_) {
        // stop if user provided input
        event(ST_STOP);
    }
    else if (rps_init) {
        event(ST_TO_CENTER);
    }
    else {
        util::print("ERROR: State transition undefined. Going to ST_STOP.");
        event(ST_STOP);
    }
}



//-----------------------------------------------------------------------------
// "GO TO CENTER" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRTControl::sf_to_center(const util::NoEventData* data) {
    util::print("Go to Center");
    
    // initialize local state variables
    bool target_reached = false;
    double_vec command_torques(meii_.N_aj_, 0.0);

    // initialize rps position controller mode and reference
    meii_.set_rps_control_mode(1);
    if (meii_.anat_ref_.is_started()) {
        meii_.anat_ref_.set_ref(center_pos_, clock_.time());
    }
    else {
        meii_.anat_ref_.start(center_pos_, meii_.get_anatomical_joint_positions(), clock_.time());
    }

    // write to Unity
    viz_target_num_ = 0;
    viz_target_num_share_.write(viz_target_num_);

    // enter the control loop
    while (!target_reached && !stop_) {

        // read and reload DAQs
        daq_->reload_watchdog();
        daq_->read_all();

        // update robot kinematics
        meii_.update_kinematics();

        // write kinematics to MelScope
        pos_share_.write(meii_.get_anatomical_joint_positions());
        vel_share_.write(meii_.get_anatomical_joint_velocities());

        // check joint limits
        if (meii_.check_all_joint_limits()) {
            stop_ = true;
            break;
        }

        // run position control
        command_torques = meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, clock_.time()); 

        // write motor commands to MelScope
        torque_share_.write(command_torques);

        // write to daq
        daq_->write_all();

        // check for target reached
        target_reached = meii_.check_goal_anat_pos(center_pos_, { 1, 1, 1, 1, 0 });

        // check for stop input
        stop_ = check_stop();

        // log data
        log_robot_row();

        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    // transition to next state from "GOT TO CENTER"
    if (stop_) {
        // stop if user provided input
        event(ST_STOP);
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
        util::print("ERROR: State transition undefined. Going to ST_STOP.");
        event(ST_STOP);
    }
}


//-----------------------------------------------------------------------------
// "HOLD AT CENTER" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRTControl::sf_hold_center(const util::NoEventData* data) {
    util::print("Hold at Center");

    // initialize global variables
    
    if (is_cal()) {
        hold_center_time_ = 3.0;
        emg_calibration_data_buffer_.flush();
        meii_.butter_hp_.reset();   
    }
    else {
        hold_center_time_ = 1.0;
    }

    // initialize local state variables
    bool hold_center_time_reached = false;
    double st_enter_time = clock_.time();
    double_vec command_torques(meii_.N_aj_, 0.0);
    double_vec filtered_emg_voltages = double_vec(meii_.N_emg_, 0.0);
    double_vec tkeo_vec(meii_.N_emg_, 0.0);
    double_vec filtered_tkeo_vec(meii_.N_emg_, 0.0);

    // write to Unity
    viz_target_num_ = 0;
    viz_target_num_share_.write(viz_target_num_);

    // enter the control loop
    while (!hold_center_time_reached && !stop_) {

        // read and reload DAQs
        daq_->reload_watchdog();
        daq_->read_all();

        // update robot kinematics
        meii_.update_kinematics();

        // write kinematics to MelScope
        pos_share_.write(meii_.get_anatomical_joint_positions());
        vel_share_.write(meii_.get_anatomical_joint_velocities());

        // check joint limits
        if (meii_.check_all_joint_limits()) {
            stop_ = true;
            break;
        }

        // run position control
        command_torques = meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, clock_.time());

        // write motor commands to MelScope
        torque_share_.write(command_torques);
      
        if (is_cal()) {
            // get measured emg voltages
            meii_.butter_hp_.filter(meii_.get_emg_voltages(), filtered_emg_voltages);
            emg_share_.write(filtered_emg_voltages);

            // compute the teager-kaiser metric online with rectification and filtering
            meii_.tko_.tkeo(filtered_emg_voltages, tkeo_vec);
            std::for_each(tkeo_vec.begin(), tkeo_vec.end(), [](double x) { return std::abs(x); });
            meii_.tkeo_butter_lp_.filter(tkeo_vec, filtered_tkeo_vec);
            emg_calibration_data_buffer_.push_back(filtered_tkeo_vec);
        }


        // check for hold time reached
        hold_center_time_reached = check_wait_time_reached(hold_center_time_, st_enter_time, clock_.time());

        // write to daq
        daq_->write_all();

        // check for stop input
        stop_ = check_stop();

        // log data
        log_robot_row();

        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    // transition to next state from "HOLD AT CENTER"
    if (stop_) {
        // stop if user provided input
        event(ST_STOP);
    }
    else if (hold_center_time_reached) {
        if (is_cal()) {
            store_buffer(emg_calibration_data_buffer_, tkeo_rest_data_);
            event(ST_PRESENT_TARGET);   
        }
        event(ST_PRESENT_TARGET);    
    }
    else {
        util::print("ERROR: State transition undefined. Going to ST_STOP.");
        event(ST_STOP);
    }
}


//-----------------------------------------------------------------------------
// "HOLD FOR INPUT" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRTControl::sf_hold_for_input(const util::NoEventData* data) {
    util::print("Hold for Input");

    // reset global variables
    if (class_label_sequence_.empty()) {
        end_of_label_sequence_ = false;
    }

    // initialize local state variables
    bool finished = false;
    bool present_more_targets = false;
    bool python_return = false;
    bool more_training_data = false;
    int num_observations_per_class;

    util::Input::Key key;
    double_vec command_torques(meii_.N_aj_, 0.0);
    int lda_training_complete = 0;

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
            load_calibration_data();

            // ask user if they want to use the existing training data
            util::print("Press Enter to collect new training data, or press L to Load previous training data...");
        }
        else if (is_testing()) {

            // read in emg calibration data from file
            load_calibration_data();

            // read in classifier from file
            load_classifier();

            // ask user how much testing data they want to collect
            util::print("How many observations per class would you like to test? (0-9)");
        }
        else {
            util::print("ERROR: Condition number was set improperly. Going to ST_STOP.");
            stop_ = true;
        }
    }

    // initialization actions upon end of label sequence
    else { 
        if (is_cal()) {

            // compute statistics from calibration data and store in writable data type
            tkeo_model_estimation();

            // write calibration data to file
            write_calibration_data();
            finished = true;
        }
        else if (is_training()) {

            // write training data to file
            if (!write_csv(training_data_filename_, subject_dof_directory_, emg_training_data_)) {
                stop_ = true;
            }

            // send file location to python over melshare
            directory_share_.write_message(subject_dof_directory_);
            file_name_share_.write_message(training_data_filename_);

            // open LDA script in Python
            std::string system_command;
            if (is_single_dof()) {
                system_command = "start " + program_directory_ + "\\" + "EMG_FS_LDA.py &";
            }
            else {
                system_command = "start " + program_directory_ + "\\" + "EMG_FS_LDA.py &";
            }
            system(system_command.c_str());

            // wait for python to return results
            util::print("Waiting for Python to return training results...");
        }
        else if (is_testing()) {

            // going directly to ST_FINISH
            finished = true;
        }
        else {
            util::print("ERROR: Condition number was set improperly. Going to ST_STOP.");
            stop_ = true;
        } 
    }
    
    // write to Unity
    viz_target_num_ = 0;
    viz_target_num_share_.write(viz_target_num_);

    // enter the control loop
    while ( !present_more_targets && !finished && !stop_) {

        // read and reload DAQs
        daq_->reload_watchdog();
        daq_->read_all();

        // update robot kinematics
        meii_.update_kinematics();

        // write kinematics to MelScope
        pos_share_.write(meii_.get_anatomical_joint_positions());
        vel_share_.write(meii_.get_anatomical_joint_velocities());

        // check joint limits
        if (meii_.check_all_joint_limits()) {
            stop_ = true;
            break;
        }

        // run position control
        command_torques = meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, clock_.time());

        // write motor commands to MelScope
        torque_share_.write(command_torques);

        // write to daq
        daq_->write_all();

        // check for external input upon first entry to ST_HOLD_FOR_INPUT
        if (!end_of_label_sequence_) {
            if (is_cal()) {
                util::print("ERROR: Should not be reached. Going to ST_STOP.");
                stop_ = true;
            }
            else if (is_training()) {
                
                if (!more_training_data) {
                    key = util::Input::are_any_keys_pressed({ util::Input::Enter, util::Input::L });
                    switch (key) {
                    case util::Input::Key::Enter:
                        more_training_data = true;
                        util::print("How many observations per class would you like to collect? (0-9)");
                        break;
                    case util::Input::Key::L:
                        util::print("Loading previous training data from " + subject_dof_directory_ + "\\" + training_data_filename_);
                        if (!read_csv(training_data_filename_, subject_dof_directory_, prev_emg_training_data_)) {
                            stop_ = true;
                        }
                        emg_training_data_ = prev_emg_training_data_;
                        more_training_data = true;
                        util::print("How many observations per class would you like to collect? (0-9)");
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

                num_observations_per_class = is_any_num_key_pressed();
                if (num_observations_per_class > 0) {
                    present_more_targets = true;
                }
                else if (num_observations_per_class == 0) {
                    finished = true;
                }
            }
            else {
                util::print("ERROR: Condition number was set improperly. Going to ST_STOP.");
                stop_ = true;
            }
        }

        // check for external input upon end of label sequence
        else {
            if (is_cal()) {
                util::print("ERROR: Should not be reached. Going to ST_STOP.");
                stop_ = true;
            }
            else if (is_training()) {

                if (lda_training_complete == 0) {
                    lda_training_flag_.read(lda_training_complete);
                }
                else {

                    if (!python_return) {
                        python_return = true;

                        // read in the results from the Cross-validation test in Python
                        util::print("The cross validation results are:");
                        std::vector<double> cross_eval_test(5);
                        cv_results_.read(cross_eval_test);
                        util::print(cross_eval_test);

                        // ask user if they want to collect more training data
                        util::print("Press Enter to complete training mode. Press C to continue collecting training data.");

                    }
                    else {

                        if (!more_training_data) {
                            key = util::Input::are_any_keys_pressed({ util::Input::Enter,util::Input::C });
                            switch (key) {
                            case util::Input::Key::Enter:
                                finished = true;
                                break;
                            case util::Input::Key::C:
                                more_training_data = true;
                                util::print("How many more observations per class would you like to collect? (0-9)");
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
                }
            }
            else if (is_testing()) {
                util::print("ERROR: Should not be reached. Going to ST_STOP.");
                stop_ = true;
            }
            else {
                util::print("ERROR: Condition number was set improperly. Going to ST_STOP.");
                stop_ = true;
            }       
        }

        // check for stop input
        stop_ = check_stop();

        // log data
        log_robot_row();

        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    // transition to next state from "HOLD AT CENTER"
    if (stop_) {
        // stop if user provided input
        event(ST_STOP);
    }
    else if (finished) {
        event(ST_FINISH);
    }
    else if (present_more_targets) {
        end_of_label_sequence_ = false;
        lda_training_complete = 0;
        lda_training_flag_.write(lda_training_complete);
        if (!is_cal()) {
            std::vector<int> new_class_labels = rand_shuffle_class_labels(num_observations_per_class);
            for (int i = 0; i < new_class_labels.size(); ++i) {
                class_label_sequence_.push_back(new_class_labels[i]);
            }
        }
        event(ST_PRESENT_TARGET);
    }
    else {
        util::print("ERROR: State transition undefined. Going to ST_STOP.");
        event(ST_STOP);
    }
}



//-----------------------------------------------------------------------------
// "PRESENT TARGET" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRTControl::sf_present_target(const util::NoEventData* data) {
    util::print("Present Target");

    // initialize global variables
    emg_calibration_data_buffer_.flush();
    emg_trigger_data_buffer_.flush();
    emg_classification_data_buffer_.flush();
    meii_.butter_hp_.reset();
    meii_.tko_.reset();
    meii_.tkeo_butter_lp_.reset();

    // initialize local state variables
    bool force_mag_reached = false;
    bool active_state_reached = false;
    bool mvc_started = false;
    double mvc_start_time = 0;
    bool mvc_completed = false;
    bool keep_mvc = false;
    bool redo_mvc = false;
    double_vec filtered_emg_voltages(meii_.N_emg_, 0.0);
    double_vec tkeo_vec(meii_.N_emg_, 0.0);
    double_vec filtered_tkeo_vec(meii_.N_emg_, 0.0);
    double_vec command_torques(meii_.N_aj_, 0.0);
    double force_mag = 0.0;
    
    
    // initialize magnitude force checking algorithm
    force_mag_maintained_ = 0.0;
    force_mag_time_now_ = clock_.global_time();
    force_mag_time_last_ = clock_.global_time();

    // read from target sequence and write to Unity
    set_viz_target_num(class_label_sequence_[current_class_label_idx_]);
    viz_target_num_share_.write(viz_target_num_);

    if (is_cal()) {
        // ask user to indicate when MVC has started
        util::print("Press ENTER when MVC has started.");
    }

    // enter the control loop
    while (!end_of_label_sequence_ && !keep_mvc && !redo_mvc && !active_state_reached && !stop_) {

        // read and reload DAQs
        daq_->reload_watchdog();
        daq_->read_all();

        // update robot kinematics
        meii_.update_kinematics();

        // write kinematics to MelScope
        pos_share_.write(meii_.get_anatomical_joint_positions());
        vel_share_.write(meii_.get_anatomical_joint_velocities());

        // check joint limits
        if (meii_.check_all_joint_limits()) {
            stop_ = true;
            break;
        }

        // run position control
        command_torques = meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, clock_.time());

        // write motor commands to MelScope
        torque_share_.write(command_torques);

        // get measured emg voltages
        meii_.butter_hp_.filter(meii_.get_emg_voltages(), filtered_emg_voltages);
        emg_share_.write(filtered_emg_voltages);
        if (is_cal()) {
            if (mvc_started && !mvc_completed) {

                // compute the teager-kaiser metric online with rectification and filtering
                meii_.tko_.tkeo(filtered_emg_voltages, tkeo_vec);
                std::for_each(tkeo_vec.begin(), tkeo_vec.end(), [](double x) { return std::abs(x); });
                meii_.tkeo_butter_lp_.filter(tkeo_vec, filtered_tkeo_vec);
                emg_calibration_data_buffer_.push_back(filtered_tkeo_vec);
            }
        }
        else {

            // compute the teager-kaiser metric online with rectification and filtering
            meii_.tko_.tkeo(filtered_emg_voltages, tkeo_vec);
            std::for_each(tkeo_vec.begin(), tkeo_vec.end(), [](double x) { return std::abs(x); });
            meii_.tkeo_butter_lp_.filter(tkeo_vec, filtered_tkeo_vec);
            tkeo_sufficient_statistic(filtered_tkeo_vec);

            emg_classification_data_buffer_.push_back(filtered_emg_voltages);
        }
        
        // measure interaction force for specified dof(s)
        force_mag = measure_task_force(command_torques, class_label_sequence_[current_class_label_idx_], dof_, condition_);

        // check for exit conditions
        if (is_cal()) {

            // check for user input
            if (!mvc_started) {
                if (util::Input::is_key_pressed(util::Input::Enter)) {
                    mvc_started = true;
                    mvc_start_time = clock_.time();
                    util::print("Holding for contraction.");
                }
            }
            else {  
                if (!mvc_completed) {
                    mvc_completed = check_wait_time_reached(wait_mvc_time_, mvc_start_time, clock_.time());
                    if (mvc_completed) {
                        util::print("Do you want to keep the calibration data for this contraction (Y/N)?");
                    }
                }
                else {
                    

                    util::Input::Key key = util::Input::are_any_keys_pressed({ util::Input::Y, util::Input::N });
                    switch (key) {
                    case util::Input::Y:
                        keep_mvc = true;
                        break;
                    case util::Input::N:
                        redo_mvc = true;
                        break;
                    }
                }  
            }
        }
        else {

            // compute the teager-kaiser metric online with rectification and filtering
            meii_.tko_.tkeo(filtered_emg_voltages, tkeo_vec);
            std::for_each(tkeo_vec.begin(), tkeo_vec.end(), [](double x) { return std::abs(x); });
            meii_.tkeo_butter_lp_.filter(tkeo_vec, filtered_tkeo_vec);

            if (tkeo_detector(filtered_tkeo_vec)) {
                active_state_reached = true;
            }

            // check force magnitude
            //force_mag_reached = check_force_mag_reached(force_mag_goal_, force_mag);
        }


        // write to unity
        force_mag_share_.write(force_mag);

        // write to daq
        daq_->write_all();

        // check for stop input
        stop_ = check_stop();

        // log data
        log_robot_row();

        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    // transition to next state from "PRESENT TARGET"
    if (stop_) {
        // stop if user provided input
        event(ST_STOP);
    }
    else if (keep_mvc) {
        store_buffer(emg_calibration_data_buffer_, tkeo_active_data_);
        event(ST_TO_CENTER);
    }
    else if (redo_mvc) {
        event(ST_PRESENT_TARGET);
    }
    else if (active_state_reached) {      
        if (process_emg()) {
            if (is_training()) {
                event(ST_TO_CENTER);
            }
            else {
                classify();
                if (is_blind()) {
                    event(ST_TO_CENTER);
                }
                else {
                    event(ST_TO_TARGET);
                }
            }
        }
        else {
            util::print("WARNING: EMG data unsuccessfully processed. Going to ST_STOP.");
            event(ST_STOP);
        }       
    }
    else {
        util::print("ERROR: State transition undefined. Going to ST_STOP.");
        event(ST_STOP);
    }
}



//-----------------------------------------------------------------------------
// "GO TO TARGET" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRTControl::sf_to_target(const util::NoEventData* data) {
    util::print("Go to Target");

    // initialize local state variables
    bool target_reached = false;
    double_vec command_torques(meii_.N_aj_, 0.0);

    // set new reference position
    double_vec target_pos = get_target_position(class_label_sequence_[current_class_label_idx_]);
    meii_.anat_ref_.set_ref(target_pos, clock_.time());

    // write to Unity
    set_viz_target_num(class_label_sequence_[current_class_label_idx_]);
    viz_target_num_share_.write(viz_target_num_); // do we want to show the correct target or the target they're moving to???

    // enter the control loop
    while (!target_reached && !stop_) {

        // read and reload DAQs
        daq_->reload_watchdog();
        daq_->read_all();

        // update robot kinematics
        meii_.update_kinematics();

        // write kinematics to MelScope
        pos_share_.write(meii_.get_anatomical_joint_positions());
        vel_share_.write(meii_.get_anatomical_joint_velocities());

        // check joint limits
        if (meii_.check_all_joint_limits()) {
            stop_ = true;
            break;
        }

        // run position control
        command_torques = meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, clock_.time());

        // write motor commands to MelScope
        torque_share_.write(command_torques);

        // write to daq
        daq_->write_all();

        // check for target reached
        target_reached = meii_.check_goal_anat_pos(target_pos, { 1, 1, 1, 1, 0 });

        // check for stop input
        stop_ = check_stop();

        // log data
        log_robot_row();

        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    // transition to next state from "GO TO TARGET"
    if (stop_) {
        // stop if user provided input
        event(ST_STOP);
    }
    else if (target_reached) {
        event(ST_HOLD_TARGET);
    }
    else {
        util::print("ERROR: State transition undefined. Going to ST_STOP.");
        event(ST_STOP);
    }
}


//-----------------------------------------------------------------------------
// "HOLD AT TARGET" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRTControl::sf_hold_target(const util::NoEventData* data) {
    util::print("Hold at Target");

    // initialize global event variables
    

    // initialize local state variables
    bool hold_target_time_reached = false;
    double st_enter_time = clock_.time();
    double_vec command_torques(meii_.N_aj_, 0.0);


    // enter the control loop
    while (!hold_target_time_reached && !stop_) {

        // read and reload DAQs
        daq_->reload_watchdog();
        daq_->read_all();

        // update robot kinematics
        meii_.update_kinematics();

        // write kinematics to MelScope
        pos_share_.write(meii_.get_anatomical_joint_positions());
        vel_share_.write(meii_.get_anatomical_joint_velocities());

        // check joint limits
        if (meii_.check_all_joint_limits()) {
            stop_ = true;
            break;
        }

        // run position control
        command_torques = meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, clock_.time());

        // write motor commands to MelScope
        torque_share_.write(command_torques);

        // write to daq
        daq_->write_all();

        // check for hold time reached
        hold_target_time_reached = check_wait_time_reached(hold_target_time_, st_enter_time, clock_.time());

        // check for stop input
        stop_ = check_stop();

        // log data
        log_robot_row();

        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    // transition to next state from "HOLD TARGET"
    if (stop_) {
        // stop if user provided input
        event(ST_STOP);
    }
    else if (hold_target_time_reached) {
        event(ST_TO_CENTER);
    }
    else {
        util::print("ERROR: State transition undefined. Going to ST_STOP.");
        event(ST_STOP);
    }
}



//-----------------------------------------------------------------------------
// "FINISH EXPERIMENT" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRTControl::sf_finish(const util::NoEventData* data) {
    util::print("Finishing Experiment");

    // intialize local state variables
    bool menu_selected = false;
    bool exit_program = false;

    // disable robot
    if (meii_.is_enabled()) {
        meii_.disable();
    }

    // save data
    save_data();

    // wait for user input
    util::print("Press 'm' in Unity to return to the GUI main menu or press 'ENTER' to stop the experiment");
    while (!menu_selected && !exit_program && !stop_) {

        // read from Unity
        scene_num_share_.read(SCENE_NUM_);

        // check if menu selected in Unity
        if (SCENE_NUM_ == 0) {
            menu_selected = true;
        }

        // check for end of experiment
        if (util::Input::is_key_pressed(util::Input::Enter)) {
            exit_program = true;
        }

        // check for stop input
        stop_ = check_stop();

        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    // transition to next state from "FINISH"
    if (stop_) {
        // stop if user provided input
        event(ST_STOP);
    }
    else if (menu_selected) {
        event(ST_WAIT_FOR_GUI);
    }
    else if (exit_program) {
        // do nothing
    }
    else {
        util::print("ERROR: State transition undefined. Going to ST_STOP.");
        event(ST_STOP);
    }
}


//-----------------------------------------------------------------------------
// "STOP" STATE FUNCTION
//-----------------------------------------------------------------------------

void EmgRTControl::sf_stop(const util::NoEventData* data) {
    std::cout << "Program Stopped" << std::endl;

    // disable robot and daq
    if (meii_.is_enabled()) {
        meii_.disable();
    }
    if (daq_->is_enabled()) {
        daq_->disable();
    }

    // ask user if want to save data
    util::print("The was stopped manually or with a fault. Would you still like to save the data to the 'Error Report' directory (Y/N)?");
    util::Input::Key key = util::Input::wait_for_any_keys({ util::Input::Y, util::Input::N });
    switch (key) {
    case util::Input::Y:
        save_data();
        break;
    case util::Input::N:
        util::print("Data not saved.");
        break;
    }
}



//-----------------------------------------------------------------------------
// EMG REAL-TIME CONTROL UTILITY FUNCTIONS
//-----------------------------------------------------------------------------

bool EmgRTControl::process_emg() {
    util::print("Process EMG Data");

    // default return value to true; only write false if error encountered
    bool emg_data_processed = true; 

    // extract features from EMG data
    emg_feature_vec_ = feature_extract(emg_classification_data_buffer_);

    // check for successful data processing
    bool are_nan_features_ = false;
    for (int i = 0; i < emg_feature_vec_.size(); ++i) {
        if (std::isnan(emg_feature_vec_[i])) {
            are_nan_features_ = true;
            emg_feature_vec_[i] = 1.0;
        }
    }
    if (are_nan_features_) {
        util::print("WARNING: NaN feature(s) detected; replacing with 1.0 .");
        emg_data_processed = false;
    }

    // store feature array in training data set and log in training_log
    if (is_training()) {
        emg_feature_vec_.push_back(class_label_sequence_[current_class_label_idx_]);
        emg_training_data_.push_back(emg_feature_vec_);
        log_training_row();
    }

    return emg_data_processed;
}

int EmgRTControl::classify() {
    util::print("Classifying EMG Activation");

    // add dummy feature and convert feature vector to Eigen type
    emg_feature_vec_.push_back(1.0);
    util::print(emg_feature_vec_.size());
    Eigen::VectorXd emg_feature_vec_eig = math::stdvec_to_eigvec(emg_feature_vec_);

    // compute the distance from the classification boundary in feature space
    lda_dist_eig_ = lda_class_eig_ * emg_feature_vec_eig;

    util::print(lda_dist_eig_);

    // compare the result to the intercept to determine which class the features belong to
    if (is_single_dof()) {
        if (lda_dist_eig_[0] < 0) {
            classifier_result_ = 1;
        }
        else {
            classifier_result_ = 2;
        }
    }
    else {
        double_vec lda_prob = math::eigvec_to_stdvec(lda_dist_eig_);
        classifier_result_ = std::distance(lda_prob.begin(), std::max_element(lda_prob.begin(), lda_prob.end()));
        classifier_result_ += 1;
    }

    util::print(classifier_result_);
    log_training_row();

    emg_training_data_.clear();


    // set predicted class label
    pred_class_label_ = classifier_result_;

}


void EmgRTControl::tkeo_model_estimation() {

    // compute rest sample mean
    for (int i = 0; i < tkeo_rest_data_.size(); ++i) {
        rest_sample_mean_[i] = math::mean(tkeo_rest_data_[i]);
    }

    // compute rest sample covariance and inverse
    Eigen::VectorXd sample = Eigen::VectorXd::Zero(tkeo_rest_data_.size(), 1);
    for (int l = 0; l < tkeo_rest_data_[0].size(); ++l) {
        for (int i = 0; i < tkeo_rest_data_.size(); ++i) {
            sample[i] = tkeo_rest_data_[i][l];
        }
        sample -= rest_sample_mean_;

        rest_sample_cov_ += sample * sample.transpose();
    }
    rest_sample_cov_ = rest_sample_cov_ / (tkeo_rest_data_[0].size());

    rest_sample_cov_inv_ = rest_sample_cov_.fullPivLu().inverse();

    // compute active sample mean
    for (int i = 0; i < tkeo_active_data_.size(); ++i) {
        active_sample_mean_[i] = math::mean(tkeo_active_data_[i]);
    }

    // compute active sample covariance and inverse
    for (int l = 0; l < tkeo_active_data_[0].size(); ++l) {
        for (int i = 0; i < tkeo_active_data_.size(); ++i) {
            sample[i] = tkeo_active_data_[i][l];
        }
        sample -= active_sample_mean_;
        active_sample_cov_ += sample * sample.transpose();
    }
    active_sample_cov_ = active_sample_cov_ / (tkeo_active_data_[0].size());
    active_sample_cov_inv_ = active_sample_cov_.fullPivLu().inverse();

    // compute single sample threshold assuming uniform priors
    tkeo_threshold_ = std::log(rest_sample_cov_.fullPivLu().determinant()) - std::log(active_sample_cov_.fullPivLu().determinant())
        + rest_sample_mean_.transpose() * rest_sample_cov_inv_ * rest_sample_mean_
        - active_sample_mean_.transpose() * active_sample_cov_inv_ * active_sample_mean_;
}

double EmgRTControl::tkeo_sufficient_statistic(double_vec& sample_vec) {

    Eigen::VectorXd sample = math::stdvec_to_eigvec(sample_vec);
    double upsilon;
    upsilon = sample.transpose() * active_sample_cov_inv_ * sample; // -2 * sample.transpose() * active_sample_cov_inv_ * active_sample_mean_ -sample.transpose() * rest_sample_cov_inv_ * sample + 2 * sample.transpose() * rest_sample_cov_inv_ * rest_sample_mean_;
    upsilon -= 2 * sample.transpose() * active_sample_cov_inv_ * active_sample_mean_;
    upsilon -= sample.transpose() * rest_sample_cov_inv_ * sample;
    upsilon += 2 * sample.transpose() * rest_sample_cov_inv_ * rest_sample_mean_;
    return upsilon;
}

int EmgRTControl::tkeo_detector(double_vec& sample_vec) {
    int num_samples = 25;
    double alpha = 0.05;
    double beta = 0.95;
    double eta_0 = (1 - beta) / (1 - alpha);
    double eta_1 = beta / alpha;
    if (tkeo_sufficient_statistic(sample_vec) > tkeo_threshold_ + 2 * std::log(eta_1)) {
       ++tkeo_detector_memory_;
    }
    else {
        tkeo_detector_memory_ = 0;
    }
    /*else if (tkeo_sufficient_statistic(sample_vec) < tkeo_threshold_ + 2 * std::log(eta_0)) {
        --tkeo_detector_memory_;
    }*/

    if (tkeo_detector_memory_ > num_samples) {
        return 1;
    }
    else {
        return 0;
    }
}


//-----------------------------------------------------------------------------
// EXPERIMENT SETUP/CONDITIONS UTILITY FUNCTIONS
//-----------------------------------------------------------------------------


void EmgRTControl::save_data() {
    
    std::string filename;
    std::string directory;
    if (stop_) {
        directory = program_directory_ + "\\" + "ErrorReport";
    }
    else {
        directory = subject_dof_directory_;
    }

    if (subject_number_ < 10) {
        filename = "S0";
    }
    else {
        filename = "S";
    }
    filename += std::to_string(subject_number_) + "_" + str_dofs_[dof_] + "_" + str_conditions_[condition_];

    robot_log_.save_and_clear_data("robot_data_log_" + filename, directory, true);
    training_log_.save_and_clear_data("class_data_" + filename, directory, true);
    if (is_training()) {
        lda_log_.save_and_clear_data("LDA_Coeffs_" + filename, directory, true);
        //feature_log_.save_and_clear_data("selected_features_" + filename, directory, true);
    }

    // resetting column names in data logs
    robot_log_ = util::DataLog("robot_data_log", false);
    training_log_ = util::DataLog("training_log", false);
    lda_log_ = util::DataLog("lda_coeff_log", false);
    //feature_log_ = util::DataLog("feat_sel_log", false);
    
}

void EmgRTControl::log_robot_row() {
    std::vector<double> row;
    row.push_back(clock_.time());
    row.push_back(get_current_state());
    row.push_back(meii_.joints_[0]->get_position() * math::RAD2DEG);
    row.push_back(meii_.joints_[0]->get_velocity() * math::RAD2DEG);
    row.push_back(static_cast<core::Motor*>(meii_.actuators_[0])->get_torque_command());
    row.push_back(meii_.joints_[1]->get_position() * math::RAD2DEG);
    row.push_back(meii_.joints_[1]->get_velocity() * math::RAD2DEG);
    row.push_back(static_cast<core::Motor*>(meii_.actuators_[1])->get_torque_command());
    row.push_back(meii_.joints_[2]->get_position() * math::RAD2DEG);
    row.push_back(meii_.joints_[2]->get_velocity() * math::RAD2DEG);
    row.push_back(static_cast<core::Motor*>(meii_.actuators_[2])->get_torque_command());
    row.push_back(meii_.joints_[3]->get_position() * math::RAD2DEG);
    row.push_back(meii_.joints_[3]->get_velocity() * math::RAD2DEG);
    row.push_back(static_cast<core::Motor*>(meii_.actuators_[3])->get_torque_command());
    row.push_back(meii_.joints_[4]->get_position() * math::RAD2DEG);
    row.push_back(meii_.joints_[4]->get_velocity() * math::RAD2DEG);
    row.push_back(static_cast<core::Motor*>(meii_.actuators_[4])->get_torque_command());
    robot_log_.add_row(row);
}

void EmgRTControl::log_training_row() {
    std::vector<double> row;
    row.push_back(clock_.time());
    row.push_back(dof_);
    row.push_back(condition_);
    for (auto i = 0; i < meii_.N_emg_ * num_features_; ++i) {
        row.push_back(emg_feature_vec_[i]);
    }
    row.push_back(class_label_sequence_[current_class_label_idx_]);
    if (condition_ > 0) {
        row.push_back(classifier_result_);
        row.push_back(lda_dist_eig_[0]);
        if (dof_ > 4) {
            row.push_back(lda_dist_eig_[1]);
            row.push_back(lda_dist_eig_[2]);
            row.push_back(lda_dist_eig_[3]);
        }
    }
    training_log_.add_row(row);
}


bool EmgRTControl::check_stop() {
    std::vector<util::Input::Key> keys{ util::Input::LControl, util::Input::C };
    return (util::Input::are_all_keys_pressed(keys, false) | stop_);
}

void EmgRTControl::set_experiment_conditions(int scene_num) {
    dof_ = (scene_num - 2) / 4;
    condition_ = (scene_num - 2) % 4;
}

std::vector<int> EmgRTControl::gen_rand_class_labels(int num_labels) const {
    std::random_device rd_seed; // random seed
    std::mt19937 rd_gen(rd_seed()); // random unsigned integer generator
    int num_class;
    if (is_single_dof()) {
        num_class = 2;
    }
    else {
        num_class = 4;
    }
    std::uniform_int_distribution<> class_labels_dist(1, num_class);
    std::vector<int> class_label_list(num_labels, 0);
    for (int i = 0; i < num_labels; ++i) {
        class_label_list[i] = class_labels_dist(rd_gen);
    }
}

std::vector<int> EmgRTControl::rand_shuffle_class_labels(int num_labels_per_class) const {
    int num_class;
    if (is_single_dof()) {
        num_class = 2;
    }
    else {
        num_class = 4;
    }
    std::vector<int> class_label_list(num_labels_per_class * num_class, 0);
    for (int i = 0; i < num_labels_per_class * num_class; ++i) {
        class_label_list[i] = 1 + (i / num_labels_per_class);
    }
    std::srand(std::time(0));
    std::random_shuffle(class_label_list.begin(), class_label_list.end());
    return class_label_list;
}

void EmgRTControl::set_viz_target_num(int class_label) {
    viz_target_num_ = class_label;
}

double_vec EmgRTControl::get_target_position(int class_label) const {
    if (is_single_dof()) {
        return single_dof_targets_[hand_num_][dof_][class_label - 1];
    }
    else {
        return multi_dof_targets_[hand_num_][dof_ - 4][class_label - 1];
    }
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



bool EmgRTControl::check_wait_time_reached(double wait_time, double init_time, double current_time) const {
    return (current_time - init_time) > wait_time;
}

double EmgRTControl::measure_task_force(double_vec commanded_torques, int target_num, int dof, int condition) const {

    double task_force;
    std::vector<char_vec> target_dir;

    switch (task_force_measurement_mode_) {
    case 0: 
        if (hand_def_ == "R") {
            target_dir = target_dir_R_;
        }
        else {
            target_dir = target_dir_L_;
        }

        if (dof < 4) {
            task_force = (-1.0) * force_mag_goal_ * (commanded_torques[dof] + gravity_offsets_[dof])  / force_dof_scale_[dof][target_num - 1][dof] * target_dir[dof][target_num - 1];
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
        util::print("ERROR: Invalid choice for task_force_measurement_mode_. Must be 0 to 2.");
        return NAN;
    }
}


bool EmgRTControl::check_force_mag_reached(double force_mag_goal, double force_mag) {
    force_mag_time_now_ = clock_.global_time();
    force_mag_maintained_ = std::abs(force_mag_maintained_ + std::copysign(1.0, force_mag_tol_ - std::abs(force_mag_goal - force_mag)) * (force_mag_time_now_ - force_mag_time_last_));
    force_mag_time_last_ = force_mag_time_now_;
    return force_mag_maintained_ > force_mag_dwell_time_;
}

template <typename T>
bool EmgRTControl::read_csv(std::string filename, std::string directory, std::vector<std::vector<T>>& output) {
    std::string full_filename = directory + "\\" + filename + ".csv";
    std::ifstream input(full_filename);
    if (input.is_open()) {
        std::string csv_line;
        while (std::getline(input, csv_line)) {
            std::istringstream csv_stream(csv_line);
            std::vector<T> row;
            std::string number;
            T data;
            while (std::getline(csv_stream, number, ',')) {
                data = std::atof(number.c_str());
                row.push_back(data);
            }
            output.push_back(row);
        }
        return true;
    }
    else {
        util::print("ERROR: File not found.");
        return false;
    }
}
template bool EmgRTControl::read_csv<int>(std::string filename, std::string directory, std::vector<std::vector<int>>& output);
template bool EmgRTControl::read_csv<double>(std::string filename, std::string directory, std::vector<std::vector<double>>& output);

template <typename T>
bool EmgRTControl::write_csv(std::string filename, std::string directory, const std::vector<std::vector<T>>& input) const {
    std::string full_filename = directory + "\\" + filename + ".csv";
    boost::filesystem::path dir(directory.c_str());
    boost::filesystem::create_directories(dir);
    std::ofstream output(full_filename);
    std::cout << output.fail() << std::endl;
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
        util::print("ERROR: File not found.");
        return false;
    }
}
template bool EmgRTControl::write_csv<int>(std::string filename, std::string directory, const std::vector<std::vector<int>>& input) const;
template bool EmgRTControl::write_csv<double>(std::string filename, std::string directory, const std::vector<std::vector<double>>& input) const;

int EmgRTControl::is_any_num_key_pressed() const {
    std::vector<util::Input::Key> keys = { util::Input::Num0, util::Input::Num1, util::Input::Num2, util::Input::Num3, util::Input::Num4, util::Input::Num5, util::Input::Num6, util::Input::Num7, util::Input::Num8, util::Input::Num9 };
    util::Input::Key key = util::Input::are_any_keys_pressed(keys);
    switch (key) {
    case util::Input::Num0:
        return 0;
    case util::Input::Num1:
        return 1;
    case util::Input::Num2:
        return 2;
    case util::Input::Num3:
        return 3;
    case util::Input::Num4:
        return 4;
    case util::Input::Num5:
        return 5;
    case util::Input::Num6:
        return 6;
    case util::Input::Num7:
        return 7;
    case util::Input::Num8:
        return 8;
    case util::Input::Num9:
        return 9;
    default: return -1;
    }
}


void EmgRTControl::store_buffer(const exo::MahiExoIIEmg::EmgDataBuffer& data_buffer, std::vector<double_vec>& data_matrix) {
    if (data_matrix.empty()) {
        for (int i = 0; i < data_buffer.num_channels_; ++i) {
            data_matrix.push_back(double_vec());
        }
    }
    
    if (data_matrix.size() != data_buffer.num_channels_) {
        util::print("ERROR: Number of channels in data buffer does not match number of rows in data matrix.");
        return;
    }
    double_vec data_buffer_channel;
    for (int i = 0; i < data_buffer.num_channels_; ++i) {
        data_buffer_channel = data_buffer.get_channel(i);
        data_matrix[i].insert(data_matrix[i].end(), data_buffer_channel.begin(), data_buffer_channel.end());
    }
}



void EmgRTControl::write_calibration_data() {

    // store in a csv-writable data type
    Eigen::VectorXd tkeo_threshold_eig = Eigen::VectorXd::Ones(tkeo_rest_data_.size(), 1) * tkeo_threshold_;
    Eigen::MatrixXd calibration_data_eig(rest_sample_mean_.rows(), rest_sample_mean_.cols() + rest_sample_cov_.cols() + rest_sample_cov_inv_.cols() + active_sample_mean_.cols() + active_sample_cov_.cols() + active_sample_cov_inv_.cols() + tkeo_threshold_eig.cols());
    calibration_data_eig << rest_sample_mean_, rest_sample_cov_, rest_sample_cov_inv_, active_sample_mean_, active_sample_cov_, active_sample_cov_inv_, tkeo_threshold_eig;
    calibration_data_ = math::eigmat_to_stdvecvec(calibration_data_eig);

    // write to csv
    util::print("Writing calibration data to " + subject_dof_directory_ + "\\" + calibration_data_filename_);
    if (!write_csv(calibration_data_filename_, subject_dof_directory_, calibration_data_)) {
        stop_ = true;
    }
}

void EmgRTControl::load_calibration_data() {

    // read from csv
    util::print("Loading calibration data from " + subject_dof_directory_ + "\\" + calibration_data_filename_);
    if (!read_csv<double>(calibration_data_filename_, subject_dof_directory_, calibration_data_)) {
        stop_ = true;
    }
    
    // parse csv data into separate variables
    int rows = calibration_data_.size();
    int cols = calibration_data_[0].size();
    Eigen::MatrixXd calibration_data_eig = math::stdvecvec_to_eigmat(calibration_data_);
    int it = 0;
    
    int width = 1;
    rest_sample_mean_ = calibration_data_eig.block(0, it, rows, width);
    it += width;

    width = rows;
    rest_sample_cov_ = calibration_data_eig.block(0, it, rows, width);
    it += width;

    width = rows;
    rest_sample_cov_inv_ = calibration_data_eig.block(0, it, rows, width);
    it += width;

    width = 1;
    active_sample_mean_ = calibration_data_eig.block(0, it, rows, width);
    it += width;

    width = rows;
    active_sample_cov_ = calibration_data_eig.block(0, it, rows, width);
    it += width;

    width = rows;
    active_sample_cov_inv_ = calibration_data_eig.block(0, it, rows, width);
    it += width;

    width = 1;
    Eigen::VectorXd tkeo_threshold_eig = calibration_data_eig.block(0, it, rows, width);
    tkeo_threshold_ = tkeo_threshold_eig[0];
}

void EmgRTControl::load_classifier() {

    util::print("Loading classifier from " + subject_dof_directory_ + "\\" + lda_classifier_filename_);
    if (!read_csv(lda_classifier_filename_, subject_dof_directory_, lda_classifier_)) {
        stop_ = true;
    }

    // convert classifier to Eigen type matrix
    lda_class_eig_ = Eigen::MatrixXd::Zero(lda_classifier_.size(), lda_classifier_[0].size());
    for (int i = 0; i < lda_classifier_.size(); ++i) {
        lda_class_eig_.row(i) = math::stdvec_to_eigvec(lda_classifier_[i]);
    }
}


double_vec EmgRTControl::feature_extract(const exo::MahiExoIIEmg::EmgDataBuffer& emg_data_buffer) const {

    double_vec feature_vec;
    double_vec nrms_vec(meii_.N_emg_, 0.0);
    double_vec nmav_vec(meii_.N_emg_, 0.0);
    double_vec nwl_vec(meii_.N_emg_, 0.0);
    double_vec nzc_vec(meii_.N_emg_, 0.0);
    double_vec nssc_vec(meii_.N_emg_, 0.0);
    double_vec ar_vec(4,0.0);
    double_vec ar1_vec(meii_.N_emg_, 0.0);
    double_vec ar2_vec(meii_.N_emg_, 0.0);
    double_vec ar3_vec(meii_.N_emg_, 0.0);
    double_vec ar4_vec(meii_.N_emg_, 0.0);

    // extract unnormalized features
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

    // normalize time-domain features
    double rms_mean = math::mean(nrms_vec);
    double mav_mean = math::mean(nmav_vec);
    double wl_mean = math::mean(nwl_vec);
    double zc_mean = math::mean(nzc_vec);
    double ssc_mean = math::mean(nssc_vec);
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
    
    // copy features into one vector
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

double EmgRTControl::rms_feature_extract(const double_vec& emg_channel_buffer) const {
    double sum_squares = 0;
    for (int i = 0; i < emg_channel_buffer.size(); ++i) {
        sum_squares += std::pow(emg_channel_buffer[i], 2);
    }
    return std::sqrt(sum_squares / emg_channel_buffer.size());
}

double EmgRTControl::mav_feature_extract(const double_vec& emg_channel_buffer) const {
    double sum_abs = 0;
    for (int i = 0; i < emg_channel_buffer.size(); ++i) {
        sum_abs += std::abs(emg_channel_buffer[i]);
    }
    return sum_abs / emg_channel_buffer.size();
}

double EmgRTControl::wl_feature_extract(const double_vec& emg_channel_buffer) const {
    double sum_abs_diff = 0.0;
    for (int i = 0; i < emg_channel_buffer.size() - 1; ++i) {       
        sum_abs_diff += std::abs(emg_channel_buffer[i + 1] - emg_channel_buffer[i]);
    }
    return sum_abs_diff;
}

double EmgRTControl::zc_feature_extract(const double_vec& emg_channel_buffer) const {
    double sum_abs_diff_sign = 0;
    for (int i = 0; i < emg_channel_buffer.size() - 1; ++i) {
        sum_abs_diff_sign += std::abs(std::copysign(1.0, emg_channel_buffer[i + 1]) - std::copysign(1.0, emg_channel_buffer[i]));
    }
    return sum_abs_diff_sign / 2;
}

double EmgRTControl::ssc_feature_extract(const double_vec& emg_channel_buffer) const {
    double sum_abs_diff_sign_diff = 0;
    for (int i = 0; i < emg_channel_buffer.size() - 2; ++i) {
        sum_abs_diff_sign_diff += std::abs(std::copysign(1.0, (emg_channel_buffer[i + 2] - emg_channel_buffer[i + 1])) - std::copysign(1.0, (emg_channel_buffer[i + 1] - emg_channel_buffer[i])));
    }
    return sum_abs_diff_sign_diff / 2;
}

void EmgRTControl::ar_4_feature_extract(double_vec& coeffs, const double_vec& emg_channel_buffer) const {

    // initialize
    size_t N = emg_channel_buffer.size();
    size_t m = coeffs.size();
    double_vec A_k(m + 1, 0.0);
    A_k[0] = 1.0;
    double_vec f = emg_channel_buffer;
    double_vec b = emg_channel_buffer;
    double D_k = 0;
    for (size_t j = 0; j <= N; ++j) {
        D_k += 2.0 * std::pow(f[j], 2);
    }
    D_k -= std::pow(f[0], 2) + std::pow(b[N-1], 2);
    
    // Burg recursion
    for (size_t k = 0; k < m; ++k) {

        // compute mu
        double mu = 0.0;
        for (size_t n = 0; n <= N - k - 1; ++n) {
            mu += f[n + k + 1] * b[n];
        }
        mu *= -2.0 / D_k;

        // update A_k
        for (size_t n = 0; n <= (k + 1) / 2; ++n) {
            double t1 = A_k[n] + mu * A_k[k + 1 - n];
            double t2 = A_k[k + 1 - n] + mu * A_k[n];
            A_k[n] = t1;
            A_k[k + 1 - n] = t2;
        }

        // update f and b
        for (size_t n = 0; n <= N - k - 1; ++n) {
            double t1 = f[n + k + 1] + mu * b[n];
            double t2 = b[n] + mu * f[n + k + 1];
            f[n + k + 1] = t1;
            b[n] = t2;
        }

        // update D_k
        D_k = (1.0 - std::pow(mu, 2)) * D_k - std::pow(f[k + 1], 2) - std::pow(b[N - k - 1], 2);
    }

    // assign coefficients
    coeffs.assign(++A_k.begin(), A_k.end());
}
