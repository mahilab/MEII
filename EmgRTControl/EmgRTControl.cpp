#include "EmgRTControl.h"
#include "Input.h"
#include "Motor.h"
#include <iostream>
#include <fstream>
#include <string>
#include <random>


using namespace mel;

EmgRTControl::EmgRTControl(util::Clock& clock, core::Daq* daq, exo::MahiExoIIEmg& meii) :
    StateMachine(15),
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
    lda_intercept_.clear();
    emg_training_data_.clear();

    // create subject and DoF specific folder for data logging
    subject_dof_directory_ = subject_directory_ + "\\" + str_dofs_[dof_];

    // generate file names
    if (subject_number_ < 10) {
        training_data_filename_ = "S0";
        lda_classifier_filename_ = "S0";
    }
    else {
        training_data_filename_ = "S";
        lda_classifier_filename_ = "S";
    }
    training_data_filename_ += std::to_string(subject_number_) + "_" + str_dofs_[dof_] + "_training_data";
    lda_classifier_filename_ += std::to_string(subject_number_) + "_" + str_dofs_[dof_] + "_lda_classifier";

    // reset robot
    meii_.disable();

    /*// ask user if they want to use the existing training data
    util::print("Press Enter to collect new training data, or press L to Load previous training data...");
    bool load_training_data = false;
    bool key_pressed = false;
    util::Input::Key key;
    double init_time = clock_.time();
    double user_input_timeout = 5.0;
    while (!key_pressed && !stop_) {

        // check timeout
        if (check_wait_time_reached(user_input_timeout, init_time, clock_.time())) {
            key_pressed = false;
            break;
        }

        // check for user input
        key = util::Input::are_any_keys_pressed({ util::Input::Enter,util::Input::L },false);
        switch (key) {
        case util::Input::Key::Enter:
            key_pressed = true;
            break;
        case util::Input::Key::L:
            key_pressed = true;
            load_training_data = true;
            break;
        default: key_pressed = false;
        }

        // check for stop input
        stop_ = check_stop();

        // wait for the next clock cycle
        clock_.hybrid_wait();
    }
    if (load_training_data) {
        util::print("Loading previous training data.");
    }
    else {
        util::print("Collecting new training data.");
    }

    // load existing training data
    std::vector<std::vector<double>> old_training_data_;
    if (load_training_data) {
        util::print(subject_dof_directory_ + training_data_filename_);
        read_csv(subject_dof_directory_ + training_data_filename_, old_training_data_);
        util::print(old_training_data_[0]);
    }*/
    
    

    
    
    /*// read in LDA classifier
    if (is_testing()) {
        read_csv(program_directory_+"LDA_coeffs.csv", lda_classifier_);
        read_csv(program_directory_+"intercept.csv", lda_intercept_);
        read_csv(program_directory_+"feat_sel.csv", sel_feats_);

        size_t lda_rows = lda_classifier_.size();
        size_t lda_cols = lda_classifier_[0].size();

        // eigenization
        lda_class_eig_ = Eigen::MatrixXd::Zero(lda_rows, lda_cols);
        lda_inter_eig_ = Eigen::VectorXd::Zero(lda_rows);
        for (int i = 0; i < lda_rows; ++i) {
            lda_class_eig_.row(i) = math::stdv2eigenv(lda_classifier_[i]);
        }
        lda_inter_eig_ = math::stdv2eigenv(lda_intercept_[0]);
    }*/

    

    // Add columns to training logger
    training_log_.add_col("Time [s]").add_col("DoF").add_col("Condition");
    for (int i = 0; i < num_emg_channels_ * num_features_; ++i) {
        training_log_.add_col("Feat. " + std::to_string(i));
    }
    training_log_.add_col("Target");
    if (is_testing()) {
        training_log_.add_col("LDA Prob");
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

    // update global variables
    ++current_class_label_idx_;

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

    // initialize local state variables
    bool hold_center_time_reached = false;
    double st_enter_time = clock_.time();
    double_vec command_torques(meii_.N_aj_, 0.0);

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

        // write to daq
        daq_->write_all();

        // check for hold time reached
        hold_center_time_reached = check_wait_time_reached(hold_center_time_, st_enter_time, clock_.time());

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

    /*// initialize class label sequence to be presented
    if (class_labels_from_file_) {
        if (is_single_dof()) { // single-DoF
            std::ifstream input(program_directory_ + "target_sequences.txt");
            if (input.is_open()) {
                while (!input.eof()) {
                    std::string number;
                    int data;
                    std::getline(input, number);
                    data = std::atoi(number.c_str());
                    class_label_sequence_.push_back(data);
                }
            }
        }
        else { // multi-DoF
            std::ifstream input(program_directory_ + "target_sequences_multi.txt");
            if (input.is_open()) {
                while (!input.eof()) {
                    std::string number;
                    int data;
                    std::getline(input, number);
                    data = std::atoi(number.c_str());
                    class_label_sequence_.push_back(data);
                }
            }
        }
    }
    else {
        class_label_sequence_ = rand_shuffle_class_labels(4);
    }*/


    /*// read in LDA classifier
    if (is_testing()) {
        read_csv(program_directory_ + "LDA_coeffs", lda_classifier_);
        read_csv(program_directory_ + "intercept", lda_intercept_);
        read_csv(program_directory_ + "feat_sel", sel_feats_);

        size_t lda_rows = lda_classifier_.size();
        size_t lda_cols = lda_classifier_[0].size();

        // eigenization
        lda_class_eig_ = Eigen::MatrixXd::Zero(lda_rows, lda_cols);
        lda_inter_eig_ = Eigen::VectorXd::Zero(lda_rows);
        for (int i = 0; i < lda_rows; ++i) {
            lda_class_eig_.row(i) = math::stdv2eigenv(lda_classifier_[i]);
        }
        lda_inter_eig_ = math::stdv2eigenv(lda_intercept_[0]);
    }*/


    if (end_of_label_sequence_) {

        if (is_training()) {

            // write training data to file
            util::DataLog training_data("training_data", false);
            for (int i = 0; i < emg_training_data_.size(); ++i) {
                training_data.add_row(emg_training_data_[i]);
                util::print(emg_training_data_[i]);
            }
            training_data.save_and_clear_data(training_data_filename_, subject_dof_directory_, false);

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
            

            //size_t lda_rows = lda_classifier_.size();
            //size_t lda_cols = lda_classifier_[0].size();

            /*// data logging
            lda_log_.add_col("intercept");
            for (int i = 0; i < lda_cols; ++i) {
                lda_log_.add_col("LDA Coeff" + std::to_string(i));
            }
            double_vec lda_row(lda_cols + 1);
            for (int i = 0; i < lda_rows; ++i) {
                lda_row[0] = lda_intercept_[0][i];
                for (int j = 0; j < lda_cols; ++j) {
                    lda_row[j + 1] = lda_classifier_[i][j];
                }
                lda_log_.add_row(lda_row);
            }

            std::string col;
            for (int i = 0; i < lda_cols; i++) {
                col = "feature" + std::to_string(i);
                feature_log_.add_col(col);
            }

            std::vector<double> feature_row;
            for (int i = 0; i < lda_cols; i++) {
                feature_row.push_back(sel_feats_[0][i]);
            }
            feature_log_.add_row(feature_row);

            // eigenization
            lda_class_eig_ = Eigen::MatrixXd::Zero(lda_rows, lda_cols);
            lda_inter_eig_ = Eigen::VectorXd::Zero(lda_rows);
            for (int i = 0; i < lda_rows; ++i) {
                lda_class_eig_.row(i) = math::stdv2eigenv(lda_classifier_[i]);
            }
            lda_inter_eig_ = math::stdv2eigenv(lda_intercept_[0]);*/

            

        }
        else {
            finished = true;
        }
    }
    else {

        if (is_training()) {

            // ask user if they want to use the existing training data
            util::print("Press Enter to collect new training data, or press L to Load previous training data...");
        }
        else {

            util::print("Loading classifier from " + subject_dof_directory_ + "\\" + lda_classifier_filename_);
            if (!read_csv(lda_classifier_filename_, subject_dof_directory_, lda_classifier_)) {
                stop_ = true;
            }
            present_more_targets = true;
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

        // check for external input
        if (end_of_label_sequence_) {

            if (is_training()) {

                if (lda_training_complete == 0) {
                    lda_training_flag_.read(lda_training_complete);
                }
                else {

                    if (!python_return) {
                        python_return = true;

                        // read in the results from the Cross-validation test in Python
                        std::vector<double> cross_eval_test;
                        cv_results_.read(cross_eval_test);
                        util::print(cross_eval_test);

                        // read in csv's generated by python
                        read_csv("LDA_coeffs", subject_dof_directory_, lda_classifier_);
                        read_csv("intercept", subject_dof_directory_, lda_intercept_);
                        read_csv("feat_sel", subject_dof_directory_, sel_feats_);

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
            else {

            }
        }
        else { // not end of target sequence

            if (is_training()) {

                if (!more_training_data) {
                    key = util::Input::are_any_keys_pressed({ util::Input::Enter,util::Input::L });
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
                        present_more_targets = true;
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
        std::vector<int> new_class_labels = rand_shuffle_class_labels(num_observations_per_class);
        for (int i = 0; i < new_class_labels.size(); ++i) {
            class_label_sequence_.push_back(new_class_labels[i]);
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

    // initialize global event variables
    

    // initialize local state variables
    bool force_mag_reached = false;
    double_vec filtered_emg_voltages = double_vec(meii_.N_emg_, 0.0);
    double_vec command_torques(meii_.N_aj_, 0.0);
    double force_mag = 0.0;
    
    // initialize magnitude force checking algorithm
    force_mag_maintained_ = 0.0;
    force_mag_time_now_ = clock_.global_time();
    force_mag_time_last_ = clock_.global_time();

    /*// read from target sequence and write to Unity
    ++current_class_label_idx_;
    if (current_class_label_idx_ >= class_label_sequence_.size()) {
        end_of_target_sequence_ = true;
    }
    else {
        set_viz_target_num(class_label_sequence_[current_class_label_idx_]);
        viz_target_num_share_.write(viz_target_num_);
    }*/

    // read from target sequence and write to Unity
    util::print(class_label_sequence_);
    util::print(current_class_label_idx_);
    set_viz_target_num(class_label_sequence_[current_class_label_idx_]);
    viz_target_num_share_.write(viz_target_num_);

    // enter the control loop
    while (!end_of_label_sequence_ && !force_mag_reached && !stop_) {

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

        // measure interaction force for specified dof(s)
        force_mag = measure_task_force(command_torques, class_label_sequence_[current_class_label_idx_], dof_, condition_);

        // get measured emg voltages
        filtered_emg_voltages = meii_.butter_hp_.filter(meii_.get_emg_voltages()); 
        emg_data_buffer_.push_back(filtered_emg_voltages);
        emg_share_.write(emg_data_buffer_.at(0));

        // check force magnitude
        force_mag_reached = check_force_mag_reached(force_mag_goal_, force_mag);

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
    /*else if (end_of_target_sequence_) {
        if (condition_ == 0) {
            event(ST_TRAIN_CLASSIFIER);
        }
        else if (condition_ == 1 || condition_ == 2) {
            event(ST_FINISH);
        }
        else {
            util::print("ERROR: Invalid condition number while exiting state ST_PRESENT_TARGET.");
        }
    }*/
    else if (force_mag_reached) {   
        
        event(ST_PROCESS_EMG);
    }
    else {
        util::print("ERROR: State transition undefined. Going to ST_STOP.");
        event(ST_STOP);
    }
}


//-----------------------------------------------------------------------------
// "PROCESS EMG DATA" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRTControl::sf_process_emg(const util::NoEventData* data) {
    util::print("Process EMG Data");

    // initialize local state variables
    bool emg_data_processed = true; // default to true; only write false if error encountered

    // extract features from EMG data
    emg_feature_vec_ = feature_extract(emg_data_buffer_);

    // check for successful data processing
    bool are_nan_features_ = false;
    for (int i = 0; i < emg_feature_vec_.size(); ++i) {
        if (std::isnan(emg_feature_vec_[i])) {
            are_nan_features_ = true;
            emg_feature_vec_[i] = 1.0;
        }
    }
    if (are_nan_features_) {
        util::print("WARNING: NaN feature(s) detected. Replacing with 1.0 .");
    }

    // copy features into an array
    //std::copy_n(feature_vec_.begin(), feature_array_.size(), feature_array_.begin());

    // store feature array in training data set and log in training_log
    if (is_training()) {
        emg_feature_vec_.push_back(class_label_sequence_[current_class_label_idx_]);
        emg_training_data_.push_back(emg_feature_vec_);
        util::print(*emg_training_data_.end());
        log_training_row();
    }

    // transition to next state from "PROCESS EMG DATA"
    if (stop_) {
        // stop if user provided input
        event(ST_STOP);
    }
    else if (emg_data_processed) {   
        if (is_training()) {
            event(ST_TO_CENTER);
        }
        else {
            event(ST_CLASSIFY);
        }  
    }
    else if (!emg_data_processed) {
        util::print("WARNING: EMG data unsuccessfully processed. Going to ST_STOP.");
        event(ST_STOP);
    }
    else {
        util::print("ERROR: State transition undefined. Going to ST_STOP.");
        event(ST_STOP);
    }
}



//-----------------------------------------------------------------------------
// "TRAIN CLASSIFIER" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRTControl::sf_train_classifier(const util::NoEventData* data) {    
    util::print("Training Classifier");

    // disable robot
    meii_.disable();
    
    // log training data
    
    training_log_.save_data(training_data_filename_, subject_dof_directory_, false);

    // send file location to python over melshare
    directory_share_.write_message(subject_dof_directory_);
    file_name_share_.write_message(training_data_filename_);


    // open LDA script in Python
    std::string system_command;
    if (is_single_dof()) {
        system_command = "start " + program_directory_ + "EMG_FS_LDA.py &";
    }
    else {
        system_command = "start " + program_directory_ + "EMG_FS_LDA.py &";
    }
    system(system_command.c_str());


    // create vector of EMG features to send to Python
    /*double_vec emg_training_data_vec;
    emg_training_data_vec.reserve(N_train_ * meii_.N_emg_ * num_features_);
    for (int i = 0; i < N_train_; ++i) {
        for (int j = 0; j < meii_.N_emg_ * num_features_; ++j) {
            emg_training_data_vec.push_back(emg_training_data_[i][j]);
        }
    }*/

    // calculate size of training data to be sent to python
    //std::array<int, 2> training_data_size = { N_train_, meii_.N_emg_ * num_features_ };

    // copy training labels into an array
    //std::vector<int> training_labels(N_train_);
    //std::copy_n(class_label_sequence_.begin(), N_train_, training_labels.begin());

    // write training data to python
    //trng_size_.write(training_data_size);
    //trng_share_.write(emg_training_data_vec);
    //label_share_.write(training_labels);



    /*// wait to receive trained classifier from python
    util::print("Waiting for Python to return training results...");
    while ((lda_training_complete_ == 0) && !stop_) {

        // check for flag
        lda_training_flag_.read(lda_training_complete_);

        // check for stop input
        stop_ = check_stop();

        // wait for the next clock cycle
        clock_.hybrid_wait();
    }
    util::print("Python results received.");*/

    /*std::vector<double> cross_eval_test;
    //read in the results from the Cross-validation test in Python
    cv_results_.read(cross_eval_test);

    util::print(cross_eval_test);*/

    /*// read in csv's generated by python
    read_csv(program_directory_+"LDA_coeffs", lda_classifier_);
    read_csv(program_directory_ + "intercept", lda_intercept_);
    read_csv(program_directory_ + "feat_sel", sel_feats_);*/
    
    size_t lda_rows = lda_classifier_.size();
    size_t lda_cols = lda_classifier_[0].size();

    
    // data logging
    lda_log_.add_col("intercept");
    for (int i = 0; i < lda_cols; ++i) {
        lda_log_.add_col("LDA Coeff" + std::to_string(i));
    }
    double_vec lda_row(lda_cols + 1);
    for (int i = 0; i < lda_rows; ++i) {
        lda_row[0] = lda_intercept_[0][i];
        for (int j = 0; j < lda_cols; ++j) {
            lda_row[j+1] = lda_classifier_[i][j];
        }
        lda_log_.add_row(lda_row);
    }


    // std::vector<int> sel_feats(lda_cols);
    //read for selected feature indicies from Python
    //feat_id_.read(sel_feats);

    util::print(sel_feats_[0]);

    std::string col;
    for (int i = 0; i < lda_cols; i++) {
        col = "feature" + std::to_string(i);
        feature_log_.add_col(col);
    }

    std::vector<double> feature_row;
    for (int i = 0; i < lda_cols; i++) {
        feature_row.push_back(sel_feats_[0][i]);
    }
    feature_log_.add_row(feature_row);

    // eigenization
    lda_class_eig_ = Eigen::MatrixXd::Zero(lda_rows, lda_cols);
    lda_inter_eig_ = Eigen::VectorXd::Zero(lda_rows);
    for (int i = 0; i < lda_rows; ++i) {
        lda_class_eig_.row(i) = math::stdv2eigenv(lda_classifier_[i]);
    }
    lda_inter_eig_ = math::stdv2eigenv(lda_intercept_[0]);

    util::print(lda_class_eig_);
    util::print("\n");
    util::print(lda_inter_eig_);

    // transition to next state from "TRAIN CLASSIFIER"
    if (stop_) {
        // stop if user provided input
        event(ST_STOP);
    }
    else {
        event(ST_FINISH);
    }
}



//-----------------------------------------------------------------------------
// "CLASSIFY" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRTControl::sf_classify(const util::NoEventData* data) {

    util::print("Classifying EMG Activation");

    //Open MelShares to get feature vector dimensions and indicies
    //std::array<int, 2> training_data_size2;
    //read for number of selected features from Python
    //trng_size2_.read(training_data_size2_);

    size_t lda_rows = lda_classifier_.size();
    size_t lda_cols = lda_classifier_[0].size();

    //std::vector<int> sel_feats(lda_cols);
    //read for selected feature indicies from Python
    //feat_id_.read(sel_feats);

    //Create a vector to hold the selected features from the last trial
    std::vector<double> classify_features(lda_cols);

    //Extract the selected features from the full set and store them in vector
    for (int i = 0; i < lda_cols; ++i) {
        classify_features[i] = emg_feature_vec_[sel_feats_[0][i]];
        //classify_features[i] = emg_training_data_[0][sel_feats_[0][i]];
    }

    util::print(classify_features);

    // util::print(lda_class_eig_);



     //Convert vector of selected features to Eigen
     //Eigen::VectorXd classify_features_eig = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(classify_features.data(), classify_features.size());
    Eigen::VectorXd classify_features_eig = math::stdv2eigenv(classify_features);

    //util::print(classify_features_eig);

    //multiply the LDA coefficients and EMG features
    lda_dist_eig_ = lda_class_eig_ * classify_features_eig + lda_inter_eig_;

    util::print(lda_dist_eig_);

    //Compare the result to the intercept to determine which class the features belong to
    if (is_single_dof()) {
        //if (lda_prob_eig_[0] > lda_intercept_[0][0]) {
        if (lda_dist_eig_[0] < 0) {
            classifier_result_ = 1;
        }
        else {
            classifier_result_ = 2;
        }
    }
    else {
        double_vec lda_prob = math::eigenv2stdv(lda_dist_eig_);
        classifier_result_ = std::distance(lda_prob.begin(), std::max_element(lda_prob.begin(), lda_prob.end()));
        classifier_result_ += 1;
    }

    util::print(classifier_result_);
    log_training_row();

    emg_training_data_.clear();


    // set predicted class label
    pred_class_label_ = classifier_result_;

    // check for stop input
    stop_ = check_stop();

    // transition to next state from "CLASSIFY"
    if (stop_) {
        event(ST_STOP);
    }
    else {
        if (is_blind()) {
            event(ST_TO_CENTER);
        }
        else {
            event(ST_TO_TARGET);
        }
    }
}


//-----------------------------------------------------------------------------
// "GO TO TARGET" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRTControl::sf_to_target(const util::NoEventData* data) {
    util::print("Go to Target");

    // initialize global event variables
    

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

    if (is_training()) {
        
        // save classifier
    }

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
// UTILITY FUNCTIONS
//-----------------------------------------------------------------------------

void EmgRTControl::save_data() {
    
    std::string filename;
    std::string directory;
    if (stop_) {
        directory = program_directory_ + "\\" + "Error Report";
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
        feature_log_.save_and_clear_data("selected_features_" + filename, directory, true);
    }

    // resetting column names in data logs
    robot_log_ = util::DataLog("robot_data_log", false);
    training_log_ = util::DataLog("training_log", false);
    lda_log_ = util::DataLog("lda_coeff_log", false);
    feature_log_ = util::DataLog("feat_sel_log", false);
    
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
    dof_ = (scene_num - 2) / 3;
    condition_ = (scene_num - 2) % 3;
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
    return SCENE_NUM_ < 14;
}

bool EmgRTControl::is_training() const {
    return condition_ == 0;
}

bool EmgRTControl::is_testing() const {
    return condition_ == 1 || condition_ == 2;
}

bool EmgRTControl::is_blind() const {
    return condition_ == 1;
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

bool EmgRTControl::read_csv(std::string filename, std::string directory, std::vector<std::vector<double>>& output) {
    std::string full_filename = directory + "\\" + filename + ".csv";
    std::ifstream input(full_filename);
    if (input.is_open()) {
        std::string csv_line;
        while (std::getline(input, csv_line)) {
            std::istringstream csv_stream(csv_line);
            std::vector<double> row;
            std::string number;
            double data;
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

bool EmgRTControl::read_csv(std::string filename, std::string directory, std::vector<std::vector<int>>& output) {
    std::string full_filename = directory + "\\" + filename + ".csv";
    std::ifstream input(full_filename);
    if (input.is_open()) {
        std::string csv_line;
        while (std::getline(input, csv_line)) {
            std::istringstream csv_stream(csv_line);
            std::vector<int> row;
            std::string number;
            int data;
            while (std::getline(csv_stream, number, ',')) {
                data = std::atoi(number.c_str());
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


double_vec EmgRTControl::feature_extract(exo::MahiExoIIEmg::EmgDataBuffer& emg_data_buffer) {

    double_vec feature_vec;
    feature_vec.reserve(meii_.N_emg_ * num_features_);
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
        nrms_vec[i] = rms_feature_extract(emg_data_buffer.data_buffer_[i]);  
        //util::print(nrms_vec[i]);
        nmav_vec[i] = mav_feature_extract(emg_data_buffer.data_buffer_[i]);
        nwl_vec[i] = wl_feature_extract(emg_data_buffer.data_buffer_[i]);
        nzc_vec[i] = zc_feature_extract(emg_data_buffer.data_buffer_[i]);
        nssc_vec[i] = ssc_feature_extract(emg_data_buffer.data_buffer_[i]);
        ar4_feature_extract(ar_vec, emg_data_buffer.get_channel(i));
        ar1_vec[i] = ar_vec[0];
        ar2_vec[i] = ar_vec[1];
        ar3_vec[i] = ar_vec[2];
        ar4_vec[i] = ar_vec[3];
    }    
    util::print(nzc_vec);
    util::print("");

    // normalize features
    double rms_mean = 0.0;
    double mav_mean = 0.0;
    double wl_mean = 0.0;
    double zc_mean = 0.0;
    double ssc_mean = 0.0;
    for (int i = 0; i < meii_.N_emg_; ++i) {
        rms_mean += nrms_vec[i] / meii_.N_emg_;
        mav_mean += nmav_vec[i] / meii_.N_emg_;
        wl_mean += nwl_vec[i] / meii_.N_emg_;
        zc_mean += nzc_vec[i] / meii_.N_emg_;
        ssc_mean += nssc_vec[i] / meii_.N_emg_;
    }
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
    util::print(nzc_vec);
    // copy features into one vector (inserted in reverse order)
    auto it = feature_vec.begin();
    it = feature_vec.insert(it, ar4_vec.begin(), ar4_vec.end());
    it = feature_vec.insert(it, ar3_vec.begin(), ar3_vec.end());
    it = feature_vec.insert(it, ar2_vec.begin(), ar2_vec.end());
    it = feature_vec.insert(it, ar1_vec.begin(), ar1_vec.end());
    it = feature_vec.insert(it, nssc_vec.begin(), nssc_vec.end());
    it = feature_vec.insert(it, nzc_vec.begin(), nzc_vec.end());
    it = feature_vec.insert(it, nwl_vec.begin(), nwl_vec.end());
    it = feature_vec.insert(it, nmav_vec.begin(), nmav_vec.end());
    it = feature_vec.insert(it, nrms_vec.begin(), nrms_vec.end());
    return feature_vec;
}

double EmgRTControl::rms_feature_extract(boost::circular_buffer<double> emg_channel_buffer) {
    double sum_squares = 0;
    for (int i = 0; i < emg_channel_buffer.size(); ++i) {
        sum_squares += std::pow(emg_channel_buffer[i], 2);
    }
    return std::sqrt(sum_squares / emg_channel_buffer.size());
}

double EmgRTControl::mav_feature_extract(boost::circular_buffer<double> emg_channel_buffer) {
    double sum_abs = 0;
    for (int i = 0; i < emg_channel_buffer.size(); ++i) {
        sum_abs += std::abs(emg_channel_buffer[i]);
    }
    return sum_abs / emg_channel_buffer.size();
}

double EmgRTControl::wl_feature_extract(boost::circular_buffer<double> emg_channel_buffer) {
    double sum_abs_diff = 0.0;
    for (int i = 0; i < emg_channel_buffer.size() - 1; ++i) {       
        sum_abs_diff += std::abs(emg_channel_buffer[i + 1] - emg_channel_buffer[i]);
    }
    return sum_abs_diff;
}

double EmgRTControl::zc_feature_extract(boost::circular_buffer<double> emg_channel_buffer) {
    double sum_abs_diff_sign = 0;
    for (int i = 0; i < emg_channel_buffer.size() - 1; ++i) {
        sum_abs_diff_sign += std::abs(std::copysign(1.0, emg_channel_buffer[i + 1]) - std::copysign(1.0, emg_channel_buffer[i]));
    }
    return sum_abs_diff_sign / 2;
}

double EmgRTControl::ssc_feature_extract(boost::circular_buffer<double> emg_channel_buffer) {
    double sum_abs_diff_sign_diff = 0;
    for (int i = 0; i < emg_channel_buffer.size() - 2; ++i) {
        sum_abs_diff_sign_diff += std::abs(std::copysign(1.0, (emg_channel_buffer[i + 2] - emg_channel_buffer[i + 1])) - std::copysign(1.0, (emg_channel_buffer[i + 1] - emg_channel_buffer[i])));
    }
    return sum_abs_diff_sign_diff / 2;
}

void EmgRTControl::ar4_feature_extract(double_vec& coeffs, const double_vec& emg_channel_buffer) {

    // initialize
    size_t N = emg_channel_buffer.size();
    size_t m = coeffs.size();
    double_vec A_k(m + 1, 0.0);
    A_k[0] = 1.0;
    double_vec f(emg_channel_buffer);
    double_vec b(emg_channel_buffer);
    double D_k = 0;
    for (size_t j = 0; j <= N; ++j) {
        D_k += 2.0 * std::pow(f[j], 2);
    }
    D_k -= std::pow(f[0], 2) + std::pow(b[N], 2);

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
