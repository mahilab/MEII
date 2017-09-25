#include "EmgRTControl.h"
#include "Input.h"
#include "Motor.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace mel;

EmgRTControl::EmgRTControl(util::Clock& clock, core::Daq* daq, exo::MahiExoIIEmg& meii) :
    StateMachine(14),
    clock_(clock),
    daq_(daq),
    meii_(meii)
{
    // create subject folder for data logging
    if (subject_number_ < 10) {
        directory_ = "C:\\Users\\Ted\\GitHub\\MEII\\EmgRTControl\\EMG_S0" + std::to_string(subject_number_);
    }
    else {
        directory_ = "C:\\Users\\Ted\\GitHub\\MEII\\EmgRTControl\\EMG_S" + std::to_string(subject_number_);
    } 
}   


//-----------------------------------------------------------------------------
// "WAIT FOR GUI" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRTControl::sf_wait_for_gui(const util::NoEventData* data) {
    util::print("Waiting for Gui Input");

    // initialize global event variables
    scene_selected_ = false;

    // launch game
    game.launch();

    // enter the control loop
    while (!scene_selected_ && !stop_) {

        // read from Unity
        scene_num_share_.read(SCENE_NUM_);

        // check if scene selected in Unity
        if (SCENE_NUM_ > 0) {
            scene_selected_ = true;
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
    else if (scene_selected_) {
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

    std::vector<std::string> str_conditions = { "Training", "Blind Testing", "Full Testing" };
    std::vector<std::string> str_dofs = { "Elbow F/E Single-DoF", "Forearm P/S Single-Dof", "Wrist F/E Single-DoF", "Wrist R/U Single-DoF", "Elbow F/E & Forearm P/S Multi-DoF", "Wrist F/E & Wrist R/U Multi-DoF" };
    util::print("Initializing for " + str_conditions[condition_] + " of " + str_dofs[dof_]);

    // initialize global event variables
    menu_selected_ = false;
    scene_selected_ = true;
    init_backdrive_time_reached_ = false;
    rps_init_ = false;
    target_reached_ = false;
    hold_target_time_reached_ = false;
    force_mag_reached_ = false;
    emg_data_processed_ = false;
    end_of_target_sequence_ = false;
    lda_training_complete_ = 0;

    // reset global experiment variables
    class_label_sequence_.clear();
    current_class_label_idx_ = -1;
    viz_target_num_ = 0;
    pred_class_label_ = 0;
    lda_classifier_.clear();
    lda_intercept_.clear();
    emg_training_data_.clear();

    // reset position control
    meii_.rps_init_par_ref_.stop();
    meii_.anat_ref_.stop();
    
    // read in class label sequence from file
    if (is_single_dof()) { // single-DoF
        std::ifstream input("target_sequences.txt");
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
        std::ifstream input("target_sequences_multi.txt");
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
    util::print("Reading in Class Label Sequence: ");
    util::print(class_label_sequence_);
    

    if (dof_ >= 4) {
        N_train_ = 8;
        num_class_ = 4;
    }
    else {
        //N_train_ = 10;
        N_train_ = class_label_sequence_.size();
        util::print(N_train_);
        num_class_ = 2;
    }


    // read in LDA classifier
    if (is_testing()) {
        read_csv("LDA_coeffs.csv", lda_classifier_);
        read_csv("intercept.csv", lda_intercept_);
        read_csv("feat_sel.csv", sel_feats_);

        size_t lda_rows = lda_classifier_.size();
        size_t lda_cols = lda_classifier_[0].size();

        // eigenization
        lda_class_eig_ = Eigen::MatrixXd::Zero(lda_rows, lda_cols);
        lda_inter_eig_ = Eigen::VectorXd::Zero(lda_rows);
        for (int i = 0; i < lda_rows; ++i) {
            lda_class_eig_.row(i) = math::stdv2eigenv(lda_classifier_[i]);
        }
        lda_inter_eig_ = math::stdv2eigenv(lda_intercept_[0]);
    }

    

    // Add columns to logger
    log_.add_col("Time [s]").add_col("DoF").add_col("Condition");
    for (int i = 0; i < num_emg_channels_ * num_features_; ++i) {
        log_.add_col("Feat. " + std::to_string(i));
    }
        /*.add_col("Ch. 1 RMS").add_col("Ch. 1 MAV").add_col("Ch. 1 WL").add_col("Ch. 1 ZC").add_col("Ch. 1 SSC").add_col("Ch. 1 AR1").add_col("Ch. 1 AR2").add_col("Ch. 1 AR3").add_col("Ch. 1 AR4")
        .add_col("Ch. 2 RMS").add_col("Ch. 2 MAV").add_col("Ch. 2 WL").add_col("Ch. 2 ZC").add_col("Ch. 2 SSC").add_col("Ch. 2 AR1").add_col("Ch. 2 AR2").add_col("Ch. 2 AR3").add_col("Ch. 2 AR4")
        .add_col("Ch. 3 RMS").add_col("Ch. 3 MAV").add_col("Ch. 3 WL").add_col("Ch. 3 ZC").add_col("Ch. 3 SSC").add_col("Ch. 3 AR1").add_col("Ch. 3 AR2").add_col("Ch. 3 AR3").add_col("Ch. 3 AR4")
        .add_col("Ch. 4 RMS").add_col("Ch. 4 MAV").add_col("Ch. 4 WL").add_col("Ch. 4 ZC").add_col("Ch. 4 SSC").add_col("Ch. 4 AR1").add_col("Ch. 4 AR2").add_col("Ch. 4 AR3").add_col("Ch. 4 AR4")
        .add_col("Ch. 5 RMS").add_col("Ch. 5 MAV").add_col("Ch. 5 WL").add_col("Ch. 5 ZC").add_col("Ch. 5 SSC").add_col("Ch. 5 AR1").add_col("Ch. 5 AR2").add_col("Ch. 5 AR3").add_col("Ch. 5 AR4")
        .add_col("Ch. 6 RMS").add_col("Ch. 6 MAV").add_col("Ch. 6 WL").add_col("Ch. 6 ZC").add_col("Ch. 6 SSC").add_col("Ch. 6 AR1").add_col("Ch. 6 AR2").add_col("Ch. 6 AR3").add_col("Ch. 6 AR4")
        .add_col("Ch. 7 RMS").add_col("Ch. 7 MAV").add_col("Ch. 7 WL").add_col("Ch. 7 ZC").add_col("Ch. 7 SSC").add_col("Ch. 7 AR1").add_col("Ch. 7 AR2").add_col("Ch. 7 AR3").add_col("Ch. 7 AR4")
        .add_col("Ch. 8 RMS").add_col("Ch. 8 MAV").add_col("Ch. 8 WL").add_col("Ch. 8 ZC").add_col("Ch. 8 SSC").add_col("Ch. 8 AR1").add_col("Ch. 8 AR2").add_col("Ch. 8 AR3").add_col("Ch. 8 AR4")
        .add_col("Target No.");*/
    if (is_testing()) {
        log_.add_col("LDA class").add_col("LDA Probability");
    }   

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

    // start the clock
    clock_.start();

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

    // initialize global event variables
    init_backdrive_time_reached_ = false;

    // initialize local state variables
    double st_enter_time = clock_.time();
    const double_vec command_torques(meii_.N_aj_, 0.0);

    // enter the control loop
    while (!init_backdrive_time_reached_ && !stop_) {

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
        init_backdrive_time_reached_ = check_wait_time_reached(init_backdrive_time_, st_enter_time, clock_.time());

        // check for stop input
        stop_ = check_stop();

        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    // transition to next state from "BACKDRIVE"
    if (stop_) {
        // stop if user provided input
        event(ST_STOP);
    }
    else if (init_backdrive_time_reached_) {
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

    // initialize global event variables
    rps_init_ = false;

    // initialize rps initialization position controller
    meii_.set_rps_control_mode(0);
    meii_.update_kinematics();
    meii_.rps_init_par_ref_.start(meii_.get_wrist_parallel_positions(), clock_.time());

    // enter the control loop
    while (!rps_init_ && !stop_) {

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
        rps_init_ = meii_.check_rps_init();

        // check for stop input
        stop_ = check_stop();

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
    else if (rps_init_) {
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

    // initialize global event variables
    target_reached_ = false;

    // initialize local state variables
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
    while (!target_reached_ && !stop_) {

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
        target_reached_ = meii_.check_goal_anat_pos(center_pos_, { 1, 1, 1, 1, 0 });

        // check for stop input
        stop_ = check_stop();

        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    // transition to next state from "GOT TO CENTER"
    if (stop_) {
        // stop if user provided input
        event(ST_STOP);
    }
    else if (target_reached_) {
        event(ST_HOLD_CENTER);
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

    // initialize global event variables
    hold_target_time_reached_ = false;

    // initialize local state variables
    double st_enter_time = clock_.time();
    double_vec command_torques(meii_.N_aj_, 0.0);

    // write to Unity
    viz_target_num_ = 0;
    viz_target_num_share_.write(viz_target_num_);

    // enter the control loop
    while (!hold_target_time_reached_ && !stop_) {

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
        hold_target_time_reached_ = check_wait_time_reached(hold_center_time_, st_enter_time, clock_.time());

        // check for stop input
        stop_ = check_stop();

        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    // transition to next state from "HOLD AT CENTER"
    if (stop_) {
        // stop if user provided input
        event(ST_STOP);
    }
    else if (hold_target_time_reached_) {
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
    force_mag_reached_ = false;

    // initialize local state variables
    double_vec command_torques(meii_.N_aj_, 0.0);
    double force_mag = 0.0;
    
    // initialize magnitude force checking algorithm
    force_mag_maintained_ = 0.0;
    force_mag_time_now_ = clock_.global_time();
    force_mag_time_last_ = clock_.global_time();

    // read from target sequence and write to Unity
    ++current_class_label_idx_;
    if (current_class_label_idx_ >= class_label_sequence_.size()) {
        end_of_target_sequence_ = true;
    }
    else {
        set_viz_target_num(class_label_sequence_[current_class_label_idx_]);
        viz_target_num_share_.write(viz_target_num_);
    }

    // enter the control loop
    while (!end_of_target_sequence_ && !force_mag_reached_ && !stop_) {

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
        filtered_emg_voltages_ = meii_.butter_hp_.filter(meii_.get_emg_voltages()); 
        emg_data_buffer_.push_back(filtered_emg_voltages_);
        emg_share_.write(emg_data_buffer_.at(0));

        // check force magnitude
        force_mag_reached_ = check_force_mag_reached(force_mag_goal_, force_mag);

        // write to unity
        force_mag_share_.write(force_mag);

        // write to daq
        daq_->write_all();

        // check for stop input
        stop_ = check_stop();

        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    // transition to next state from "PRESENT TARGET"
    if (stop_) {
        // stop if user provided input
        event(ST_STOP);
    }
    else if (end_of_target_sequence_) {
        if (condition_ == 0) {
            event(ST_TRAIN_CLASSIFIER);
        }
        else if (condition_ == 1 || condition_ == 2) {
            event(ST_FINISH);
        }
        else {
            util::print("ERROR: Invalid condition number while exiting state ST_PRESENT_TARGET.");
        }
    }
    else if (force_mag_reached_) {   
        
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

    // initialize global event variables
    emg_data_processed_ = true; // default to true; only write false if error encountered

    // extract features from EMG data
    feature_vec_ = feature_extract(emg_data_buffer_);

    // check for successful data processing
    bool are_nans_ = false;
    for (int i = 0; i < feature_vec_.size(); ++i) {
        if (std::isnan(feature_vec_[i])) {
            are_nans_ = true;
            feature_vec_[i] = 1.0;
            //emg_data_processed_ = false;
        }
    }
    if (are_nans_) {
        util::print("WARNING: NaN feature(s) detected. Replacing with 1.0 .");
    }

    // copy features into an array
    std::copy_n(feature_vec_.begin(), feature_array_.size(), feature_array_.begin());

    // store feature array in training data set
    emg_training_data_.push_back(feature_array_);

    // log data when in training mode
    if (is_training()) {
        log_row();
    }

    // transition to next state from "PROCESS EMG DATA"
    if (stop_) {
        // stop if user provided input
        event(ST_STOP);
    }
    else if (emg_data_processed_) {    
        if (is_training()) {
            event(ST_HOLD_CENTER);
        }
        else {
            event(ST_CLASSIFY);
        }
    }
    else if (!emg_data_processed_) {
        util::print("WARNING: EMG Data unsuccessfully processed. Goint to ST_STOP.");
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

    // open LDA script in Python
    system("start EMG_FS_LDA.py &");

    // create vector of EMG features to send to Python
    double_vec emg_training_data_vec;
    emg_training_data_vec.reserve(N_train_ * meii_.N_emg_ * num_features_);
    for (int i = 0; i < N_train_; ++i) {
        for (int j = 0; j < meii_.N_emg_ * num_features_; ++j) {
            emg_training_data_vec.push_back(emg_training_data_[i][j]);
        }
    }

    // calculate size of training data to be sent to python
    std::array<int, 2> training_data_size = { N_train_, meii_.N_emg_ * num_features_ };

    // copy training labels into an array
    std::vector<int> training_labels(N_train_);
    std::copy_n(class_label_sequence_.begin(), N_train_, training_labels.begin());

    // write training data to python
    trng_size_.write(training_data_size);
    trng_share_.write(emg_training_data_vec);
    label_share_.write(training_labels);

    // wait to receive trained classifier from python
    while ((lda_training_complete_ == 0) && !stop_) {

        // check for flag
        lda_training_flag_.read(lda_training_complete_);

        // check for stop input
        stop_ = check_stop();

        // wait for the next clock cycle
        clock_.hybrid_wait();
    }


    // read in csv's generated by python
    read_csv("LDA_coeffs.csv", lda_classifier_);
    read_csv("intercept.csv", lda_intercept_);
    read_csv("feat_sel.csv", sel_feats_);
    
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
    for (int i = 0; i < lda_cols -1; ++i) {
        classify_features[i] = emg_training_data_[0][sel_feats_[0][i]];
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
    log_row();

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
            event(ST_HOLD_CENTER);
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
    target_reached_ = false;

    // initialize local state variables
    double_vec command_torques(meii_.N_aj_, 0.0);

    // set new reference position
    double_vec target_pos = get_target_position(class_label_sequence_[current_class_label_idx_]);
    meii_.anat_ref_.set_ref(target_pos, clock_.time());

    // write to Unity
    set_viz_target_num(class_label_sequence_[current_class_label_idx_]);
    viz_target_num_share_.write(viz_target_num_); // do we want to show the correct target or the target they're moving to???

    // enter the control loop
    while (!target_reached_ && !stop_) {

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
        target_reached_ = meii_.check_goal_anat_pos(target_pos, { 1, 1, 1, 1, 0 });

        // check for stop input
        stop_ = check_stop();

        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    // transition to next state
    if (stop_) {
        // stop if user provided input
        event(ST_STOP);
    }
    else if (target_reached_) {
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
    hold_target_time_reached_ = false;

    // initialize local state variables
    double st_enter_time = clock_.time();
    double_vec command_torques(meii_.N_aj_, 0.0);


    // enter the control loop
    while (!hold_target_time_reached_ && !stop_) {

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
        hold_target_time_reached_ = check_wait_time_reached(hold_target_time_, st_enter_time, clock_.time());

        // check for stop input
        stop_ = check_stop();

        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    // transition to next state
    if (stop_) {
        // stop if user provided input
        event(ST_STOP);
    }
    else if (target_reached_) {
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
    util::print("Finish Experiment");

    // intialize global event variables
    menu_selected_ = false;

    // disable robot
    if (meii_.is_enabled()) {
        meii_.disable();
    }

    // log data
    std::vector<std::string> dofs = {"EFE", "FPS", "WFE", "WRU", "ELFM", "WMLT"};
    std::vector<std::string> conditions = { "trng", "blind", "full" };

    std::string filename;
    if (subject_number_ < 10) {
        filename = "S0" + std::to_string(subject_number_) + "_" + dofs[dof_] + "_" + conditions[condition_];
    }
    else {
        filename = "S" + std::to_string(subject_number_) + "_" + dofs[dof_] + "_" + conditions[condition_];
    }

    log_.save_and_clear_data("emg_exp_data_"+filename, directory_, true);
        if (condition_ == 0) {
            lda_log_.save_and_clear_data("LDA_Coeffs_" + filename, directory_, true);
            feature_log_.save_and_clear_data("selected_features_" + filename, directory_, true);
        }

     log_ = util::DataLog("emg_exp_log", false);
     lda_log_ = util::DataLog("lda_coeff_log", false);
     feature_log_ = util::DataLog("feat_sel_log", false);



    // wait for user input
    util::print("Press 'm' in Unity to return to the GUI main menu or press 'CTRL + C' to stop the experiment");
    while (!menu_selected_ && !stop_) {

        // read from Unity
        scene_num_share_.read(SCENE_NUM_);

        // check if menu selected in Unity
        if (SCENE_NUM_ == 0) {
            menu_selected_ = true;
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
    else if (menu_selected_) {
        event(ST_WAIT_FOR_GUI);
    }

}

//-----------------------------------------------------------------------------
// "STOP" STATE FUNCTION
//-----------------------------------------------------------------------------

void EmgRTControl::sf_stop(const util::NoEventData* data) {
    std::cout << "State Stop " << std::endl;

    if (meii_.is_enabled()) {
        meii_.disable();
    }
    if (daq_->is_enabled()) {
        daq_->disable();
    }
}

//-----------------------------------------------------------------------------
// UTILITY FUNCTIONS
//-----------------------------------------------------------------------------

void EmgRTControl::log_row() {
    std::vector<double> row;
    row.push_back(clock_.time());
    row.push_back(dof_);
    row.push_back(condition_);
    for (auto i = 0; i < meii_.N_emg_ * num_features_; ++i) {
        row.push_back(feature_vec_[i]);
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
    log_.add_row(row);
}

void EmgRTControl::wait_for_input() {
    util::Input::wait_for_key(util::Input::Key::Space);
}

bool EmgRTControl::check_stop() {
    return util::Input::is_key_pressed(util::Input::Escape) || (util::Input::is_key_pressed(util::Input::LControl) && util::Input::is_key_pressed(util::Input::C));
}


void EmgRTControl::set_experiment_conditions(int scene_num) {
    dof_ = (scene_num - 2) / 3;
    condition_ = (scene_num - 2) % 3;
    
}

void EmgRTControl::set_viz_target_num(int class_label) {
    viz_target_num_ = class_label;
}

double_vec EmgRTControl::get_target_position(int class_label) {
    if (is_single_dof()) {
        return single_dof_targets_[hand_num_][dof_][class_label - 1];
    }
    else {
        return multi_dof_targets_[hand_num_][dof_ - 4][class_label - 1];
    }
}

bool EmgRTControl::is_single_dof() {
    return SCENE_NUM_ < 14;
}

bool EmgRTControl::is_training() {
    return condition_ == 0;
}

bool EmgRTControl::is_testing() {
    return condition_ == 1 || condition_ == 2;
}

bool EmgRTControl::is_blind() {
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
            task_force = (-1.0) * force_mag_goal_ * (commanded_torques[dof] + gravity_offsets_[dof])  / force_dof_scale_[dof] * target_dir[dof][target_num - 1];
        }
        else if (dof == 4) {
            task_force = (-1.0) * force_mag_goal_ * ((commanded_torques[0] + gravity_offsets_[0]) / force_dof_scale_[0] * target_dir[dof][target_num - 1] + (commanded_torques[1] + gravity_offsets_[1]) / force_dof_scale_[1] * target_dir[dof + 1][target_num - 1]) / 2.0;
        }
        else if (dof == 5) {
            task_force = (-1.0) * force_mag_goal_ * ((commanded_torques[2] + gravity_offsets_[2]) / force_dof_scale_[2] * target_dir[dof][target_num - 1] + (commanded_torques[3] + gravity_offsets_[3]) / force_dof_scale_[3] * target_dir[dof + 1][target_num - 1]) / 2.0;
        }
        return std::max(0.0, task_force);
        break;
    case 1:
        if (dof < 4) {
            task_force = force_mag_goal_ * (commanded_torques[dof] + gravity_offsets_[dof]) / force_dof_scale_[dof];
        }
        else if (dof == 4) {
            task_force = force_mag_goal_ * (std::abs((commanded_torques[0] + gravity_offsets_[0])) / force_dof_scale_[0] + std::abs((commanded_torques[1] + gravity_offsets_[1])) / force_dof_scale_[1]) / 2.0;
        }
        else if (dof == 5) {
            task_force = force_mag_goal_ * (std::abs((commanded_torques[2] + gravity_offsets_[2])) / force_dof_scale_[2] + std::abs((commanded_torques[3] + gravity_offsets_[3])) / force_dof_scale_[3]) / 2.0;
        }
        return std::abs(task_force);
        break;
    case 2:
        task_force = force_mag_goal_ * (std::abs((commanded_torques[0] + gravity_offsets_[0])) / force_dof_scale_[0] + std::abs((commanded_torques[1] + gravity_offsets_[1])) / force_dof_scale_[1] + std::abs((commanded_torques[2] + gravity_offsets_[2])) / force_dof_scale_[2] + std::abs((commanded_torques[3] + gravity_offsets_[3])) / force_dof_scale_[3]) / 4.0;
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

void EmgRTControl::read_csv(std::string filename, std::vector<std::vector<double>>& output) {
    std::ifstream input(filename);
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
    }
    else {
        util::print("ERROR: File not found.");
    }
}

void EmgRTControl::read_csv(std::string filename, std::vector<std::vector<int>>& output) {
    std::ifstream input(filename);
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
    }
    else {
        util::print("ERROR: File not found.");
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
    util::print(nwl_vec);
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
    util::print(nwl_vec);
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
