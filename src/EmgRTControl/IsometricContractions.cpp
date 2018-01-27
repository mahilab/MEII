#include "EmgRTControl/IsometricContractions.hpp"
#include "MEL/Utility/Windows/Keyboard.hpp"
#include "MEL/Core/Motor.hpp"
#include <iostream>
#include <fstream>
#include <string>

using namespace mel;

IsometricContractions::IsometricContractions(Clock& clock, Timer timer, Daq& daq, Input<voltage>& analog_input, Output<voltage>& analog_output, Watchdog& watchdog, MahiExoIIEmg& meii) :
    StateMachine(12),
    clock_(clock),
    timer_(timer),
    daq_(daq),
    analog_input_(analog_input),
    analog_output_(analog_output),
    watchdog_(watchdog),
    meii_(meii),
    scene_num_share_("scene_num"),
    viz_target_num_share_("target"),
    force_mag_share_("force_mag"),
    hand_select_("hand"),
    trng_size_("trng_size"),
    trng_share_("trng_share", 16384),
    label_share_("label_share"),
    lda_coeff_("LDA_coeff", 2048),
    trng_size2_("trng_size2"),
    feat_id_("feat_id"),
    lda_training_flag_("lda_training_flag"),
    pos_share_("pos_share"),
    vel_share_("vel_share"),
    emg_share_("emg_share"),
    torque_share_("torque_share")
{
}


//-----------------------------------------------------------------------------
// "WAIT FOR GUI" STATE FUNCTION
//-----------------------------------------------------------------------------
void IsometricContractions::sf_wait_for_gui(const NoEventData* data) {
    print("Waiting for Gui Input");

    // initialize global event variables
    scene_selected_ = false;

    // launch game
    game.launch();

    // enter the control loop
    while (!scene_selected_ && !stop_) {

        // read from Unity
        scene_num_share_.read_data(SCENE_NUM_);

        // check if scene selected in Unity
        if (SCENE_NUM_ > 0) {
            scene_selected_ = true;
        }

        // check for stop input
        stop_ = check_stop();

        // wait for the next clock cycle
        timer_.wait();
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
        print("ERROR: State transition undefined. Going to ST_STOP.");
        event(ST_STOP);
    }
}


//-----------------------------------------------------------------------------
// "INITIALIZATION" STATE FUNCTION
//-----------------------------------------------------------------------------
void IsometricContractions::sf_init(const NoEventData* data) {

    // set experiment condition variables dof_ and condition_ based on scene number
    set_experiment_conditions(SCENE_NUM_);

    std::vector<std::string> str_conditions = { "Training", "Blind Testing", "Full Testing" };
    std::vector<std::string> str_dofs = { "Elbow F/E Single-DoF", "Forearm P/S Single-Dof", "Wrist F/E Single-DoF", "Wrist R/U Single-DoF", "Elbow F/E & Forearm P/S Multi-DoF", "Wrist F/E & Wrist R/U Multi-DoF" };
    print("Initializing for " + str_conditions[condition_] + " of " + str_dofs[dof_]);

    // initialize global event variables
    menu_selected_ = false;
    scene_selected_ = true;
    init_backdrive_time_reached_ = false;
    rps_init_ = false;
    target_reached_ = false;
    hold_target_time_reached_ = false;
    target_input_ = false;
    force_mag_reached_ = false;
    emg_data_processed_ = false;
    end_of_target_sequence_ = false;
    lda_training_complete_ = 0;

    // reset global experiment variables
    current_class_label_ = 0;
    viz_target_num_ = 0;
    pred_class_label_ = 0;
    lda_classifier_.clear();
    lda_intercept_.clear();
    emg_training_data_.clear();

    // reset robot
    meii_.disable();

    
    if (is_single_dof()) {
        num_class_ = 2;
    }
    else {
        num_class_ = 4;
    }


    // read in LDA classifier
    if (is_testing()) {
        read_csv(program_directory_ + "LDA_coeffs.csv", lda_classifier_);
        read_csv(program_directory_ + "intercept.csv", lda_intercept_);
        read_csv(program_directory_ + "feat_sel.csv", sel_feats_);

        size_t lda_rows = lda_classifier_.size();
        size_t lda_cols = lda_classifier_[0].size();

        // eigenization
        lda_class_eig_ = Eigen::MatrixXd::Zero(lda_rows, lda_cols);
        lda_inter_eig_ = Eigen::VectorXd::Zero(lda_rows);
        for (int i = 0; i < lda_rows; ++i) {
            lda_class_eig_.row(i) = stdvec_to_eigvec(lda_classifier_[i]);
        }
        lda_inter_eig_ = stdvec_to_eigvec(lda_intercept_[0]);
    }

    // enable MEII EMG DAQ
    daq_.enable();
    if (!daq_.is_enabled()) {
        event(ST_STOP);
        return;
    }

    // check DAQ behavior for safety
    analog_input_.update();
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
    print("\nRunning EMG Real-Time Control ... ");

    // write to Unity
    hand_select_.write_message(hand_def_);

    // start the watchdog
    watchdog_.start();

    // start the clock
    timer_.restart();

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
void IsometricContractions::sf_backdrive(const NoEventData* data) {
    print("Robot Backdrivable");

    // initialize global event variables
    init_backdrive_time_reached_ = false;

    // initialize local state variables
    double st_enter_time = timer_.get_elapsed_time;
    const std::vector<double> command_torques(meii_.N_aj_, 0.0);

    // enter the control loop
    while (!init_backdrive_time_reached_ && !stop_) {

        // read and reload DAQs
        watchdog_.kick();
        analog_input_.update();

        // update robot kinematics
        meii_.update_kinematics();

        // write kinematics to MelScope
        pos_share_.write_data(meii_.get_anatomical_joint_positions());
        vel_share_.write_data(meii_.get_anatomical_joint_velocities());

        // check joint limits
        if (meii_.check_all_joint_limits()) {
            stop_ = true;
            break;
        }

        // set zero torques
        meii_.set_joint_torques(command_torques);

        // write to daq
        analog_output_.update();

        // check for init transparent time reached
        init_backdrive_time_reached_ = check_wait_time_reached(init_backdrive_time_, st_enter_time, clock_.time());

        // check for stop input
        stop_ = check_stop();

        // wait for the next clock cycle
        timer_.wait();
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
        print("ERROR: State transition undefined. Going to ST_STOP.");
        event(ST_STOP);
    }
}


//-----------------------------------------------------------------------------
// "INITIALIZE RPS MECHANISM" STATE FUNCTION
//-----------------------------------------------------------------------------
void IsometricContractions::sf_init_rps(const NoEventData* data) {
    print("Initialize RPS Mechanism");

    // initialize global event variables
    rps_init_ = false;

    // initialize rps initialization position controller
    meii_.set_rps_control_mode(0);
    meii_.update_kinematics();
    meii_.rps_init_par_ref_.start(meii_.get_wrist_parallel_positions(), clock_.time());

    // enter the control loop
    while (!rps_init_ && !stop_) {

        // read and reload DAQs
        watchdog_.kick();
        analog_input_.update();

        // update robot kinematics
        meii_.update_kinematics();

        // check joint limits
        if (meii_.check_all_joint_limits()) {
            stop_ = true;
            break;
        }

        // write kinematics to MelScope
        pos_share_.write_data(meii_.get_anatomical_joint_positions());
        vel_share_.write_data(meii_.get_anatomical_joint_velocities());

        // set zero torque for elbow and forearm joints (joints 0 and 1)
        meii_.joints_[0]->set_torque(0.0);
        meii_.joints_[1]->set_torque(0.0);

        // run rps position control
        meii_.set_rps_pos_ctrl_torques(meii_.rps_init_par_ref_, clock_.time());

        // write to daq
        analog_output_.update();

        // check for rps mechanism in intialization position
        rps_init_ = meii_.check_rps_init();

        // check for stop input
        stop_ = check_stop();

        // wait for the next clock cycle
        timer_.wait();
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
        print("ERROR: State transition undefined. Going to ST_STOP.");
        event(ST_STOP);
    }
}



//-----------------------------------------------------------------------------
// "GO TO CENTER" STATE FUNCTION
//-----------------------------------------------------------------------------
void IsometricContractions::sf_to_center(const NoEventData* data) {
    print("Go to Center");

    // initialize global event variables
    target_reached_ = false;

    // initialize local state variables
    std::vector<double> command_torques(meii_.N_aj_, 0.0);

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
    viz_target_num_share_.write_data(viz_target_num_);

    // enter the control loop
    while (!target_reached_ && !stop_) {

        // read and reload DAQs
        watchdog_.kick();
        analog_input_.update();

        // update robot kinematics
        meii_.update_kinematics();

        // write kinematics to MelScope
        pos_share_.write_data(meii_.get_anatomical_joint_positions());
        vel_share_.write_data(meii_.get_anatomical_joint_velocities());

        // check joint limits
        if (meii_.check_all_joint_limits()) {
            stop_ = true;
            break;
        }

        // run position control
        command_torques = meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, clock_.time());

        // write motor commands to MelScope
        torque_share_.write_data(command_torques);

        // write to daq
        analog_output_.update();

        // check for target reached
        target_reached_ = meii_.check_goal_anat_pos(center_pos_, { 1, 1, 1, 1, 0 });

        // check for stop input
        stop_ = check_stop();

        // wait for the next clock cycle
        timer_.wait();
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
        print("ERROR: State transition undefined. Going to ST_STOP.");
        event(ST_STOP);
    }
}


//-----------------------------------------------------------------------------
// "HOLD AT CENTER" STATE FUNCTION
//-----------------------------------------------------------------------------
void IsometricContractions::sf_hold_center(const NoEventData* data) {
    print("Hold at Center");

    // initialize global event variables
    target_input_ = false;
    end_of_target_sequence_ = false;
    current_class_label_ = 0;

    // initialize local state variables
    double st_enter_time = clock_.time();
    std::vector<double> command_torques(meii_.N_aj_, 0.0);

    // write to Unity
    viz_target_num_ = 0;
    viz_target_num_share_.write_data(viz_target_num_);

    // prompt user for input
    print("Enter the next class label to be presented, or press escape to finish current session.");
    

    // enter the control loop
    while (!target_input_ && !end_of_target_sequence_ && !stop_) {

        // read and reload DAQs
        watchdog_.kick();
        analog_input_.update();

        // update robot kinematics
        meii_.update_kinematics();

        // write kinematics to MelScope
        pos_share_.write_data(meii_.get_anatomical_joint_positions());
        vel_share_.write_data(meii_.get_anatomical_joint_velocities());

        // check joint limits
        if (meii_.check_all_joint_limits()) {
            stop_ = true;
            break;
        }

        // run position control
        command_torques = meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, clock_.time());

        // write motor commands to MelScope
        torque_share_.write_data(command_torques);

        // write to daq
        analog_output_.update();

        // check for user input
        current_class_label_ = check_for_user_input_target();
        if (current_class_label_ < 0) {
            end_of_target_sequence_ = true;
        }
        else if (current_class_label_ > 0) {
            class_label_sequence_.push_back(current_class_label_);
            target_input_ = true;
        }

        // check for stop input
        stop_ = check_stop();

        // wait for the next clock cycle
        timer_.wait();
    }

    // transition to next state from "HOLD AT CENTER"
    if (stop_) {
        // stop if user provided input
        event(ST_STOP);
    }
    else if (target_input_) {
        event(ST_PRESENT_TARGET);
    }
    else if (end_of_target_sequence_) {
        if (is_training()) {
            event(ST_TRAIN_CLASSIFIER);
        }
        else {
            event(ST_FINISH);
        }
    }
    else {
        print("ERROR: State transition undefined. Going to ST_STOP.");
        event(ST_STOP);
    }
}



//-----------------------------------------------------------------------------
// "PRESENT TARGET" STATE FUNCTION
//-----------------------------------------------------------------------------
void IsometricContractions::sf_present_target(const NoEventData* data) {
    print("Present Target");

    // initialize global event variables
    force_mag_reached_ = false;

    // initialize local state variables
    std::vector<double> command_torques(meii_.N_aj_, 0.0);
    double force_mag = 0.0;

    // initialize magnitude force checking algorithm
    force_mag_maintained_ = 0.0;
    force_mag_time_now_ = clock_.global_time();
    force_mag_time_last_ = clock_.global_time();

    
    set_viz_target_num(current_class_label_);
    viz_target_num_share_.write_data(viz_target_num_);
    

    // enter the control loop
    while (!force_mag_reached_ && !stop_) {

        // read and reload DAQs
        watchdog_.kick();
        analog_input_.update();

        // update robot kinematics
        meii_.update_kinematics();

        // write kinematics to MelScope
        pos_share_.write_data(meii_.get_anatomical_joint_positions());
        vel_share_.write_data(meii_.get_anatomical_joint_velocities());

        // check joint limits
        if (meii_.check_all_joint_limits()) {
            stop_ = true;
            break;
        }

        // run position control
        command_torques = meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, clock_.time());

        // write motor commands to MelScope
        torque_share_.write_data(command_torques);

        // measure interaction force for specified dof(s)
        force_mag = measure_task_force(command_torques, current_class_label_, dof_, condition_);

        // get measured emg voltages
        meii_.butter_hp_.filter(meii_.get_emg_voltages(), filtered_emg_voltages_);
        emg_data_buffer_.push_back(filtered_emg_voltages_);
        emg_share_.write_data(emg_data_buffer_.at(0));

        // check force magnitude
        force_mag_reached_ = check_force_mag_reached(force_mag_goal_, force_mag);

        // write to unity
        force_mag_share_.write_data(force_mag);

        // write to daq
        analog_output_.update();

        // check for stop input
        stop_ = check_stop();

        // wait for the next clock cycle
        timer_.wait();
    }

    // transition to next state from "PRESENT TARGET"
    if (stop_) {
        // stop if user provided input
        event(ST_STOP);
    }
    else if (force_mag_reached_) {
        event(ST_PROCESS_EMG);
    }
    else {
        print("ERROR: State transition undefined. Going to ST_STOP.");
        event(ST_STOP);
    }
}


//-----------------------------------------------------------------------------
// "PROCESS EMG DATA" STATE FUNCTION
//-----------------------------------------------------------------------------
void IsometricContractions::sf_process_emg(const NoEventData* data) {
    print("Process EMG Data");

    // initialize global event variables
    emg_data_processed_ = true; // default to true; only write false if error encountered

                                // extract features from EMG data
    feature_vec_ = feature_extract(emg_data_buffer_);

    // check for successful data processing
    bool are_nan_features_ = false;
    for (int i = 0; i < feature_vec_.size(); ++i) {
        if (std::isnan(feature_vec_[i])) {
            are_nan_features_ = true;
            feature_vec_[i] = 1.0;
            //emg_data_processed_ = false;
        }
    }
    if (are_nan_features_) {
        print("WARNING: NaN feature(s) detected. Replacing with 1.0 .");
    }

    // copy features into an array
    std::copy_n(feature_vec_.begin(), feature_array_.size(), feature_array_.begin());

    // store feature array in training data set
    emg_training_data_.push_back(feature_array_);

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
        print("WARNING: EMG Data unsuccessfully processed. Goint to ST_STOP.");
        event(ST_STOP);
    }
    else {
        print("ERROR: State transition undefined. Going to ST_STOP.");
        event(ST_STOP);
    }
}



//-----------------------------------------------------------------------------
// "TRAIN CLASSIFIER" STATE FUNCTION
//-----------------------------------------------------------------------------
void IsometricContractions::sf_train_classifier(const NoEventData* data) {
    print("Training Classifier");

    // disable robot
    meii_.disable();
    
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
    N_train_ = emg_training_data_.size();
    std::vector<double> emg_training_data_vec;
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
    trng_size_.write_data(training_data_size);
    trng_share_.write_data(emg_training_data_vec);
    label_share_.write_data(training_labels);

    // wait to receive trained classifier from python
    while ((lda_training_complete_ == 0) && !stop_) {

        // check for flag
        lda_training_flag_.read(lda_training_complete_);

        // check for stop input
        stop_ = check_stop();

        // wait for the next clock cycle
        timer_.wait();
    }


    // read in csv's generated by python
    read_csv(program_directory_ + "LDA_coeffs.csv", lda_classifier_);
    read_csv(program_directory_ + "intercept.csv", lda_intercept_);
    read_csv(program_directory_ + "feat_sel.csv", sel_feats_);

    size_t lda_rows = lda_classifier_.size();
    size_t lda_cols = lda_classifier_[0].size();


    print(sel_feats_[0]);

    // eigenization
    lda_class_eig_ = Eigen::MatrixXd::Zero(lda_rows, lda_cols);
    lda_inter_eig_ = Eigen::VectorXd::Zero(lda_rows);
    for (int i = 0; i < lda_rows; ++i) {
        lda_class_eig_.row(i) = stdvec_to_eigvec(lda_classifier_[i]);
    }
    lda_inter_eig_ = stdvec_to_eigvec(lda_intercept_[0]);

    print(lda_class_eig_);
    print("\n");
    print(lda_inter_eig_);

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
void IsometricContractions::sf_classify(const NoEventData* data) {

    print("Classifying EMG Activation");

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
    for (int i = 0; i < lda_cols - 1; ++i) {
        classify_features[i] = emg_training_data_[0][sel_feats_[0][i]];
    }

    print(classify_features);

    // print(lda_class_eig_);



    //Convert vector of selected features to Eigen
    //Eigen::VectorXd classify_features_eig = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(classify_features.data(), classify_features.size());
    Eigen::VectorXd classify_features_eig = stdvec_to_eigvec(classify_features);

    //print(classify_features_eig);

    //multiply the LDA coefficients and EMG features
    lda_dist_eig_ = lda_class_eig_ * classify_features_eig + lda_inter_eig_;

    print(lda_dist_eig_);

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
        std::vector<double> lda_prob = eigvec_to_stdvec(lda_dist_eig_);
        classifier_result_ = std::distance(lda_prob.begin(), std::max_element(lda_prob.begin(), lda_prob.end()));
        classifier_result_ += 1;
    }

    print(classifier_result_);

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
        event(ST_HOLD_CENTER);
    }

}



//-----------------------------------------------------------------------------
// "FINISH EXPERIMENT" STATE FUNCTION
//-----------------------------------------------------------------------------
void IsometricContractions::sf_finish(const NoEventData* data) {
    print("Finish Experiment");

    // intialize global event variables
    menu_selected_ = false;

    // disable robot
    if (meii_.is_enabled()) {
        meii_.disable();
    }

    // log data
    std::vector<std::string> dofs = { "EFE", "FPS", "WFE", "WRU", "ELFM", "WMLT" };
    std::vector<std::string> conditions = { "trng", "blind", "full" };


    // wait for user input
    print("Press 'm' in Unity to return to the GUI main menu or press 'CTRL + C' to stop the experiment");
    while (!menu_selected_ && !stop_) {

        // read from Unity
        scene_num_share_.read_data(SCENE_NUM_);

        // check if menu selected in Unity
        if (SCENE_NUM_ == 0) {
            menu_selected_ = true;
        }

        // check for stop input
        stop_ = check_stop();

        // wait for the next clock cycle
        timer_.wait();
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

void IsometricContractions::sf_stop(const NoEventData* data) {
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

int IsometricContractions::check_for_user_input_target() {
    Keyboard::Key key;
    if (is_single_dof()) {
        key = Keyboard::are_any_keys_pressed({ Keyboard::Key::Num1, Keyboard::Key::Num2, Keyboard::Key::Escape }, true);
        switch (key) {
        case Keyboard::Key::Num1:
            return 1;
            break;
        case Keyboard::Key::Num2:
            return 2;
            break;
        case Keyboard::Key::Escape:
            return -1;
            break;
        default: return 0;
        }
    }
    else {
        key = Keyboard::are_any_keys_pressed({ Keyboard::Key::Num1, Keyboard::Key::Num2, Keyboard::Key::Num3, Keyboard::Key::Num4, Keyboard::Key::Escape }, true);
        switch (key) {
        case Keyboard::Key::Num1:
            return 1;
            break;
        case Keyboard::Key::Num2:
            return 2;
            break;
        case Keyboard::Key::Num3:
            return 3;
            break;
        case Keyboard::Key::Num4:
            return 4;
            break;
        case Keyboard::Key::Escape:
            return -1;
            break;
        default: return 0;
        }
    }
}

bool IsometricContractions::check_stop() {
    return (Keyboard::is_key_pressed(Keyboard::LControl) && Keyboard::is_key_pressed(Keyboard::C));
}

void IsometricContractions::set_experiment_conditions(int scene_num) {
    dof_ = (scene_num - 2) / 3;
    condition_ = (scene_num - 2) % 3;
}

void IsometricContractions::set_viz_target_num(int class_label) {
    viz_target_num_ = class_label;
}

bool IsometricContractions::is_single_dof() {
    return SCENE_NUM_ < 14;
}

bool IsometricContractions::is_training() {
    return condition_ == 0;
}

bool IsometricContractions::is_testing() {
    return condition_ == 1 || condition_ == 2;
}

bool IsometricContractions::is_blind() {
    return condition_ == 1;
}

bool IsometricContractions::check_wait_time_reached(double wait_time, double init_time, double current_time) const {
    return (current_time - init_time) > wait_time;
}

double IsometricContractions::measure_task_force(std::vector<double> commanded_torques, int target_num, int dof, int condition) const {

    double task_force;
    std::vector<char> target_dir;

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


bool IsometricContractions::check_force_mag_reached(double force_mag_goal, double force_mag) {
    force_mag_time_now_ = clock_.global_time();
    force_mag_maintained_ = std::abs(force_mag_maintained_ + std::copysign(1.0, force_mag_tol_ - std::abs(force_mag_goal - force_mag)) * (force_mag_time_now_ - force_mag_time_last_));
    force_mag_time_last_ = force_mag_time_now_;
    return force_mag_maintained_ > force_mag_dwell_time_;
}

void IsometricContractions::read_csv(std::string filename, std::vector<std::vector<double>>& output) {
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
        print("ERROR: File not found.");
    }
}

void IsometricContractions::read_csv(std::string filename, std::vector<std::vector<int>>& output) {
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
        print("ERROR: File not found.");
    }
}

std::vector<double> IsometricContractions::feature_extract(MahiExoIIEmg::EmgDataBuffer& emg_data_buffer) {

    std::vector<double> feature_vec;
    feature_vec.reserve(meii_.N_emg_ * num_features_);
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

    // extract unnormalized features
    for (int i = 0; i < meii_.N_emg_; ++i) {
        nrms_vec[i] = rms_feature_extract(emg_data_buffer.data_buffer_[i]);
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
    print(nzc_vec);
    print("");

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
    print(nzc_vec);
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

double IsometricContractions::rms_feature_extract(boost::circular_buffer<double> emg_channel_buffer) {
    double sum_squares = 0;
    for (int i = 0; i < emg_channel_buffer.size(); ++i) {
        sum_squares += std::pow(emg_channel_buffer[i], 2);
    }
    return std::sqrt(sum_squares / emg_channel_buffer.size());
}

double IsometricContractions::mav_feature_extract(boost::circular_buffer<double> emg_channel_buffer) {
    double sum_abs = 0;
    for (int i = 0; i < emg_channel_buffer.size(); ++i) {
        sum_abs += std::abs(emg_channel_buffer[i]);
    }
    return sum_abs / emg_channel_buffer.size();
}

double IsometricContractions::wl_feature_extract(boost::circular_buffer<double> emg_channel_buffer) {
    double sum_abs_diff = 0.0;
    for (int i = 0; i < emg_channel_buffer.size() - 1; ++i) {
        sum_abs_diff += std::abs(emg_channel_buffer[i + 1] - emg_channel_buffer[i]);
    }
    return sum_abs_diff;
}

double IsometricContractions::zc_feature_extract(boost::circular_buffer<double> emg_channel_buffer) {
    double sum_abs_diff_sign = 0;
    for (int i = 0; i < emg_channel_buffer.size() - 1; ++i) {
        sum_abs_diff_sign += std::abs(std::copysign(1.0, emg_channel_buffer[i + 1]) - std::copysign(1.0, emg_channel_buffer[i]));
    }
    return sum_abs_diff_sign / 2;
}

double IsometricContractions::ssc_feature_extract(boost::circular_buffer<double> emg_channel_buffer) {
    double sum_abs_diff_sign_diff = 0;
    for (int i = 0; i < emg_channel_buffer.size() - 2; ++i) {
        sum_abs_diff_sign_diff += std::abs(std::copysign(1.0, (emg_channel_buffer[i + 2] - emg_channel_buffer[i + 1])) - std::copysign(1.0, (emg_channel_buffer[i + 1] - emg_channel_buffer[i])));
    }
    return sum_abs_diff_sign_diff / 2;
}

void IsometricContractions::ar4_feature_extract(std::vector<double>& coeffs, const std::vector<double>& emg_channel_buffer) {

    // initialize
    size_t N = emg_channel_buffer.size();
    size_t m = coeffs.size();
    std::vector<double> A_k(m + 1, 0.0);
    A_k[0] = 1.0;
    std::vector<double> f(emg_channel_buffer);
    std::vector<double> b(emg_channel_buffer);
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
