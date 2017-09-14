#include "EmgRTControl.h"
#include "Input.h"
#include "Motor.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace mel;

EmgRTControl::EmgRTControl(util::Clock& clock, core::Daq* daq, exo::MahiExoIIEmg& meii) :
    StateMachine(11),
    clock_(clock),
    daq_(daq),
    meii_(meii)
{
}

void EmgRTControl::wait_for_input() {
    util::Input::wait_for_key_press(util::Input::Key::Space);
}

bool EmgRTControl::check_stop() {
    return util::Input::is_key_pressed(util::Input::Escape) || (util::Input::is_key_pressed(util::Input::LControl) && util::Input::is_key_pressed(util::Input::C));
}

//-----------------------------------------------------------------------------
// "WAIT FOR GUI" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRTControl::sf_wait_for_gui(const util::NoEventData* data) {
    util::print("Waiting for Gui Input");

    // initialize event variables
    st_enter_time_ = clock_.time();

    // launch game
    game.launch();

    // enter the control loop
    while (!scene_selected_ && !stop_) {

        // read from Unity
        scene_num_share_.read(scene_num_);

        // check if scene selected in Unity
        if (scene_num_ > 0) {
            scene_selected_ = true;
        }

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
    else if (scene_selected_) {
        set_experiment_conditions(scene_num_);
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

    // read in target sequence from file
    std::ifstream input("target_sequences.txt");
    if (input.is_open()) {
        while(!input.eof()) {
            std::string number;
            int data;
            std::getline(input, number);
            data = std::atoi(number.c_str());
            target_sequence_.push_back(data);
        }
    }

    // confirm start of experiment
    util::print("\nRunning EMG Real-Time Control ... ");

    // write to Unity
    target_num_ = 0;
    target_num_share_.write(target_num_);

    // start the watchdog
    daq_->start_watchdog(0.1);

    // start the clock
    clock_.start();

    // transition to next state
    if (stop_) {
        event(ST_STOP);
    }
    else {
        event(ST_TRANSPARENT);
    }
}


//-----------------------------------------------------------------------------
// "TRANSPARENT" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRTControl::sf_transparent(const util::NoEventData* data) {
    util::print("Robot Transparent");

    // initialize event variables
    st_enter_time_ = clock_.time();

    // initialize temporary variables for control
    const double_vec commanded_torques(meii_.N_aj_, 0.0);

    // enter the control loop
    while (!init_transparent_time_reached_ && !stop_) {

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
        meii_.set_anatomical_joint_torques(commanded_torques);

        // write to daq
        daq_->write_all();

        // check for init transparent time reached
        init_transparent_time_reached_ = check_wait_time_reached(init_transparent_time_, st_enter_time_, clock_.time());

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
    else if (init_transparent_time_reached_) {
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

    // initialize event variables
    st_enter_time_ = clock_.time();

    // initialize temporary variables
    meii_.update_kinematics();
    double_vec rps_start_pos = meii_.get_wrist_parallel_positions();
    double_vec rps_ref_pos = rps_start_pos;
    double_vec rps_pd_torques(3, 0.0);
    double_vec commanded_torques(meii_.N_aj_, 0.0);

    // write to Unity
    target_num_ = 0;
    target_num_share_.write(target_num_);

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
        commanded_torques[0] = 0.0;
        commanded_torques[1] = 0.0;

        // compute wrist pd torques
        for (auto i = 0; i < 3; ++i) {
            rps_ref_pos[i] = moving_set_point(rps_start_pos[i], rps_init_pos_[i], st_enter_time_, clock_.time(), robot_joint_speed_[i + 2]);
            rps_pd_torques[i] = meii_.robot_joint_pd_controllers_[i + 2].calculate(rps_ref_pos[i], meii_.joints_[i + 2]->get_position(), 0, meii_.joints_[i + 2]->get_velocity());
        }
        if (rps_backdrive_ == 1) {
            commanded_torques[2] = 0;
            commanded_torques[3] = 0;
            commanded_torques[4] = 0;
        }
        else {
            commanded_torques[2] = rps_pd_torques[0];
            commanded_torques[3] = rps_pd_torques[1];
            commanded_torques[4] = rps_pd_torques[2];
        }

        // set command torques
        meii_.set_joint_torques(commanded_torques);

        // write to daq
        daq_->write_all();

        // check for rps mechanism in intialization position
        rps_init_ = check_rps_init(rps_init_pos_, meii_.get_wrist_parallel_positions(), { 1, 1, 1 });

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
    else if (rps_init_) {
        start_pos_ = meii_.get_anatomical_joint_positions(); // set initial waypoint as current position
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

    // initialize event variables
    st_enter_time_ = clock_.time();
    target_reached_ = false; // reset flag

    // initialize temporary variables for control
    double_vec ref_pos = start_pos_;
    double_vec pd_torques(meii_.N_aj_, 0.0);
    double_vec commanded_torques(meii_.N_aj_, 0.0);

    // set goal position for trajectory
    goal_pos_ = center_pos_;

    // MelShare unity interface
    target_num_ = 0;

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

        // compute pd torques in anatomical joint space (serial wrist)
        for (auto i = 0; i < meii_.N_aj_; ++i) {
            ref_pos[i] = moving_set_point(start_pos_[i], goal_pos_[i], st_enter_time_, clock_.time(), anatomical_joint_speed_[i]);
            pd_torques[i] = meii_.anatomical_joint_pd_controllers_[i].calculate(ref_pos[i], meii_.get_anatomical_joint_position(i), 0, meii_.get_anatomical_joint_velocity(i));
            if (anatomical_joint_backdrive_[i] == 1) {
                commanded_torques[i] = 0;
            }
            else {
                commanded_torques[i] = pd_torques[i];
            }
        }

        // set command torques
        meii_.set_anatomical_joint_torques(commanded_torques, meii_.error_code_);
        switch (meii_.error_code_) {
        case -1: util::print("ERROR: Eigensolver did not converge!");
            break;
        case -2: util::print("ERROR: Discontinuity in spectral norm of wrist jacobian");
            break;
        case -3: util::print("ERROR: Spectral norm of wrist Jacobian matrix too large");
            break;
        }

        if (meii_.error_code_ < 0) {
            stop_ = true;
            break;
        }

        // write motor commands to MelScope
        torque_share_.write(commanded_torques);

        // write to Unity
        target_num_share_.write(target_num_);

        // write to daq
        daq_->write_all();

        // check for target reached
        target_reached_ = check_target_reached(goal_pos_, meii_.get_anatomical_joint_positions(), target_check_joint_, target_anatomical_pos_tol_);

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
        start_pos_ = meii_.get_anatomical_joint_positions(); // set starting position as current position
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

    // initialize event variables
    st_enter_time_ = clock_.time();
    hold_center_time_reached_ = false;

    // initialize temporary variables for control
    double_vec ref_pos = start_pos_;
    double_vec pd_torques(meii_.N_aj_, 0.0);
    double_vec commanded_torques(meii_.N_aj_, 0.0);

    // initialize trajectory
    start_pos_ = center_pos_;
    goal_pos_ = center_pos_;

    // MelShare unity interface
    target_num_ = 0;

    // enter the control loop
    while (!hold_center_time_reached_ && !stop_) {

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

        // compute pd torques in anatomical joint space (serial wrist)
        for (auto i = 0; i < meii_.N_aj_; ++i) {
            ref_pos[i] = moving_set_point(start_pos_[i], goal_pos_[i], st_enter_time_, clock_.time(), anatomical_joint_speed_[i]);
            pd_torques[i] = meii_.anatomical_joint_pd_controllers_[i].calculate(ref_pos[i], meii_.get_anatomical_joint_position(i), 0, meii_.get_anatomical_joint_velocity(i));
            if (anatomical_joint_backdrive_[i] == 1) {
                commanded_torques[i] = 0;
            }
            else {
                commanded_torques[i] = pd_torques[i];
            }
        }

        // set command torques
        meii_.set_anatomical_joint_torques(commanded_torques, meii_.error_code_);
        switch (meii_.error_code_) {
        case -1: util::print("ERROR: Eigensolver did not converge!");
            break;
        case -2: util::print("ERROR: Discontinuity in spectral norm of wrist jacobian");
            break;
        case -3: util::print("ERROR: Spectral norm of wrist Jacobian matrix too large");
            break;
        }

        if (meii_.error_code_ < 0) {
            stop_ = true;
            break;
        }

        // write motor commands to MelScope
        torque_share_.write(commanded_torques);

        // write to unity
        target_num_share_.write(target_num_);

        // write to daq
        daq_->write_all();

        // check for hold time reached
        hold_center_time_reached_ = check_wait_time_reached(hold_center_time_, st_enter_time_, clock_.time());

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
    else if (hold_center_time_reached_) {
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

    // initialize event variables
    st_enter_time_ = clock_.time();

    // initialize temporary variables for control
    double_vec ref_pos = start_pos_;
    double_vec pd_torques(meii_.N_aj_, 0.0);
    double_vec commanded_torques(meii_.N_aj_, 0.0);

    // initialize trajectory
    start_pos_ = center_pos_;
    goal_pos_ = center_pos_;

    // initialize force checking algorithm
    force_mag_ = 0;
    force_mag_maintained_ = 0; // [s]
    force_mag_time_now_ = clock_.global_time();
    force_mag_time_last_ = clock_.global_time();

    // MelShare unity interface
    if (current_target_idx_ < target_sequence_.size()) {
        target_num_ = target_sequence_[current_target_idx_];
    }
    else {
        target_num_ = target_sequence_.back();
        end_of_target_sequence_ = true;
    }
    
    
    

    // initialize emg data buffer
    //boost::circular_buffer<double> emg_data_buffer_(200);
    //emg_data_buffer_ = mel::array_2D<double, 8, 200>(0);

    // enter the control loop
    while (!force_mag_reached_ && !end_of_target_sequence_ && !stop_) {

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

        // compute pd torques in anatomical joint space (serial wrist)
        for (auto i = 0; i < meii_.N_aj_; ++i) {
            ref_pos[i] = moving_set_point(start_pos_[i], goal_pos_[i], st_enter_time_, clock_.time(), anatomical_joint_speed_[i]);
            pd_torques[i] = meii_.anatomical_joint_pd_controllers_[i].calculate(ref_pos[i], meii_.get_anatomical_joint_position(i), 0, meii_.get_anatomical_joint_velocity(i));
            if (anatomical_joint_backdrive_[i] == 1) {
                commanded_torques[i] = 0;
            }
            else {
                commanded_torques[i] = pd_torques[i];
            }
        }

        // set command torques
        meii_.set_anatomical_joint_torques(commanded_torques, meii_.error_code_);
        switch (meii_.error_code_) {
        case -1: util::print("ERROR: Eigensolver did not converge!");
            break;
        case -2: util::print("ERROR: Discontinuity in spectral norm of wrist jacobian");
            break;
        case -3: util::print("ERROR: Spectral norm of wrist Jacobian matrix too large");
            break;
        }

        if (meii_.error_code_ < 0) {
            stop_ = true;
            break;
        }

        // write motor commands to MelScope
        torque_share_.write(commanded_torques);

        // measure interaction force for specified dof(s)
        force_mag_ = measure_task_force(commanded_torques, target_num_, dof_, condition_);
        
        //util::print(force_mag_);

        // get measured emg voltages
        // TO DO: add in band pass filtering
        emg_data_buffer_.push_back(meii_.get_emg_voltages());
        emg_share_.write(emg_data_buffer_.at(0));

        // check force magnitude
        force_mag_reached_ = check_force_mag_reached(force_mag_goal_, force_mag_);

        // write to unity
        target_num_share_.write(target_num_);
        force_mag_share_.write(force_mag_);

        // write to daq
        daq_->write_all();

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
    else if (force_mag_reached_) {
        force_mag_reached_ = false; // reset flag
        current_target_idx_++;
        event(ST_PROCESS_EMG);
    }
    else if (end_of_target_sequence_) {
        event(ST_TRAIN_CLASSIFIER);
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

    // extract features from EMG data
    feature_vec_ = feature_extract(emg_data_buffer_);

    // store features in training data set
    std::copy_n(feature_vec_.begin(), feature_array_.size(), feature_array_.begin());
    emg_training_data_.push_back(feature_array_);

    emg_data_processed_ = true;
    // transition to next state
    if (stop_) {
        // stop if user provided input
        event(ST_STOP);
    }
    else if (emg_data_processed_) {
        emg_data_processed_ = false; // reset flag
        event(ST_HOLD_CENTER);
    }
}


//-----------------------------------------------------------------------------
// "TRAIN CLASSIFIER" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRTControl::sf_train_classifier(const util::NoEventData* data) {
    
    util::print("Training Complete");

    // write to Unity (hasn't been working)
    // scene_num_ = 0;
    // scene_num_share_.write(scene_num_);

    meii_.disable();

    //open LDA script in Python
    //system("start EMG_FS_LDA.py &");

    // create vector to send training data to python
    double_vec emg_training_data_vec_(N_train_ * num_emg_channels_ * num_features_);
    for (int i = 0; i < N_train_; ++i) {
        std::copy_n(emg_training_data_[i].begin(), num_emg_channels_ * num_features_, emg_training_data_vec_.begin());
    }

    // write training data to python
    std::array<int, 2> training_data_size = {N_train_, num_emg_channels_ * num_features_};
    trng_size_.write(training_data_size);
    trng_share_.write(emg_training_data_vec_);
    std::vector<int> training_labels(N_train_);
    std::copy_n(target_sequence_.begin(), N_train_, training_labels.begin());
    label_share_.write(training_labels);

    std::array<int, 2> training_data_size2;
    std::vector<int> sel_feats(training_data_size2[1]);
    double_vec lda_coeff_c(num_class_, training_data_size2[1]); //replaced 2nd index (num_emg_channels_ * num_features_)

    // wait for python to receive
    // restart the clock
    clock_.start();
    while (!stop_) {

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

        //read for number of selected features from Python
        trng_size2_.read(training_data_size2);

        //read for selected feature indicies from Python
        feat_id_.read(sel_feats);

        //read for LDA Coefficients from Python
        lda_coeff_.read(lda_coeff_c);
        //mel::print(lda_coeff_c);

        // check for stop input
        stop_ = check_stop();

        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    event(ST_FINISH);
}

//-----------------------------------------------------------------------------
// "FINISH Experiment" STATE FUNCTION
//-----------------------------------------------------------------------------
void EmgRTControl::sf_finish(const util::NoEventData* data) {

    util::print("Finish Experiment");

    event(ST_STOP);

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

void EmgRTControl::set_experiment_conditions(int scene_num) {
    dof_ = (scene_num - 2) / 3;
    condition_ = (scene_num - 2) % 3;
}


bool EmgRTControl::check_rps_init(double_vec rps_init_pos, double_vec rps_current_pos, char_vec check_joint, bool print_output) const {

    bool pos_reached = true;
    for (int i = 0; i < 3; ++i) {
        if (check_joint[i]) {
            if (std::abs(rps_init_pos[i] - rps_current_pos[i]) > std::abs(rps_pos_tol_)) {
                if (print_output && pos_reached) {
                    std::cout << "Joint " << std::to_string(i) << " error is " << (std::abs(rps_init_pos[i] - rps_current_pos[i])) << std::endl;
                }
                pos_reached = false;
            }
        }
    }
    return pos_reached;
}

bool EmgRTControl::check_target_reached(double_vec goal_pos, double_vec current_pos, char_vec target_check_joint, double_vec target_tol, bool print_output) const {

    bool target_reached = true;
    for (int i = 0; i < 5; ++i) {
        if (target_check_joint[i]) {
            if (std::abs(goal_pos[i] - current_pos[i]) > std::abs(target_tol[i])) {
                if (print_output && target_reached) {
                    std::cout << "Joint " << std::to_string(i) << " error is " << (abs(goal_pos[i] - current_pos[i])*math::RAD2DEG) << std::endl;
                }
                target_reached = false;
            }
        }
    }
    return target_reached;
}

bool EmgRTControl::check_wait_time_reached(double wait_time, double init_time, double current_time) const {
    return (current_time - init_time) > wait_time;
}

double EmgRTControl::measure_task_force(double_vec commanded_torques, int target_num, int dof, int condition) const {
    double signed_task_force;
    if (dof < 4) {
        signed_task_force = (-1.0) * (commanded_torques[dof] + gravity_offsets_[dof]) * force_mag_goal_ / force_dof_scale_[dof] * target_dir_[dof][target_num-1];
    }
    else if (dof == 4) {
        signed_task_force = (-1.0) * ((commanded_torques[0] + gravity_offsets_[0]) * force_mag_goal_ / force_dof_scale_[0] * target_dir_[dof][target_num-1] + (commanded_torques[1] + gravity_offsets_[1]) * force_mag_goal_ / force_dof_scale_[1] * target_dir_[dof+1][target_num-1]) / 2.0;
    }
    else if (dof == 5) {
        signed_task_force = (-1.0) * ((commanded_torques[2] + gravity_offsets_[2]) * force_mag_goal_ / force_dof_scale_[2] * target_dir_[dof][target_num-1] + (commanded_torques[3] + gravity_offsets_[3]) * force_mag_goal_ / force_dof_scale_[3] * target_dir_[dof+1][target_num-1]) / 2.0;
    }
    return std::max(0.0, signed_task_force);
}

bool EmgRTControl::check_force_mag_reached(double force_mag_goal, double force_mag) {
    force_mag_time_now_ = clock_.global_time();
    force_mag_maintained_ = std::abs(force_mag_maintained_ + std::copysign(1.0, force_mag_tol_ - std::abs(force_mag_goal - force_mag)) * (force_mag_time_now_ - force_mag_time_last_));
    force_mag_time_last_ = force_mag_time_now_;
    return force_mag_maintained_ > force_mag_dwell_time_;
}

double_vec EmgRTControl::feature_extract(exo::MahiExoIIEmg::EmgDataBuffer& emg_data_buffer) {

    double_vec feature_vec;
    feature_vec.reserve(num_emg_channels_*num_features_);
    double_vec nrms_vec(num_emg_channels_, 0.0);
    double_vec nmav_vec(num_emg_channels_, 0.0);
    double_vec nwl_vec(num_emg_channels_, 0.0);
    double_vec nzc_vec(num_emg_channels_, 0.0);
    double_vec nssc_vec(num_emg_channels_, 0.0);
    double_vec ar_vec(4,0.0);
    double_vec ar1_vec(num_emg_channels_, 0.0);
    double_vec ar2_vec(num_emg_channels_, 0.0);
    double_vec ar3_vec(num_emg_channels_, 0.0);
    double_vec ar4_vec(num_emg_channels_, 0.0);

    // extract unnormalized features
    for (int i = 0; i < num_emg_channels_; ++i) {
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

    // normalize features
    double rms_mean = 0.0;
    double mav_mean = 0.0;
    double wl_mean = 0.0;
    double zc_mean = 0.0;
    double ssc_mean = 0.0;
    for (int i = 0; i < num_emg_channels_; ++i) {
        rms_mean += nrms_vec[i] / num_emg_channels_;
        mav_mean += nmav_vec[i] / num_emg_channels_;
        wl_mean += nwl_vec[i] / num_emg_channels_;
        zc_mean += nzc_vec[i] / num_emg_channels_;
        ssc_mean += nssc_vec[i] / num_emg_channels_;
    }
    for (int i = 0; i < num_emg_channels_; ++i) {
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