#include "SmoothPositionControl.h"
#include "Input.h"
#include "Q8Usb.h"
#include "mahiexoii_util.h"

using namespace mel;

SmoothPositionControl::SmoothPositionControl(util::Clock& clock, core::Daq* daq, exo::MahiExoII& meii) :
    StateMachine(6),
    clock_(clock),
    daq_(daq),
    meii_(meii)
{
}

void SmoothPositionControl::wait_for_input() {
    util::Input::wait_for_key_press(util::Input::Key::Space);
}

bool SmoothPositionControl::check_stop() {
    return util::Input::is_key_pressed(util::Input::Escape) || (util::Input::is_key_pressed(util::Input::LControl) && util::Input::is_key_pressed(util::Input::C));
}

//-----------------------------------------------------------------------------
// "INITIALIZATION" STATE FUNCTION
//-----------------------------------------------------------------------------
void SmoothPositionControl::sf_init(const util::NoEventData* data) {

    // enable MEII EMG DAQ
    util::print("\nPress Enter to enable MEII EMG Daq <" + daq_->name_ + ">.");
    util::Input::wait_for_key_press(util::Input::Key::Return);
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
    util::print("\nPress Enter to enable MEII.");
    util::Input::wait_for_key_press(util::Input::Key::Return);
    meii_.enable();
    if (!meii_.is_enabled()) {
        event(ST_STOP);
        return;
    }

    // confirm start of experiment
    util::print("\nPress Enter to run Position Control");
    util::Input::wait_for_key_press(util::Input::Key::Return);
    util::print("\nRunning Position Control ... ");

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
void SmoothPositionControl::sf_transparent(const util::NoEventData* data) {
    util::print("Robot Transparent");

    // initialize event variables
    st_enter_time_ = clock_.time();

    // enter the control loop
    while (!init_transparent_time_reached_ && !stop_) {

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

        // set zero torques
        meii_.set_joint_torques({ 0.0, 0.0, 0.0, 0.0, 0.0 });

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
void SmoothPositionControl::sf_init_rps(const util::NoEventData* data) {
    util::print("Initialize RPS Mechanism");

    // initialize event variables
    st_enter_time_ = clock_.time();

    // initialize temporary variables
    meii_.update_kinematics();
    double_vec rps_start_pos = meii_.get_wrist_parallel_positions();
    double_vec rps_ref_pos = rps_start_pos;
    double_vec rps_pd_torques(3, 0.0);
    double_vec commanded_torques(meii_.N_aj_, 0.0);

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
            rps_ref_pos[i] = moving_set_point(rps_start_pos[i], rps_init_pos_[i], st_enter_time_, clock_.time(), robot_joint_speed_[i+2]);
            rps_pd_torques[i] = meii_.robot_joint_pd_controllers_[i+2].calculate(rps_ref_pos[i], meii_.joints_[i+2]->get_position(), 0, meii_.joints_[i+2]->get_velocity());
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
        event(ST_WAYPOINT);
    }
    else {
        util::print("ERROR: State transition undefined. Going to ST_STOP.");
        event(ST_STOP);
    }
}



//-----------------------------------------------------------------------------
// "WAYPOINT" STATE FUNCTION
//-----------------------------------------------------------------------------
void SmoothPositionControl::sf_waypoint(const util::NoEventData* data) {
    util::print("Moving to Waypoint");

    // initialize event variables
    st_enter_time_ = clock_.time();

    // initialize event variables
    waypoint_reached_ = false;

    // initialize temporary variables 
    double_vec ref_pos = start_pos_;
    double_vec pd_torques(meii_.N_aj_, 0.0);
    double_vec commanded_torques(meii_.N_aj_, 0.0);

    // set goal position as next waypoint position
    goal_pos_ = wp_1_;

    // enter the control loop
    //while (!waypoint_reached_ && !stop_) {
    while (!stop_) {

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

        util::print(commanded_torques[2]);

        // write to daq
        daq_->write_all();

        // check for waypoint reached
        waypoint_reached_ = check_waypoint_reached(goal_pos_, meii_.get_anatomical_joint_positions(), check_joint_);

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
    else if (waypoint_reached_) {
        ++current_wp_;
        if (current_wp_ < num_wp_) {
            start_pos_ = goal_pos_; // set starting position as current waypoint
            event(ST_WAYPOINT);
        }
        else {
            event(ST_FINISH);
        }
    }
    else {
        util::print("ERROR: State transition undefined. Going to ST_STOP.");
        event(ST_STOP);
    }
}

//-----------------------------------------------------------------------------
// "FINISH" STATE FUNCTION
//-----------------------------------------------------------------------------
void SmoothPositionControl::sf_finish(const util::NoEventData* data) {
    util::print("Finish Execution");

    if (meii_.is_enabled()) {
        meii_.disable();
    }
    if (daq_->is_enabled()) {
        daq_->disable();
    }

}

//-----------------------------------------------------------------------------
// "STOP" STATE FUNCTION
//-----------------------------------------------------------------------------

void SmoothPositionControl::sf_stop(const util::NoEventData* data) {
    std::cout << "Stop Robot" << std::endl;
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

bool SmoothPositionControl::check_rps_init(double_vec rps_init_pos, double_vec rps_current_pos, char_vec check_joint, bool print_output) const {

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

bool SmoothPositionControl::check_waypoint_reached(double_vec goal_pos, double_vec current_pos, char_vec check_joint, bool print_output) const {

    bool pos_reached = true;
    for (int i = 0; i < 5; ++i) {
        if (check_joint[i]) {
            if (std::abs(goal_pos[i] - current_pos[i]) > std::abs(pos_tol_[i])) {
                if (print_output && pos_reached) {
                    std::cout << "Joint " << std::to_string(i) << " error is " << (std::abs(goal_pos[i] - current_pos[i])*math::RAD2DEG) << std::endl;
                }
                pos_reached = false;
            }
        }
    }
    return pos_reached;
}

bool SmoothPositionControl::check_wait_time_reached(double wait_time, double init_time, double current_time) const {
    return (current_time - init_time) > wait_time;
}