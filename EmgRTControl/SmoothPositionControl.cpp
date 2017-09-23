#include "SmoothPositionControl.h"
#include "Input.h"
#include "Q8Usb.h"
#include "mahiexoii_util.h"

using namespace mel;

SmoothPositionControl::SmoothPositionControl(util::Clock& clock, core::Daq* daq, exo::MahiExoII& meii) :
    StateMachine(7),
    clock_(clock),
    daq_(daq),
    meii_(meii)
{
}

void SmoothPositionControl::wait_for_input() {
    util::Input::wait_for_key(util::Input::Key::Space);
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
    util::Input::wait_for_key(util::Input::Key::Return);
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
    util::Input::wait_for_key(util::Input::Key::Return);
    meii_.enable();
    if (!meii_.is_enabled()) {
        event(ST_STOP);
        return;
    }

    // confirm start of experiment
    util::print("\nPress Enter to run Position Control");
    util::Input::wait_for_key(util::Input::Key::Return);
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
        event(ST_BACKDRIVE);
    }
}


//-----------------------------------------------------------------------------
// "BACKDRIVE" STATE FUNCTION
//-----------------------------------------------------------------------------
void SmoothPositionControl::sf_backdrive(const util::NoEventData* data) {
    util::print("Robot Backdriviable");

    // initialize event variables
    double st_enter_time = clock_.time();

    // enter the control loop
    while (!init_backdrive_time_reached_ && !stop_) {

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

        // check for init backdrive time reached
        init_backdrive_time_reached_ = check_wait_time_reached(init_backdrive_time_, st_enter_time, clock_.time());

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
void SmoothPositionControl::sf_init_rps(const util::NoEventData* data) {
    util::print("Initialize RPS Mechanism");

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

    // transition to next state
    if (stop_) {
        // stop if user provided input
        event(ST_STOP);
    }
    else if (rps_init_) {
        meii_.set_rps_control_mode(1); // initialize rps position controller mode and reference
        //meii_.rps_par_ref_.start(wp_[current_wp_idx_], meii_.get_wrist_parallel_positions(), clock_.time());
        //meii_.rps_ser_ref_.start(wp_[current_wp_idx_], meii_.get_wrist_serial_positions(), clock_.time());
        meii_.anat_ref_.start(wp_[current_wp_idx_], meii_.get_anatomical_joint_positions(), clock_.time());
        event(ST_TO_WAYPOINT);
    }
    else {
        util::print("ERROR: State transition undefined. Going to ST_STOP.");
        event(ST_STOP);
    }
}



//-----------------------------------------------------------------------------
// "GO TO WAYPOINT" STATE FUNCTION
//-----------------------------------------------------------------------------
void SmoothPositionControl::sf_to_waypoint(const util::NoEventData* data) {
    util::print("Moving to Waypoint");

    // initialize event variables
    waypoint_reached_ = false;

    // enter the control loop
    while (!waypoint_reached_ && !stop_) {

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
        //meii_.joints_[0]->set_torque(0.0);
        //meii_.joints_[1]->set_torque(0.0);

        // run position control
        //meii_.set_rps_pos_ctrl_torques(meii_.rps_par_ref_, clock_.time());
        //meii_.set_rps_pos_ctrl_torques(meii_.rps_ser_ref_, clock_.time());
        meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, clock_.time());

        // write to daq
        daq_->write_all();

        // check for waypoint reached
        //waypoint_reached_ = meii_.check_goal_rps_par_pos(wp_[current_wp_idx_], { 1, 1, 1 });
        //waypoint_reached_ = meii_.check_goal_rps_ser_pos(wp_[current_wp_idx_], { 1, 1, 0 });
        waypoint_reached_ = meii_.check_goal_anat_pos(wp_[current_wp_idx_], { 1, 1, 1, 1, 0 });

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
        event(ST_HOLD_WAYPOINT);
    }
    else {
        util::print("ERROR: State transition undefined. Going to ST_STOP.");
        event(ST_STOP);
    }
}


//-----------------------------------------------------------------------------
// "HOLD AT WAYPOINT" STATE FUNCTION
//-----------------------------------------------------------------------------
void SmoothPositionControl::sf_hold_waypoint(const util::NoEventData* data) {
    util::print("Holding at Waypoint");

    // initialize event variables
    double st_enter_time = clock_.time();
    hold_time_reached_ = false;

    // enter the control loop
    while (!hold_time_reached_ && !stop_) {

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
        //meii_.joints_[0]->set_torque(0.0);
        //meii_.joints_[1]->set_torque(0.0);

        // run position control
        //meii_.set_rps_pos_ctrl_torques(meii_.rps_par_ref_, clock_.time());
        //meii_.set_rps_pos_ctrl_torques(meii_.rps_ser_ref_, clock_.time());
        meii_.set_anat_pos_ctrl_torques(meii_.anat_ref_, clock_.time());

        // write to daq
        daq_->write_all();

        // check for hold time reached
        hold_time_reached_ = check_wait_time_reached(hold_time_, st_enter_time, clock_.time());

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
    else if (hold_time_reached_) {
        ++current_wp_idx_;
        if (current_wp_idx_ < num_wp_) {
            //meii_.rps_par_ref_.set_ref(wp_[current_wp_idx_], clock_.time());
            //meii_.rps_ser_ref_.set_ref(wp_[current_wp_idx_], clock_.time());
            meii_.anat_ref_.set_ref(wp_[current_wp_idx_], clock_.time());
            event(ST_TO_WAYPOINT);
        }
        else {
            //meii_.rps_par_ref_.stop();
            //meii_.rps_ser_ref_.stop();
            meii_.anat_ref_.stop();
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

bool SmoothPositionControl::check_wait_time_reached(double wait_time, double init_time, double current_time) const {
    return (current_time - init_time) > wait_time;
}