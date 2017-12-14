#include "BackdriveMode.h"
#include "Input.h"
#include "mahiexoii_util.h"
#include "Q8Usb.h"

using namespace mel;

BackdriveMode::BackdriveMode(util::Clock& clock, core::Daq* daq, exo::MahiExoIIEmg& meii) :
    StateMachine(4),
    clock_(clock),
    daq_(daq),
    meii_(meii)
{
}


//-----------------------------------------------------------------------------
// "INITIALIZATION" STATE FUNCTION
//-----------------------------------------------------------------------------
void BackdriveMode::sf_init(const util::NoEventData* data) {

   
    // reset global variables
    emg_voltages_ = double_vec(meii_.N_emg_, 0.0);
    filtered_emg_voltages_ = double_vec(meii_.N_emg_, 0.0);

    // reset robot
    meii_.disable();

    // enable MEII EMG DAQ
    daq_->enable();
    if (!daq_->is_enabled()) {
        event(ST_FAULT_STOP);
        return;
    }

    // check DAQ behavior for safety
    daq_->read_all();
    meii_.update_kinematics();
    if (meii_.check_all_joint_limits()) {
        event(ST_FAULT_STOP);
        return;
    }
    if (!dev::Q8Usb::check_digital_loopback(0, 7)) {
        event(ST_FAULT_STOP);
        return;
    }

    // enable MEII
    meii_.enable();
    if (!meii_.is_enabled()) {
        event(ST_FAULT_STOP);
        return;
    }

    // confirm start of experiment
    util::print("\nRunning Backdrive Mode ... ");

    // start the watchdog
    daq_->start_watchdog(0.1);

    // start the clock
    clock_.start();

    // transition to next state from "INITIALIZATION"
    if (auto_stop_ || manual_stop_) {
        event(ST_FAULT_STOP);
    }
    else {
        event(ST_BACKDRIVE);
    }
}


//-----------------------------------------------------------------------------
// "BACKDRIVE" STATE FUNCTION
//-----------------------------------------------------------------------------
void BackdriveMode::sf_backdrive(const util::NoEventData* data) {
    util::print("Robot Backdrivable");

    // BACKDRIVE START
    sf_backdrive_start();

    // initialize local state variables
    const double_vec command_torques(meii_.N_aj_, 0.0);
    bool exit_program = false;

    // enter the control loop
    while (!exit_program && !manual_stop_ && !auto_stop_) {

        // read and reload DAQs
        daq_->reload_watchdog();
        daq_->read_all();

        // update robot kinematics
        meii_.update_kinematics();

        // BACKDRIVE STEP
        sf_backdrive_step();

        // check joint limits
        if (meii_.check_all_joint_limits()) {
            auto_stop_ = true;
            break;
        }

        // check for end of experiment
        if (util::Input::is_key_pressed(util::Input::Enter)) {
            exit_program = true;
        }

        // check for manual stop input
        check_manual_stop();

        // set zero torques
        meii_.set_joint_torques(command_torques);

        // write to daq
        daq_->write_all();

        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    // BACKDRIVE STOP
    sf_backdrive_stop();

    // transition to next state from "BACKDRIVE"
    if (auto_stop_ || manual_stop_) {
        event(ST_FAULT_STOP);
    }
    else if (exit_program) {
        event(ST_STOP);
    }
    else {
        util::print("ERROR: State transition undefined. Going to ST_FAULT_STOP.");
        event(ST_FAULT_STOP);
    }
}


//-----------------------------------------------------------------------------
// "STOP" STATE FUNCTION
//-----------------------------------------------------------------------------

void BackdriveMode::sf_stop(const util::NoEventData* data) {
    std::cout << "Exiting Program" << std::endl;
    if (meii_.is_enabled()) {
        meii_.disable();
    }
    if (daq_->is_enabled()) {
        daq_->disable();
    }
}

//-----------------------------------------------------------------------------
// "FAULT STOP" STATE FUNCTION
//-----------------------------------------------------------------------------

void BackdriveMode::sf_fault_stop(const util::NoEventData* data) {
    std::cout << "Program Stopped with Potential Fault" << std::endl;

    // disable robot and daq
    if (meii_.is_enabled()) {
        meii_.disable();
    }
    if (daq_->is_enabled()) {
        if (daq_->is_watchdog_expired()) {
            util::print("WATCHDOG HAS EXPIRED.");
        }
        daq_->disable();
    }

}

//-----------------------------------------------------------------------------
// NESTED TRANSPARENT STATE FUNCTIONS
//-----------------------------------------------------------------------------

void BackdriveMode::sf_backdrive_start() {

    /*// initialize kinematics test
    double q_ser_0_ = meii_.get_anatomical_joint_position(2);
    double q_ser_1_ = meii_.get_anatomical_joint_position(3);
    double q_ser_2_ = meii_.get_anatomical_joint_position(4);

    mel::Integrator q_ser_igr_0_ = mel::Integrator(q_ser_0_);
    mel::Integrator q_ser_igr_1_ = mel::Integrator(q_ser_1_);
    mel::Integrator q_ser_igr_2_ = mel::Integrator(q_ser_2_);*/
}

void BackdriveMode::sf_backdrive_step() {

    /*// test kinematics
    q_ser_0_ = q_ser_igr_0_.integrate(meii_.get_anatomical_joint_velocity(2), clock_.time());
    q_ser_1_ = q_ser_igr_1_.integrate(meii_.get_anatomical_joint_velocity(3), clock_.time());
    q_ser_2_ = q_ser_igr_2_.integrate(meii_.get_anatomical_joint_velocity(4), clock_.time());

    // print
    std::cout << q_ser_2_ << " " << meii_.get_anatomical_joint_position(4) << std::endl;*/

    // get measured emg voltages
    //emg_voltages_ = meii_.get_emg_voltages();
    //meii_.butter_hp_.filter(meii_.get_emg_voltages(), filtered_emg_voltages_);
    //emg_share_.write(emg_voltages_);
    //filter_emg_share_.write(filtered_emg_voltages_);

}

void BackdriveMode::sf_backdrive_stop() {

}

//-----------------------------------------------------------------------------
// USER INPUT/OUTPUT UTILITY FUNCTIONS
//-----------------------------------------------------------------------------

void BackdriveMode::check_manual_stop() {
    std::vector<util::Input::Key> keys{ util::Input::LControl, util::Input::C };
    manual_stop_ = (util::Input::are_all_keys_pressed(keys, false) | manual_stop_);
}