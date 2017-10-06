#include "TransparentMode.h"
#include "Input.h"
#include "mahiexoii_util.h"
#include "Q8Usb.h"

using namespace mel;

TransparentMode::TransparentMode(util::Clock& clock, core::Daq* daq, exo::MahiExoIIEmg& meii) :
    StateMachine(3),
    clock_(clock),
    daq_(daq),
    meii_(meii)
{
}

void TransparentMode::wait_for_input() {
    util::Input::wait_for_key(util::Input::Key::Space);
}

bool TransparentMode::check_stop() {
    return util::Input::is_key_pressed(util::Input::Escape) || (util::Input::is_key_pressed(util::Input::LControl) && util::Input::is_key_pressed(util::Input::C));
}

//-----------------------------------------------------------------------------
// "INITIALIZATION" STATE FUNCTION
//-----------------------------------------------------------------------------
void TransparentMode::sf_init(const util::NoEventData* data) {

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
    util::print("\nPress Enter to run Transparent Mode");
    util::Input::wait_for_key(util::Input::Key::Return);
    util::print("\nRunning Transparent Mode ... ");

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
void TransparentMode::sf_transparent(const util::NoEventData* data) {
    util::print("Robot Transparent");

    // TRANSPARENT START
    sf_transparent_start();

    // initialize local state variables
    const double_vec command_torques(meii_.N_aj_, 0.0);

    // enter the control loop
    while (!stop_) {

        // read and reload DAQs
        daq_->reload_watchdog();
        daq_->read_all();

        // update robot kinematics
        meii_.update_kinematics();

        // TRANSPARENT STEP
        sf_transparent_step();

        // check joint limits
        if (meii_.check_all_joint_limits()) {
            stop_ = true;
            break;
        }

        // write kinematics to MelScope
        pos_share_.write(meii_.get_anatomical_joint_positions());
        vel_share_.write(meii_.get_anatomical_joint_velocities());

        // set zero torques
        meii_.set_joint_torques(command_torques);

        // write to daq
        daq_->write_all();

        // check for stop input
        stop_ = check_stop();

        // wait for the next clock cycle
        clock_.wait();
    }

    // TRANSPARENT STOP
    sf_transparent_stop();

    // transition to next state
    if (stop_) {
        // stop if user provided input
        event(ST_STOP);
    }
    else {
        util::print("ERROR: State transition undefined. Going to ST_STOP.");
        event(ST_STOP);
    }
}


//-----------------------------------------------------------------------------
// "STOP" STATE FUNCTION
//-----------------------------------------------------------------------------

void TransparentMode::sf_stop(const util::NoEventData* data) {
    std::cout << "State Stop " << std::endl;
    if (meii_.is_enabled()) {
        meii_.disable();
    }
    if (daq_->is_enabled()) {
        daq_->disable();
    }
}

//-----------------------------------------------------------------------------
// NESTED TRANSPARENT STATE FUNCTIONS
//-----------------------------------------------------------------------------

void TransparentMode::sf_transparent_start() {

    /*// initialize kinematics test
    double q_ser_0_ = meii_.get_anatomical_joint_position(2);
    double q_ser_1_ = meii_.get_anatomical_joint_position(3);
    double q_ser_2_ = meii_.get_anatomical_joint_position(4);

    mel::Integrator q_ser_igr_0_ = mel::Integrator(q_ser_0_);
    mel::Integrator q_ser_igr_1_ = mel::Integrator(q_ser_1_);
    mel::Integrator q_ser_igr_2_ = mel::Integrator(q_ser_2_);*/
}

void TransparentMode::sf_transparent_step() {

    /*// test kinematics
    q_ser_0_ = q_ser_igr_0_.integrate(meii_.get_anatomical_joint_velocity(2), clock_.time());
    q_ser_1_ = q_ser_igr_1_.integrate(meii_.get_anatomical_joint_velocity(3), clock_.time());
    q_ser_2_ = q_ser_igr_2_.integrate(meii_.get_anatomical_joint_velocity(4), clock_.time());

    // print
    std::cout << q_ser_2_ << " " << meii_.get_anatomical_joint_position(4) << std::endl;*/

    // get measured emg voltages
    emg_voltages_ = meii_.get_emg_voltages();
    meii_.butter_hp_.filter(meii_.get_emg_voltages(), filtered_emg_voltages_);
    emg_share_.write(emg_voltages_);
    filter_emg_share_.write(filtered_emg_voltages_);

}

void TransparentMode::sf_transparent_stop() {

}