#include "EmgRTControl/BackdriveMode.hpp"
#include "MEL/Utility/Windows/Keyboard.hpp"

using namespace mel;

BackdriveMode::BackdriveMode(Timer timer,Daq& daq, Input<voltage>& analog_input, Output<voltage>& analog_output, Watchdog& watchdog, MahiExoIIEmg& meii) :
    StateMachine(4),
    timer_(timer),
    daq_(daq),
    analog_input_(analog_input),
    analog_output_(analog_output_),
    watchdog_(watchdog),
    meii_(meii),
    pos_share_("pos_share"),
    vel_share_("vel_share"),
    emg_share_("emg_share"),
    filter_emg_share_("filter_emg_share")
{

}


//-----------------------------------------------------------------------------
// "INITIALIZATION" STATE FUNCTION
//-----------------------------------------------------------------------------
void BackdriveMode::sf_init(const NoEventData* data) {

   
    // reset global variables
    emg_voltages_ = std::vector<double>(meii_.N_emg_, 0.0);
    filtered_emg_voltages_ = std::vector<double>(meii_.N_emg_, 0.0);

    // reset robot
    meii_.disable();

    // enable MEII EMG DAQ
    daq_.enable();
    if (!daq_.is_enabled()) {
        event(ST_FAULT_STOP);
        return;
    }

    // check DAQ behavior for safety
    analog_input_.update();
    meii_.update_kinematics();
    if (meii_.check_all_joint_limits()) {
        event(ST_FAULT_STOP);
        return;
    }

    //Removed: should add to main loop to make this generalizable
    //if (!Q8Usb::identify(7)) {
    //    event(ST_FAULT_STOP);
    //    return;
    //}

    // enable MEII
    meii_.enable();
    if (!meii_.is_enabled()) {
        event(ST_FAULT_STOP);
        return;
    }

    // confirm start of experiment
    print("\nRunning Backdrive Mode ... ");

    // start the watchdog
    watchdog_.start();

    // start the clock
    timer_.restart();

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
void BackdriveMode::sf_backdrive(const NoEventData* data) {
    print("Robot Backdrivable");

    // BACKDRIVE START
    sf_backdrive_start();

    // initialize local state variables
    const std::vector<double> command_torques(meii_.N_aj_, 0.0);
    bool exit_program = false;

    // enter the control loop
    while (!exit_program && !manual_stop_ && !auto_stop_) {

        // read and reload DAQs
        watchdog_.kick();
        analog_input_.update();

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
        if (Keyboard::is_key_pressed(Keyboard::Enter)) {
            exit_program = true;
        }

        // check for manual stop input
        check_manual_stop();

        // set zero torques
        meii_.set_joint_torques(command_torques);

        // write to daq
        analog_output_.update();

        // wait for the next clock cycle
        timer_.wait();
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
        print("ERROR: State transition undefined. Going to ST_FAULT_STOP.");
        event(ST_FAULT_STOP);
    }
}


//-----------------------------------------------------------------------------
// "STOP" STATE FUNCTION
//-----------------------------------------------------------------------------

void BackdriveMode::sf_stop(const NoEventData* data) {
    std::cout << "Exiting Program" << std::endl;
    if (meii_.is_enabled()) {
        meii_.disable();
    }
    if (daq_.is_enabled()) {
        daq_.disable();
    }
}

//-----------------------------------------------------------------------------
// "FAULT STOP" STATE FUNCTION
//-----------------------------------------------------------------------------

void BackdriveMode::sf_fault_stop(const NoEventData* data) {
    std::cout << "Program Stopped with Potential Fault" << std::endl;

    // disable robot and daq
    if (meii_.is_enabled()) {
        meii_.disable();
    }
    if (daq_.is_enabled()) {
        if (watchdog_.is_expired()) {
            print("WATCHDOG HAS EXPIRED.");
        }
        daq_.disable();
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

    Integrator q_ser_igr_0_ = Integrator(q_ser_0_);
    Integrator q_ser_igr_1_ = Integrator(q_ser_1_);
    Integrator q_ser_igr_2_ = Integrator(q_ser_2_);*/
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
    //emg_share_.write_data(emg_voltages_);
    //filter_emg_share_.write_data(filtered_emg_voltages_);

}

void BackdriveMode::sf_backdrive_stop() {

}

//-----------------------------------------------------------------------------
// USER INPUT/OUTPUT UTILITY FUNCTIONS
//-----------------------------------------------------------------------------

void BackdriveMode::check_manual_stop() {
    std::vector<Keyboard::Key> keys{ Keyboard::LControl, Keyboard::C };
    manual_stop_ = (Keyboard::are_all_keys_pressed(keys, false) | manual_stop_);
}