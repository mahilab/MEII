#include "EMGTesting/LogEmg.hpp"
#include "MEL/Utility/Windows/Keyboard.hpp"
#include "MEL/Daq/Quanser/Q8Usb.hpp"

using namespace mel;

LogEMG::LogEMG(Timer timer, Daq& daq, Input<voltage>& analog_input, Output<voltage>& analog_output, Watchdog& watchdog, MahiExoIIEmg& meii) :
    StateMachine(4),
    timer_(timer),
    daq_(daq),
    analog_input_(analog_input),
    analog_output_(analog_output),
    watchdog_(watchdog),
    meii_(meii),

    pos_share_("pos_share"),
    vel_share_("vel_share"),
    emg_share_("emg_share")
{

}


//-----------------------------------------------------------------------------
// "INITIALIZATION" STATE FUNCTION
//-----------------------------------------------------------------------------
void LogEMG::sf_init(const NoEventData* data) {


    // reset global variables
    emg_voltages_ = std::vector<double>(meii_.N_emg_, 0.0);
    filtered_emg_voltages_ = std::vector<double>(meii_.N_emg_, 0.0);

    // reset robot
    meii_.disable();

    // enable MEII EMG DAQ
    daq_.enable();
    if (!daq_.is_enabled()) {
        event(ST_FAULT_STOP);
        auto_stop_ = true;
        return;
    }

    // check DAQ behavior for safety
    analog_input_.update();
    meii_.update_kinematics();
    if (meii_.check_all_joint_limits()) {
        event(ST_FAULT_STOP);
        auto_stop_ = true;
        return;
    }
    if (daq_.identify(7)!=1) {
        event(ST_FAULT_STOP);
        auto_stop_ = true;
        return;
    }

    // enable MEII
    meii_.enable();
    if (!meii_.is_enabled()) {
        event(ST_FAULT_STOP);
        auto_stop_ = true;
        return;
    }

    // confirm start of experiment
    print("\nRunning Log EMG ... ");

    // start the watchdog
    watchdog_.start();

    // start the clock
    //clock_.start();

    // transition to next state from "INITIALIZATION"
    if (manual_stop_) {
        event(ST_FAULT_STOP);
    }
    else {
        event(ST_READ);
    }
}


//-----------------------------------------------------------------------------
// "READ" STATE FUNCTION
//-----------------------------------------------------------------------------
void LogEMG::sf_read(const NoEventData* data) {
    print("Reading EMG");

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

        // check joint limits
        if (meii_.check_all_joint_limits()) {
            auto_stop_ = true;
            break;
        }

        // get measured emg voltages
        emg_voltages_ = meii_.get_emg_voltages();
        meii_.butter_hp_.filter(emg_voltages_, filtered_emg_voltages_);
        emg_share_.write_data(filtered_emg_voltages_);

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

    // transition to next state from "READ"
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

void LogEMG::sf_stop(const NoEventData* data) {
    std::cout << "Exiting Program" << std::endl;
    
    // disable robot and daq
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

void LogEMG::sf_fault_stop(const NoEventData* data) {
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
// USER INPUT/OUTPUT UTILITY FUNCTIONS
//-----------------------------------------------------------------------------

void LogEMG::check_manual_stop() {
    std::vector<Key> keys{Keyboard::LControl, Keyboard::C };
    manual_stop_ = (Keyboard::are_all_keys_pressed(keys, false) | manual_stop_);
}