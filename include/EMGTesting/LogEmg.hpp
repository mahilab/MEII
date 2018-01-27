#pragma once
#include "MEL/Utility/StateMachine.hpp"
#include "MEL/Utility/Clock.hpp"
#include "MEL/Utility/Timer.hpp"
#include "MEL/Daq/Daq.hpp"
#include "MEL/Exoskeletons/MahiExoII/MahiExoIIEmg.hpp"
#include "MEL/Communications/Windows/MelShare.hpp"

using namespace mel;

class LogEMGData : public EventData {

public:

};

class LogEMG : public StateMachine {

public:

    //---------------------------------------------------------------------
    // CONSTRUCTOR(S) / DESTRUCTOR(S)
    //---------------------------------------------------------------------

    LogEMG(Timer timer, Daq& daq, Input<voltage>& analog_input, Output<voltage>& analog_output, Watchdog& watchdog, MahiExoIIEmg& meii);

private:

    //-------------------------------------------------------------------------
    // STATE MACHINE SETUP
    //-------------------------------------------------------------------------

    // STATES
    enum States {
        ST_INIT,
        ST_READ,
        ST_STOP,
        ST_FAULT_STOP,
        ST_NUM_STATES
    };

    // STATE FUNCTIONS
    void sf_init(const NoEventData*);
    StateAction<LogEMG, NoEventData, &LogEMG::sf_init> sa_init;

    void sf_read(const NoEventData*);
    StateAction<LogEMG, NoEventData, &LogEMG::sf_read> sa_read;

    void sf_stop(const NoEventData*);
    StateAction<LogEMG, NoEventData, &LogEMG::sf_stop> sa_stop;

    void sf_fault_stop(const NoEventData*);
    StateAction<LogEMG, NoEventData, &LogEMG::sf_fault_stop> sa_fault_stop;


    // STATE MAP
    virtual const StateMapRow* get_state_map() {
        static const StateMapRow STATE_MAP[] = {
            &sa_init,
            &sa_read,
            &sa_stop,
            &sa_fault_stop,
        };
        return &STATE_MAP[0];
    }



    //-------------------------------------------------------------------------
    // PRIVATE VARIABLES
    //-------------------------------------------------------------------------

    // HARDWARE CLOCK
    Timer timer_;

    // HARDWARE
    Daq& daq_;
    Input<voltage>& analog_input_;
    Output<voltage>& analog_output_;
    Watchdog& watchdog_;
    MahiExoIIEmg meii_;

    // TEMPORARY EMG DATA CONTAINERS
    std::vector<double> emg_voltages_ = std::vector<double>(meii_.N_emg_);
    std::vector<double> filtered_emg_voltages_ = std::vector<double>(meii_.N_emg_);

    // STATE TRANSITION EVENT VARIABLES
    bool auto_stop_ = false;
    bool manual_stop_ = false;

    // MELSCOPE VARIABLES
    MelShare pos_share_;
    MelShare vel_share_;
    MelShare emg_share_;

    //-------------------------------------------------------------------------
    // PRIVATE FUNCTIONS
    //-------------------------------------------------------------------------

    // USER INPUT/OUTPUT UTILITY FUNCTIONS
    void check_manual_stop();



};