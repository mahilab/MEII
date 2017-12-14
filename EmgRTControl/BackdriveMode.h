#pragma once
#include "StateMachine.h"
#include "Clock.h"
#include "Daq.h"
#include "MahiExoIIEmg.h"
#include "MelShare.h"

using namespace mel;

class BackdriveModeData : public util::EventData {

public:

};

class BackdriveMode : public util::StateMachine {

public:

    //---------------------------------------------------------------------
    // CONSTRUCTOR(S) / DESTRUCTOR(S)
    //---------------------------------------------------------------------

    BackdriveMode(util::Clock& clock, core::Daq* daq, exo::MahiExoIIEmg& meii);

private:

    //-------------------------------------------------------------------------
    // STATE MACHINE SETUP
    //-------------------------------------------------------------------------

    // STATES
    enum States {
        ST_INIT,
        ST_BACKDRIVE,
        ST_STOP,
        ST_FAULT_STOP,
        ST_NUM_STATES
    };

    // STATE FUNCTIONS
    void sf_init(const util::NoEventData*);
    util::StateAction<BackdriveMode, util::NoEventData, &BackdriveMode::sf_init> sa_init;

    void sf_backdrive(const util::NoEventData*);
    util::StateAction<BackdriveMode, util::NoEventData, &BackdriveMode::sf_backdrive> sa_backdrive;

    void sf_stop(const util::NoEventData*);
    util::StateAction<BackdriveMode, util::NoEventData, &BackdriveMode::sf_stop> sa_stop;

    void sf_fault_stop(const util::NoEventData*);
    util::StateAction<BackdriveMode, util::NoEventData, &BackdriveMode::sf_fault_stop> sa_fault_stop;

    // STATE MAP
    virtual const util::StateMapRow* get_state_map() {
        static const util::StateMapRow STATE_MAP[] = {
            &sa_init,
            &sa_backdrive,
            &sa_stop,
            &sa_fault_stop,
        };
        return &STATE_MAP[0];
    }

    

    //-------------------------------------------------------------------------
    // PRIVATE VARIABLES
    //-------------------------------------------------------------------------

    // HARDWARE CLOCK
    util::Clock clock_;

    // HARDWARE
    core::Daq* daq_;
    exo::MahiExoIIEmg meii_;

    // TEMPORARY EMG DATA CONTAINERS
    double_vec emg_voltages_ = double_vec(meii_.N_emg_);
    double_vec filtered_emg_voltages_ = double_vec(meii_.N_emg_);

    // STATE TRANSITION EVENT VARIABLES
    bool auto_stop_ = false;
    bool manual_stop_ = false;

    // MELSCOPE VARIABLES
    comm::MelShare pos_share_ = comm::MelShare("pos_share");
    comm::MelShare vel_share_ = comm::MelShare("vel_share");
    comm::MelShare emg_share_ = comm::MelShare("emg_share");
    comm::MelShare filter_emg_share_ = comm::MelShare("filter_emg_share");

    //-------------------------------------------------------------------------
    // PRIVATE FUNCTIONS
    //-------------------------------------------------------------------------

    // GENERIC STATE FUNCTIONS
    void sf_backdrive_start();
    void sf_backdrive_step();
    void sf_backdrive_stop();

    // USER INPUT/OUTPUT UTILITY FUNCTIONS
    void check_manual_stop();

    

};