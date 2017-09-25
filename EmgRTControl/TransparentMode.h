#pragma once
#include "StateMachine.h"
#include "Clock.h"
#include "Daq.h"
#include "MahiExoIIEmg.h"
#include "MelShare.h"

using namespace mel;

class TransparentModeData : public util::EventData {

public:

};

class TransparentMode : public util::StateMachine {

public:

    //---------------------------------------------------------------------
    // CONSTRUCTOR(S) / DESTRUCTOR(S)
    //---------------------------------------------------------------------

    TransparentMode(util::Clock& clock, core::Daq* daq, exo::MahiExoIIEmg& meii);

private:

    //-------------------------------------------------------------------------
    // STATE MACHINE SETUP
    //-------------------------------------------------------------------------

    // STATES
    enum States {
        ST_INIT,
        ST_TRANSPARENT,
        ST_STOP,
        ST_NUM_STATES
    };

    // STATE FUNCTIONS
    void sf_init(const util::NoEventData*);
    util::StateAction<TransparentMode, util::NoEventData, &TransparentMode::sf_init> sa_init;

    void sf_transparent(const util::NoEventData*);
    util::StateAction<TransparentMode, util::NoEventData, &TransparentMode::sf_transparent> sa_transparent;

    void sf_stop(const util::NoEventData*);
    util::StateAction<TransparentMode, util::NoEventData, &TransparentMode::sf_stop> sa_stop;

    // STATE MAP
    virtual const util::StateMapRow* get_state_map() {
        static const util::StateMapRow STATE_MAP[] = {
            &sa_init,
            &sa_transparent,
            &sa_stop,
        };
        return &STATE_MAP[0];
    }

    // USER INPUT CONTROL
    void wait_for_input();
    bool check_stop();
    bool stop_ = false;

    //-------------------------------------------------------------------------
    // TRANSPARENT MODE COMPONENTS
    //-------------------------------------------------------------------------

    // HARDWARE CLOCK
    util::Clock clock_;

    // HARDWARE
    core::Daq* daq_;
    exo::MahiExoIIEmg meii_;

    // GENERIC STATE FUNCTIONS
    void sf_transparent_start();
    void sf_transparent_step();
    void sf_transparent_stop();

    //-------------------------------------------------------------------------
    // TRANSPARENT MODE
    //-------------------------------------------------------------------------


    double_vec emg_voltages_ = double_vec(meii_.N_emg_, 0.0);
    double_vec filtered_emg_voltages_ = double_vec(meii_.N_emg_, 0.0);

    // MELSCOPE VARIABLES
    comm::MelShare pos_share_ = comm::MelShare("pos_share");
    comm::MelShare vel_share_ = comm::MelShare("vel_share");
    comm::MelShare emg_share_ = comm::MelShare("emg_share");
    comm::MelShare filter_emg_share_ = comm::MelShare("filter_emg_share");

};