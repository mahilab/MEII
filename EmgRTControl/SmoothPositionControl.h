#pragma once
#include "StateMachine.h"
#include "Clock.h"
#include "Daq.h"
#include "MahiExoII.h"
#include "MelShare.h"

using namespace mel;

class SmoothPositionControlData : public util::EventData {

public:

};

class SmoothPositionControl : public util::StateMachine {

public:

    //---------------------------------------------------------------------
    // CONSTRUCTOR(S) / DESTRUCTOR(S)
    //---------------------------------------------------------------------

    SmoothPositionControl(util::Clock& clock, core::Daq* daq, exo::MahiExoII& meii);

private:

    //-------------------------------------------------------------------------
    // STATE MACHINE SETUP
    //-------------------------------------------------------------------------

    // STATES
    enum States {
        ST_INIT,
        ST_BACKDRIVE,
        ST_INIT_RPS,
        ST_TO_WAYPOINT,
        ST_HOLD_WAYPOINT,
        ST_FINISH,
        ST_STOP,
        ST_NUM_STATES
    };

    // STATE FUNCTIONS
    void sf_init(const util::NoEventData*);
    util::StateAction<SmoothPositionControl, util::NoEventData, &SmoothPositionControl::sf_init> sa_init;

    void sf_backdrive(const util::NoEventData*);
    util::StateAction<SmoothPositionControl, util::NoEventData, &SmoothPositionControl::sf_backdrive> sa_backdrive;

    void sf_init_rps(const util::NoEventData*);
    util::StateAction<SmoothPositionControl, util::NoEventData, &SmoothPositionControl::sf_init_rps> sa_init_rps;

    void sf_to_waypoint(const util::NoEventData*);
    util::StateAction<SmoothPositionControl, util::NoEventData, &SmoothPositionControl::sf_to_waypoint> sa_to_waypoint;

    void sf_hold_waypoint(const util::NoEventData*);
    util::StateAction<SmoothPositionControl, util::NoEventData, &SmoothPositionControl::sf_hold_waypoint> sa_hold_waypoint;

    void sf_finish(const util::NoEventData*);
    util::StateAction<SmoothPositionControl, util::NoEventData, &SmoothPositionControl::sf_finish> sa_finish;

    void sf_stop(const util::NoEventData*);
    util::StateAction<SmoothPositionControl, util::NoEventData, &SmoothPositionControl::sf_stop> sa_stop;

    // STATE MAP
    virtual const util::StateMapRow* get_state_map() {
        static const util::StateMapRow STATE_MAP[] = {
            &sa_init,
            &sa_backdrive,
            &sa_init_rps,
            &sa_to_waypoint,
            &sa_hold_waypoint,
            &sa_finish,
            &sa_stop,
        };
        return &STATE_MAP[0];
    }

    // USER INPUT CONTROL
    void wait_for_input();
    bool check_stop();
    bool stop_ = false;


    //-------------------------------------------------------------------------
    // SMOOTH POSITION CONTROL SETUP
    //-------------------------------------------------------------------------

    // HARDWARE CLOCK
    util::Clock clock_;

    // HARDWARE
    core::Daq* daq_;
    exo::MahiExoII meii_;

    // EXO PARAMETERS

    //char_vec anatomical_joint_backdrive_ = { 0, 0, 0, 0, 1 }; // 1 = backdrivable, 0 = active

    

    //-------------------------------------------------------------------------
    // WAYPOINT TRACKING
    //-------------------------------------------------------------------------

    // WAYPOINTS
    
    // waypoint indexing variable
    int current_wp_idx_ = 0;
    
    // waypoints specifying rps parallel coordinates
    /*std::vector<double_vec> wp_ = { { 0.09, 0.09, 0.09 },
                                    { 0.10, 0.10, 0.10 },
                                    { 0.10, 0.09, 0.11 },
                                    { 0.10, 0.10, 0.10 } };*/

    // waypoints specifying rps serial (anatomical) coordinates
    /*std::vector<double_vec> wp_ = { { 0.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.09 },
                                    { 0.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.10 },
                                    { 10.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.10 },
                                    { 0.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.10 },
                                    { -10.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.10 },
                                    { 0.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.10 },
                                    { 0.0 * math::DEG2RAD, 10.0 * math::DEG2RAD, 0.10 },
                                    { 0.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.10 },
                                    { 0.0 * math::DEG2RAD, -10.0 * math::DEG2RAD, 0.10 },
                                    { 0.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.10 } };*/

    // waypoints specifying anatomical coordinates
    std::vector<double_vec> wp_ = { { -35.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.10 },
                                    { -35.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 10.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.10 },
                                    { -35.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.10 },
                                    { -35.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, -10.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.10 },
                                    { -35.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.10 },
                                    { -35.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 10.0 * math::DEG2RAD, 0.10 },
                                    { -35.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.10 },
                                    { -35.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, -10.0 * math::DEG2RAD, 0.10 },
                                    { -35.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.10 } };

    size_t num_wp_ = size(wp_);


    // TIMING PARAMETERS
    const double init_backdrive_time_ = 1.0;
    const double hold_time_ = 1.0;

    // STATE TRANSITION EVENTS
    bool init_backdrive_time_reached_ = false;
    bool rps_init_ = false;
    bool waypoint_reached_ = false;
    bool hold_time_reached_ = false;

    // UTILITY FUNCTIONS
    bool check_wait_time_reached(double wait_time, double init_time, double current_time) const;

    // MELSCOPE VARIABLES
    comm::MelShare pos_share_ = comm::MelShare("pos_share");
    comm::MelShare vel_share_ = comm::MelShare("vel_share");
    comm::MelShare torque_share_ = comm::MelShare("torque_share");

};
