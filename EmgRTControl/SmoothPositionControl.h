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
        ST_TRANSPARENT,
        ST_INIT_RPS,
        ST_WAYPOINT,
        ST_FINISH,
        ST_STOP,
        ST_NUM_STATES
    };

    // STATE FUNCTIONS
    void sf_init(const util::NoEventData*);
    util::StateAction<SmoothPositionControl, util::NoEventData, &SmoothPositionControl::sf_init> sa_init;

    void sf_transparent(const util::NoEventData*);
    util::StateAction<SmoothPositionControl, util::NoEventData, &SmoothPositionControl::sf_transparent> sa_transparent;

    void sf_init_rps(const util::NoEventData*);
    util::StateAction<SmoothPositionControl, util::NoEventData, &SmoothPositionControl::sf_init_rps> sa_init_rps;

    void sf_waypoint(const util::NoEventData*);
    util::StateAction<SmoothPositionControl, util::NoEventData, &SmoothPositionControl::sf_waypoint> sa_waypoint;

    void sf_finish(const util::NoEventData*);
    util::StateAction<SmoothPositionControl, util::NoEventData, &SmoothPositionControl::sf_finish> sa_finish;

    void sf_stop(const util::NoEventData*);
    util::StateAction<SmoothPositionControl, util::NoEventData, &SmoothPositionControl::sf_stop> sa_stop;

    // STATE MAP
    virtual const util::StateMapRow* get_state_map() {
        static const util::StateMapRow STATE_MAP[] = {
            &sa_init,
            &sa_transparent,
            &sa_init_rps,
            &sa_waypoint,
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
    bool rps_backdrive_ = false; // 1 = backdrivable, 0 = active
    char_vec anatomical_joint_backdrive_ = { 1, 1, 0, 0, 1 }; // 1 = backdrivable, 0 = active
    double_vec robot_joint_speed_ = { 0.25, 0.25, 0.015, 0.015, 0.015 };
    double_vec anatomical_joint_speed_ = { 0.25, 0.25, 0.125, 0.125, 0.0125 };
    

    // RPS MECHANISM SAFE INITIALIZATION PARAMETERS
    int rps_control_mode_ = 0; // 0 = robot joint space (parallel), 1 = anatomical joint space (serial)
    double_vec rps_init_pos_ = { 0.10, 0.10, 0.10 };
    double rps_pos_tol_ = 0.005;

    // SMOOTH REFERENCE TRAJECTORY
    double init_time_ = 0.0;

    //-------------------------------------------------------------------------
    // WAYPOINT TRACKING
    //-------------------------------------------------------------------------

    // WAYPOINTS
    int num_wp_ = 1;
    int current_wp_ = 0;
    double_vec wp_1_ = { -35.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.0 * math::DEG2RAD, 0.0 * math::DEG2RAD,  0.10 }; // anatomical joint positions

    // TEMPORARY WAYPOINT CONTAINERS
    double_vec start_pos_ = double_vec(5, 0.0);
    double_vec goal_pos_ = double_vec(5, 0.0);

    // CHECKING WAYPOINT REACHED
    char_vec check_joint_ = { 1, 1, 1, 1, 1 };
    double_vec pos_tol_ = { 1.0 * math::DEG2RAD, 1.0 * math::DEG2RAD, 1.0 * math::DEG2RAD, 1.0 * math::DEG2RAD, 0.01 };

    // TIMING PARAMETERS
    double st_enter_time_;
    double init_transparent_time_ = 1.0;

    // STATE TRANSITION EVENTS
    bool init_transparent_time_reached_ = false;
    bool rps_init_ = false;
    bool waypoint_reached_ = false;

    // UTILITY FUNCTIONS
    bool check_rps_init(double_vec rps_init_pos, double_vec rps_current_pos, char_vec check_joint, bool print_output = false) const;
    bool check_waypoint_reached(double_vec goal_pos, double_vec current_pos, char_vec check_joint, bool print_output = false) const;
    bool check_wait_time_reached(double wait_time, double init_time, double current_time) const;

    // MELSCOPE VARIABLES
    comm::MelShare pos_share_ = comm::MelShare("pos_share");
    comm::MelShare vel_share_ = comm::MelShare("vel_share");
    comm::MelShare torque_share_ = comm::MelShare("torque_share");

};
