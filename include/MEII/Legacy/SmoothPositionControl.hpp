#pragma once
#include "MEL/Utility/StateMachine.hpp"
#include "MEL/Utility/Clock.hpp"
#include "MEL/Daq/Daq.hpp"
#include "MEL/Exoskeletons/MahiExoII/MahiExoII.hpp"
#include "MEL/Communications/Windows/MelShare.hpp"
#include "MEL/Utility/Timer.hpp"

using namespace mel;

class SmoothPositionControlData : public EventData {

public:

};

class SmoothPositionControl : public StateMachine {

public:

    //---------------------------------------------------------------------
    // CONSTRUCTOR(S) / DESTRUCTOR(S)
    //---------------------------------------------------------------------

    SmoothPositionControl(Clock& clock, Timer timer, Daq& daq, Input<voltage>& analog_input, Output<voltage>& analog_ouput, Watchdog& watchdog, MahiExoII& meii);

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
    void sf_init(const NoEventData*);
    StateAction<SmoothPositionControl, NoEventData, &SmoothPositionControl::sf_init> sa_init;

    void sf_backdrive(const NoEventData*);
    StateAction<SmoothPositionControl, NoEventData, &SmoothPositionControl::sf_backdrive> sa_backdrive;

    void sf_init_rps(const NoEventData*);
    StateAction<SmoothPositionControl, NoEventData, &SmoothPositionControl::sf_init_rps> sa_init_rps;

    void sf_to_waypoint(const NoEventData*);
    StateAction<SmoothPositionControl, NoEventData, &SmoothPositionControl::sf_to_waypoint> sa_to_waypoint;

    void sf_hold_waypoint(const NoEventData*);
    StateAction<SmoothPositionControl, NoEventData, &SmoothPositionControl::sf_hold_waypoint> sa_hold_waypoint;

    void sf_finish(const NoEventData*);
    StateAction<SmoothPositionControl, NoEventData, &SmoothPositionControl::sf_finish> sa_finish;

    void sf_stop(const NoEventData*);
    StateAction<SmoothPositionControl, NoEventData, &SmoothPositionControl::sf_stop> sa_stop;

    // STATE MAP
    virtual const StateMapRow* get_state_map() {
        static const StateMapRow STATE_MAP[] = {
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
    Clock clock_;
    Timer timer_;

    // HARDWARE
    Daq& daq_;
    Input<voltage>& analog_input_;
    Output<voltage>& analog_output_;
    Watchdog& watchdog_;
    MahiExoII meii_;

    // EXO PARAMETERS

    //char anatomical_joint_backdrive_ = { 0, 0, 0, 0, 1 }; // 1 = backdrivable, 0 = active

    

    //-------------------------------------------------------------------------
    // WAYPOINT TRACKING
    //-------------------------------------------------------------------------

    // WAYPOINTS
    
    // waypoint indexing variable
    int current_wp_idx_ = 0;
    
    // waypoints specifying rps parallel coordinates
    /*std::vector<std::vector<double>> wp_ = { { 0.09, 0.09, 0.09 },
                                    { 0.10, 0.10, 0.10 },
                                    { 0.10, 0.09, 0.11 },
                                    { 0.10, 0.10, 0.10 } };*/

    // waypoints specifying rps serial (anatomical) coordinates
    /*std::vector<std::vector<double>> wp_ = { { 0.0 * DEG2RAD, 0.0 * DEG2RAD, 0.09 },
                                    { 0.0 * DEG2RAD, 0.0 * DEG2RAD, 0.10 },
                                    { 10.0 * DEG2RAD, 0.0 * DEG2RAD, 0.10 },
                                    { 0.0 * DEG2RAD, 0.0 * DEG2RAD, 0.10 },
                                    { -10.0 * DEG2RAD, 0.0 * DEG2RAD, 0.10 },
                                    { 0.0 * DEG2RAD, 0.0 * DEG2RAD, 0.10 },
                                    { 0.0 * DEG2RAD, 10.0 * DEG2RAD, 0.10 },
                                    { 0.0 * DEG2RAD, 0.0 * DEG2RAD, 0.10 },
                                    { 0.0 * DEG2RAD, -10.0 * DEG2RAD, 0.10 },
                                    { 0.0 * DEG2RAD, 0.0 * DEG2RAD, 0.10 } };*/

    // waypoints specifying anatomical coordinates
    std::vector<std::vector<double>> wp_ = { { -35.0 * DEG2RAD, 0.0 * DEG2RAD, 0.0 * DEG2RAD, 0.0 * DEG2RAD, 0.10 },
                                    { -35.0 * DEG2RAD, 0.0 * DEG2RAD, 10.0 * DEG2RAD, 0.0 * DEG2RAD, 0.10 },
                                    { -35.0 * DEG2RAD, 0.0 * DEG2RAD, 0.0 * DEG2RAD, 0.0 * DEG2RAD, 0.10 },
                                    { -35.0 * DEG2RAD, 0.0 * DEG2RAD, -10.0 * DEG2RAD, 0.0 * DEG2RAD, 0.10 },
                                    { -35.0 * DEG2RAD, 0.0 * DEG2RAD, 0.0 * DEG2RAD, 0.0 * DEG2RAD, 0.10 },
                                    { -35.0 * DEG2RAD, 0.0 * DEG2RAD, 0.0 * DEG2RAD, 10.0 * DEG2RAD, 0.10 },
                                    { -35.0 * DEG2RAD, 0.0 * DEG2RAD, 0.0 * DEG2RAD, 0.0 * DEG2RAD, 0.10 },
                                    { -35.0 * DEG2RAD, 0.0 * DEG2RAD, 0.0 * DEG2RAD, -10.0 * DEG2RAD, 0.10 },
                                    { -35.0 * DEG2RAD, 0.0 * DEG2RAD, 0.0 * DEG2RAD, 0.0 * DEG2RAD, 0.10 } };

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
    MelShare pos_share_;
    MelShare vel_share_;
    MelShare torque_share_;

};
