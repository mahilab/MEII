#pragma once
#include "StateMachine.h"
#include "Q8Usb.h"
#include "MahiExoIIEmg.h"
#include "MelShare.h"
#include "Filter.h"
#include "Clock.h"
#include "mel_util.h"
#include "mahiexoii_util.h"
#include "ExternalApp.h"


using namespace mel;

class EmgRTControlData : public util::EventData {

public:

};

class EmgRTControl : public util::StateMachine {

public:

    //---------------------------------------------------------------------
    // CONSTRUCTOR(S) / DESTRUCTOR(S)
    //---------------------------------------------------------------------

    EmgRTControl(util::Clock& clock, core::Daq* daq, exo::MahiExoIIEmg& meii);

private:

    //-------------------------------------------------------------------------
    // STATE MACHINE SETUP
    //-------------------------------------------------------------------------

    // STATES
    enum States {
        ST_WAIT_FOR_GUI,
        ST_INIT,
        ST_TRANSPARENT,
        ST_INIT_RPS,
        ST_TO_CENTER,
        ST_HOLD_CENTER,
        ST_PRESENT_TARGET,
        ST_PROCESS_EMG,
        ST_TRAIN_CLASSIFIER,
        ST_FINISH,
        ST_STOP,
        ST_NUM_STATES
    };

    // STATE FUNCTIONS
    void sf_wait_for_gui(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_wait_for_gui> sa_wait_for_gui;

    void sf_init(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_init> sa_init;

    void sf_transparent(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_transparent> sa_transparent;

    void sf_init_rps(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_init_rps> sa_init_rps;

    void sf_to_center(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_to_center> sa_to_center;

    void sf_hold_center(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_hold_center> sa_hold_center;

    void sf_present_target(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_present_target> sa_present_target;

    void sf_process_emg(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_process_emg> sa_process_emg;

    void sf_train_classifier(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_train_classifier> sa_train_classifier;

    void sf_finish(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_finish> sa_finish;

    void sf_stop(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_stop> sa_stop;

    // STATE MAP
    virtual const util::StateMapRow* get_state_map() {
        static const util::StateMapRow STATE_MAP[] = {
            &sa_wait_for_gui,
            &sa_init,
            &sa_transparent,
            &sa_init_rps,
            &sa_to_center,
            &sa_hold_center,
            &sa_present_target,
            &sa_process_emg,
            &sa_train_classifier,
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
    // EXPERIMENT SETUP & COMPONENTS
    //-------------------------------------------------------------------------

    // UNITY GAME
    util::ExternalApp game = mel::util::ExternalApp("2D_targets", "C:\\Users\\Ted\\GitHub\\MEII\\Exo Visualization\\Builds\\Exo_Vis_Build_1.exe");

    // HARDWARE CLOCK
    util::Clock clock_;

    // HARDWARE
    core::Daq* daq_;
    exo::MahiExoIIEmg meii_;

    // EXPERIMENT CONDITION
    int dof_ = 0; // 0-3 is single-dof; 4-5 is multi-dof
    int condition_ = 0; // 0 = training; 1 = blind testing; 2 = full testing

    // EXO PARAMETERS
    bool rps_backdrive_ = false; // 1 = backdrivable, 0 = active
    char_vec anatomical_joint_backdrive_ = { 0, 0, 1, 1, 1 }; // 1 = backdrivable, 0 = active
    const double_vec robot_joint_speed_ = { 0.25, 0.25, 0.015, 0.015, 0.015 }; // constant speed at which robot joint reference trajectories are interpolated
    const double_vec anatomical_joint_speed_ = { 0.25, 0.25, 0.125, 0.125, 0.0125 }; // constant speed at which anatomical joint reference trajectories are interpolated

    // RPS MECHANISM SAFE INITIALIZATION PARAMETERS
    const double_vec rps_init_pos_ = { 0.10, 0.10, 0.10 };
    const double rps_pos_tol_ = 0.005; // tolerance in [m] for checking that initialization position is reached

    // RANDOMIZED TARGET LISTS
    int current_target_idx_ = 0;
    std::vector<int> target_sequence_;

    // EXPERIMENT TIMING PARAMETERS
    double st_enter_time_;
    const double init_transparent_time_ = 2.0; // [s]
    const double hold_center_time_ = 1.0; // time to hold at center target [s]

    // TEMPORARY WAYPOINT CONTAINERS
    double_vec start_pos_ = double_vec(5, 0.0);
    double_vec goal_pos_ = double_vec(5, 0.0);

    // PREDEFINED WAYPOINTS
    const double_vec center_pos_ = { -35 * math::DEG2RAD, 0 * math::DEG2RAD, 0 * math::DEG2RAD, 0 * math::DEG2RAD,  0.09 }; // anatomical joint positions

    // TARGET CHECKING PARAMETERS
    char_vec target_check_joint_ = { 1, 1, 0, 0, 0 };
    const double_vec target_anatomical_pos_tol_ = { 1.0 * math::DEG2RAD, 1.0 * math::DEG2RAD, 1.0 * math::DEG2RAD, 1.0 * math::DEG2RAD, 0.01 };

    // FORCE MEASUREMENT PARAMETERS
    const double_vec force_dof_scale_ = { 3, 0.5, 0.5, 0.5 }; // [Nm]
    const std::vector<char_vec> target_dir_ = {
        { 1, -1 },
        { -1, 1 },
        { 1, -1 },
        { 1, -1 },
        { 1, 1, -1, -1 },
        { -1, 1, -1, 1 },
        { 1, 1, -1, -1 },
        { 1, -1, 1, -1 } };
    const double_vec gravity_offsets_ = { 1.0, 0.0, 0.0, 0.1, 0.0}; // for all 5 anatomical dofs
    double force_mag_goal_ = 3000.0; //
    double force_mag_tol_ = 100.0; //
    double force_mag_dwell_time_ = 1.0; // [s]
    double force_mag_maintained_ = 0.0; // [s]
    double force_mag_time_now_ = 0.0;
    double force_mag_time_last_ = 0.0;

    

    // UNITY INPUT/OUTPUT
    int scene_num_ = 0;
    int target_num_ = 0;
    double force_mag_ = 0.0;
    comm::MelShare scene_num_share_ = comm::MelShare("scene_num");
    comm::MelShare target_num_share_ = comm::MelShare("target");
    comm::MelShare force_mag_share_ = comm::MelShare("force_mag");
    
    // EMG SENSING PARAMETERS
    static const int num_emg_channels_ = 8;

    // EMG FEATURE EXTRACTION PARAMETERS
    static const int num_features_ = 9;
   
    // STATE TRANSITION EVENT VARIABLES
    bool scene_selected_ = false;
    bool init_transparent_time_reached_ = false;
    bool rps_init_ = false;
    bool target_reached_ = false;
    bool hold_center_time_reached_ = false;
    bool force_mag_reached_ = false;
    bool emg_data_processed_ = false;
    bool end_of_target_sequence_ = false;

    double_vec emg_voltages_ = double_vec(num_emg_channels_, 0);
    double_vec feature_vec_ = double_vec(num_features_*num_emg_channels_, 0);
    std::array<double, num_features_*num_emg_channels_> feature_array_;
    exo::MahiExoIIEmg::EmgDataBuffer emg_data_buffer_ = exo::MahiExoIIEmg::EmgDataBuffer(num_emg_channels_, 200);

    // UTILITY FUNCTIONS
    void set_experiment_conditions(int scene_num);
    bool check_rps_init(double_vec rps_init_pos, double_vec rps_current_pos, char_vec check_joint, bool print_output = false) const;
    bool check_target_reached(double_vec goal_pos, double_vec current_pos, char_vec check_joint, double_vec target_tol, bool print_output = false) const;
    bool check_wait_time_reached(double wait_time, double init_time, double current_time) const;
    double measure_task_force(double_vec commanded_torques, int target_num, int dof, int condition) const;
    bool check_force_mag_reached(double force_mag_goal, double force_mag);

    // EMG FEATURE EXTRACTION FUNCTIONS
    double_vec feature_extract(exo::MahiExoIIEmg::EmgDataBuffer& emg_data_buffer);
    double rms_feature_extract(boost::circular_buffer<double> emg_channel_buffer);
    double mav_feature_extract(boost::circular_buffer<double> emg_channel_buffer);
    double wl_feature_extract(boost::circular_buffer<double> emg_channel_buffer);
    double zc_feature_extract(boost::circular_buffer<double> emg_channel_buffer);
    double ssc_feature_extract(boost::circular_buffer<double> emg_channel_buffer);
    void ar4_feature_extract(double_vec& coeffs, const double_vec& emg_channel_buffer);

    // EMG TRAINING DATA
    static const int num_class_ = 2;
    std::vector<std::array<double,num_features_*num_emg_channels_>> emg_training_data_;
    size_t N_train_ = target_sequence_.size();
    comm::MelShare trng_size_ = comm::MelShare("trng_size");
    comm::MelShare trng_share_ = comm::MelShare("trng_share", 4096);
    comm::MelShare label_share_ = comm::MelShare("label_share");
    comm::MelShare lda_coeff_ = comm::MelShare("LDA_coeff", 1024);
    comm::MelShare trng_size2_ = comm::MelShare("trng_size2");
    comm::MelShare feat_id_ = comm::MelShare("feat_id");
    

    // MELSCOPE VARIABLES
    comm::MelShare pos_share_ = comm::MelShare("pos_share");
    comm::MelShare vel_share_ = comm::MelShare("vel_share");
    comm::MelShare emg_share_ = comm::MelShare("emg_share");
    comm::MelShare torque_share_ = comm::MelShare("torque_share");

};