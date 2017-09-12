#pragma once
#include "StateMachine.h"
#include "Q8Usb.h"
#include "MahiExoIIEmg.h"
#include "MelShare.h"
#include "Filter.h"
#include "Clock.h"
#include "mel_util.h"
#include "mahiexoii_util.h"
#include "GuiFlag.h"
#include <boost/circular_buffer.hpp>

using namespace mel;

class EmgRTControlData : public util::EventData {

public:

};

class EmgRTControl : public util::StateMachine {

public:

    //---------------------------------------------------------------------
    // CONSTRUCTOR(S) / DESTRUCTOR(S)
    //---------------------------------------------------------------------

    EmgRTControl(util::Clock& clock, core::Daq* daq, exo::MahiExoIIEmg& meii, util::GuiFlag& gui_flag, int input_mode);

private:

    //-------------------------------------------------------------------------
    // STATE MACHINE SETUP
    //-------------------------------------------------------------------------

    // STATES
    enum States {
        ST_INIT,
        ST_TRANSPARENT,
        ST_TO_CENTER_PAR,
        ST_TO_CENTER_SER,
        ST_HOLD_CENTER,
        ST_PRESENT_TARGET,
        ST_PROCESS_EMG,
        ST_TRAIN_CLASSIFIER,
        ST_FINISH,
        ST_STOP,
        ST_NUM_STATES
    };

    // STATE FUNCTIONS
    void sf_init(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_init> sa_init;

    void sf_transparent(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_transparent> sa_transparent;

    void sf_to_center_par(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_to_center_par> sa_to_center_par;

    void sf_to_center_ser(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_to_center_ser> sa_to_center_ser;

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
            &sa_init,
            &sa_transparent,
            &sa_to_center_par,
            &sa_to_center_ser,
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
    int INPUT_MODE_;
    void wait_for_input();
    bool check_stop();
    bool stop_ = false;

    //-------------------------------------------------------------------------
    // EXPERIMENT SETUP & COMPONENTS
    //-------------------------------------------------------------------------

    int current_target_ = 0;
    double init_transparent_time_ = 2.0; // [s]
    std::vector<int> target_sequence_ = { 1, 2, 1, 2 };
    char_vec target_check_joint_ = { 1, 1, 1, 1, 1 };
    double_vec target_tol_par_ = { 1.0 * math::DEG2RAD, 1.0 * math::DEG2RAD, 5.0 * math::DEG2RAD, 5.0 * math::DEG2RAD, 0.05 };
    double_vec target_tol_ser_ = { 1.0 * math::DEG2RAD, 1.0 * math::DEG2RAD, 0.5 * math::DEG2RAD, 0.5 * math::DEG2RAD, 0.005 };
    double hold_center_time_ = 1.0; // time to hold at center target [s]
    double force_mag_goal_ = 1000.0; // [N^2]
    double force_mag_tol_ = 100.0; // [N]
    double force_mag_dwell_time_ = 1.0; // [s]
    double force_mag_maintained_ = 0.0; // [s]
    double force_mag_time_now_ = 0.0;
    double force_mag_time_last_ = 0.0;

    double_vec center_pos_ = { -35 * math::DEG2RAD, 0 * math::DEG2RAD, 0 * math::DEG2RAD, 0 * math::DEG2RAD,  0.09 }; // anatomical joint positions


    // GUI FLAGS
    util::GuiFlag& gui_flag_;

    // HARDWARE CLOCK
    util::Clock clock_;

    // HARDWARE
    core::Daq* daq_;
    exo::MahiExoIIEmg meii_;

    // EXO PARAMETERS
    int rps_control_mode_ = 0; // 0 = robot joint space (parallel), 1 = anatomical joint space (serial)
    char_vec robot_joint_backdrive_ = { 0, 0, 0, 0, 0 }; // 1 = backdrivable, 0 = active
    char_vec anatomical_joint_backdrive_ = { 0, 0, 0, 0, 1 }; // 1 = backdrivable, 0 = active
    double_vec speed_ = { 0.25, 0.25, 0.125, 0.125, 0.0125 };

    // SMOOTH REFERENCE TRAJECTORY
    double_vec ref_pos_ = double_vec(5, 0.0);
    double_vec q_ser_ref_ = double_vec(3, 0.0);
    double_vec q_par_ref_ = double_vec(3, 0.0);
    double init_time_ = 0.0;
    double_vec start_pos_ = double_vec(5, 0.0);
    double_vec goal_pos_ = double_vec(5, 0.0);

    // ACTUATOR CONTROL
    double_vec pd_torques_ = double_vec(5, 0.0);
    double_vec commanded_torques_ = double_vec(5, 0.0);

    // EMG SENSING
    static const int num_emg_channels_ = 8;
    double_vec emg_voltages_ = double_vec(num_emg_channels_, 0);
    struct EmgDataBuffer {
        EmgDataBuffer(size_t num_channels, size_t length) :
            num_channels_(num_channels),
            length_(length)
        {
            for (size_t i = 0; i < num_channels_; ++i) {
                data_buffer_.push_back(boost::circular_buffer<double>(length_,0.0));
            }
        }
        void push_back(double_vec data_vec) {
            if (data_vec.size() != num_channels_) {
                util::print("ERROR: Incorrect number of rows when calling EmgDataBuffer::push_back().");
            }
            for (int i = 0; i < num_channels_; ++i) {
                data_buffer_[i].push_back(data_vec[i]);
            }
        }
        double_vec at(int index) {
            double_vec data_vec;
            for (int i = 0; i < num_channels_; ++i) {
                data_vec.push_back(data_buffer_[i][index]);
            }
            return data_vec;
        }
        double_vec get_channel(int index) {
            double_vec channel_vec;
            for (int i = 0; i < length_; ++i) {
                channel_vec.push_back(data_buffer_[index][i]);
            }
            return channel_vec;
        }
        size_t num_channels_;
        size_t length_;
        std::vector<boost::circular_buffer<double>> data_buffer_;
    };
    EmgDataBuffer emg_data_buffer_ = EmgDataBuffer(num_emg_channels_, 200);

    // EMG FEATURE EXTRACTION
    static const int num_features_ = 9;
    double_vec feature_vec_ = double_vec(num_features_*num_emg_channels_, 0);
    std::array<double, num_features_*num_emg_channels_> feature_array_;
    double_vec feature_extract(EmgDataBuffer& emg_data_buffer);
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
    comm::MelShare trng_share_ = comm::MelShare("trng_share");
    comm::MelShare label_share_ = comm::MelShare("label_share");
    comm::MelShare lda_coeff_ = comm::MelShare("LDA_coeff");
    comm::MelShare trng_size2_ = comm::MelShare("trng_size2");
    comm::MelShare feat_id_ = comm::MelShare("feat_id");
    
    // STATE TRANSITION EVENTS
    bool init_transparent_time_reached_ = false;
    bool target_reached_ = false;
    bool hold_center_time_reached_ = false;
    bool force_mag_reached_ = false;
    bool emg_data_processed_ = false;
    bool end_of_target_sequence_ = false;

    // USEFUL STATE VARIABLES
    double st_enter_time_;

    // UTILITY FUNCTIONS
    bool check_target_reached(double_vec goal_pos, double_vec current_pos, char_vec check_joint, double_vec target_tol, bool print_output = false);
    bool check_wait_time_reached(double wait_time, double init_time, double current_time);
    bool check_force_mag_reached(double force_mag_goal, double force_mag);

    // UNITY INPUT/OUTPUT
    int scene_num_share = 0;
    comm::MelShare scene_num_ = comm::MelShare("scene_num");
    int target_share_ = 0;
    comm::MelShare target_ = comm::MelShare("target");
    double force_share_ = 0.0;
    comm::MelShare force_mag_ = comm::MelShare("force_mag");

    // MELSCOPE VARIABLES
    comm::MelShare pos_share_ = comm::MelShare("pos_share");
    comm::MelShare vel_share_ = comm::MelShare("vel_share");
    comm::MelShare emg_share_ = comm::MelShare("emg_share");
    comm::MelShare torque_share_ = comm::MelShare("torque_share");

};