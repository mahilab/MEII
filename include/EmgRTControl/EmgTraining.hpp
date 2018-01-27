#pragma once
#include "MEL/Utility/StateMachine.hpp"
#include "MEL/Daq/Quanser/Q8Usb.hpp"
#include "MEL/Exoskeletons/MahiExoII/MahiExoIIEmgFrc.hpp"
#include "MEL/Communications/Windows/MelShare.hpp"
#include "MEL/Math/Filter.hpp"
#include "MEL/Utility/Clock.hpp"
#include "mel_util.h"
#include "mahiexoii_util.h"
#include "GuiFlag.h"
#include <boost/circular_buffer.hpp>
#include "MEL/Math/Constants.hpp"
#include "MEL\Exoskeletons\MahiExoII\MahiExoIIEmg.hpp"

using namespace mel;

class EmgRTControlData : public EventData {

public:

};

class EmgTraining : public StateMachine {

public:

    //---------------------------------------------------------------------
    // CONSTRUCTOR(S) / DESTRUCTOR(S)
    //---------------------------------------------------------------------

    //EmgTraining(Clock& clock, Daq* q8_emg, Daq* q8_ati, MahiExoIIEmgFrc& meii, GuiFlag& gui_flag, int input_mode);
    EmgTraining(Clock& clock, Daq* q8_emg, MahiExoIIEmg& meii, GuiFlag& gui_flag, int input_mode);

private:
    
    //-------------------------------------------------------------------------
    // STATE MACHINE SETUP
    //-------------------------------------------------------------------------

    // STATES
    enum States {
        ST_INIT,
        ST_TO_CENTER,
        ST_HOLD_CENTER,
        ST_PRESENT_TARGET,
        ST_PROCESS_EMG,
        ST_FINISH,
        ST_STOP,
        ST_NUM_STATES
    };

    // STATE FUNCTIONS
    void sf_init(const NoEventData*);
    StateAction<EmgTraining, NoEventData, &EmgTraining::sf_init> sa_init;

    void sf_to_center(const NoEventData*);
    StateAction<EmgTraining, NoEventData, &EmgTraining::sf_to_center> sa_to_center;

    void sf_hold_center(const NoEventData*);
    StateAction<EmgTraining, NoEventData, &EmgTraining::sf_hold_center> sa_hold_center;

    void sf_present_target(const NoEventData*);
    StateAction<EmgTraining, NoEventData, &EmgTraining::sf_present_target> sa_present_target;

    void sf_process_emg(const NoEventData*);
    StateAction<EmgTraining, NoEventData, &EmgTraining::sf_process_emg> sa_process_emg;

    void sf_finish(const NoEventData*);
    StateAction<EmgTraining, NoEventData, &EmgTraining::sf_finish> sa_finish;

    void sf_stop(const NoEventData*);
    StateAction<EmgTraining, NoEventData, &EmgTraining::sf_stop> sa_stop;

    // STATE MAP
    virtual const StateMapRow* get_state_map() {
        static const StateMapRow STATE_MAP[] = {
            &sa_init,
            &sa_to_center,
            &sa_hold_center,
            &sa_present_target,
            &sa_process_emg,
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
    // EXPERIMENT SETUP
    //-------------------------------------------------------------------------

    int current_target_ = 0;
    std::vector<int> target_sequence_ = { 1,2,1,2,1,2 };
    char target_check_joint_ = { 1,1,1,1,1 };
    std::vector<double> target_tol_ = { 1.0 *DEG2RAD, 1.0 * DEG2RAD, 1.0 * DEG2RAD, 1.0 *DEG2RAD, 0.01 };
    double hold_center_time_ = 2.0; // time to hold at center target [s]
    double force_mag_goal_ = 1000; // [N^2]
    double force_mag_tol_ = 100; // [N]
    double force_mag_dwell_time_ = 1.0; // [s]
    double force_mag_maintained_ = 0; // [s]
    double force_mag_time_now_ = 0;
    double force_mag_time_last_ = 0;

    //-------------------------------------------------------------------------
    // EXPERIMENT COMPONENTS
    //-------------------------------------------------------------------------

    // GUI FLAGS
    GuiFlag& gui_flag_;

    // HARDWARE CLOCK
    Clock clock_;

    // HARDWARE
    Daq* q8_emg_;
    Daq* q8_ati_;
    MahiExoIIEmg meii_;
    //MahiExoIIEmgFrc meii_;

    // MEII PARAMETERS
    int8_vec backdrive_ = { 1,1,1,1,1 };

    // MEII POSITION CONTROL
    std::vector<double> kp_ = { 35, 7, 25, 30, 0 };
    std::vector<double> kd_ = { 0.25, 0.06, 0.05, 0.08, 0 };
    std::vector<double> init_pos_ = std::vector<double>(5, 0);
    std::vector<double> goal_pos_ = std::vector<double>(5, 0);
    double init_time_ = 0.0;
    std::vector<double> speed_ = { 0.25, 0.25, 0.125, 0.125, 0.0125 };
    std::vector<double> x_ref_ = std::vector<double>(5, 0);
    std::vector<double> set_points_ = std::vector<double>(5, 0);
    std::vector<double> new_torques_ = std::vector<double>(5, 0);
    MelShare pos_data_);
    std::vector<double> data_p_ = std::vector<double>(5, 0);

    // FORCE SENSING
    std::vector<double> wrist_forces_ = std::vector<double>(6, 0);
    std::vector<double> wrist_forces_filt_ = std::vector<double>(6, 0);
    Filter multi_lpf_ = Filter(6, 4, { 0.009735570656078, -0.032135367809242, 0.045449986329302, -0.032135367809242, 0.009735570656078 }, { 1.000000000000000, -3.572942808701423, 4.807914652718555, -2.886325158284144, 0.652003706289986 });

    // EMG SENSING
    int num_channels_ = 8; 
    std::vector<double> emg_voltages_ = std::vector<double>(num_channels_, 0);
    struct EmgDataBuffer {
        EmgDataBuffer(size_t num_channels, size_t length) :
            num_channels_(num_channels),
            length_(length)
        {
            for (int i = 0; i < num_channels_; ++i) {
                data_buffer_.push_back(boost::circular_buffer<double>(length_));
            }
        }
        void push_back(std::vector<double> data_vec) {
            if (data_vec.size() != num_channels_) {
                print("ERROR: Incorrect number of rows when calling EmgDataBuffer::push_back().");
            }
            for (int i = 0; i < num_channels_; ++i) {
                data_buffer_[i].push_back(data_vec[i]);
            }
        }
        std::vector<double> at(int index) {
            std::vector<double> data_vec;
            for (int i = 0; i < num_channels_; ++i) {
                data_vec.push_back(data_buffer_[i][index]);
            }
            return data_vec;
        }
        std::vector<double> get_channel(int index) {
            std::vector<double> channel_vec;
            for (int i = 0; i < length_; ++i) {
                channel_vec.push_back(data_buffer_[index][i]);
            }
            return channel_vec;
        }
        size_t num_channels_;
        size_t length_;
        std::vector<boost::circular_buffer<double>> data_buffer_;
    };   
    EmgDataBuffer emg_data_buffer_ = EmgDataBuffer(num_channels_,200);

    // EMG FEATURE EXTRACTION
    int num_features_ = 9;
    std::vector<double> feature_vec_ = std::vector<double>(num_features_*num_channels_, 0);
    std::vector<double> feature_extract(EmgDataBuffer& emg_data_buffer);
    double rms_feature_extract(boost::circular_buffer<double> emg_channel_buffer);
    double mav_feature_extract(boost::circular_buffer<double> emg_channel_buffer);
    double wl_feature_extract(boost::circular_buffer<double> emg_channel_buffer);
    double zc_feature_extract(boost::circular_buffer<double> emg_channel_buffer);
    double ssc_feature_extract(boost::circular_buffer<double> emg_channel_buffer);
    void ar4_feature_extract(std::vector<double>& coeffs, const std::vector<double>& emg_channel_buffer);
    
    // STATE TRANSITION EVENTS
    bool target_reached_ = false;
    bool hold_center_time_reached_ = false;
    bool force_mag_reached_ = false;
    bool emg_data_processed_ = false;
    bool end_of_target_sequence_ = false;
    bool check_target_reached(std::vector<double> goal_pos, std::vector<double> current_pos, char check_joint, bool print_output = false);
    bool check_wait_time_reached(double wait_time, double init_time, double current_time);
    bool check_force_mag_reached(double force_mag_goal, double force_mag);

    // UNITY INPUT/OUTPUT
    int scene_num_share = 0;
    MelShare scene_num_;
    int target_share_ = 0;
    MelShare target_;
    double force_share_ = 0;
    MelShare force_mag_;

    // MELSCOPE VARIABLES
    MelShare pos_share_;
    MelShare vel_share_;
    MelShare emg_share_;

};