#pragma once
#include "MEL/Utility/StateMachine.hpp""
#include "MEL/Daq/Quanser/Q8Usb.hpp"
#include "MEL/Exoskeletons/MahiExoII/MahiExoIIEmg.hpp"
#include "MEL/Communications/Windows/MelShare.hpp"
#include "MEL/Utility/Clock.hpp"
#include "mel_util.h"
#include "mahiexoii_util.h"
#include "MEL/Utility/Windows/ExternalApp.hpp"
#include "MEL/Utility/Timer.hpp"


using namespace mel;

class EmgRTControlData : public EventData {

public:

};

class EmgRTControl : public StateMachine {

public:

    //---------------------------------------------------------------------
    // CONSTRUCTOR(S) / DESTRUCTOR(S)
    //---------------------------------------------------------------------

    EmgRTControl(Clock& clock, Timer timer, Daq& daq, Input<voltage>& analog_input, Output<voltage>& analog_ouput, Watchdog& watchdog, MahiExoIIEmg& meii);

private:

    //-------------------------------------------------------------------------
    // STATE MACHINE SETUP
    //-------------------------------------------------------------------------

    // STATES
    enum States {
        ST_WAIT_FOR_GUI,
        ST_INIT,
        ST_BACKDRIVE,
        ST_INIT_RPS,
        ST_TO_CENTER,
        ST_HOLD_CENTER,
        ST_HOLD_FOR_INPUT,
        ST_PRESENT_TARGET,
        ST_TO_TARGET,
        ST_HOLD_TARGET,
        ST_FINISH,
        ST_STOP,
        ST_FAULT_STOP,
        ST_NUM_STATES
    };

    // STATE FUNCTIONS
    void sf_wait_for_gui(const NoEventData*);
    StateAction<EmgRTControl, NoEventData, &EmgRTControl::sf_wait_for_gui> sa_wait_for_gui;

    void sf_init(const NoEventData*);
    StateAction<EmgRTControl, NoEventData, &EmgRTControl::sf_init> sa_init;

    void sf_backdrive(const NoEventData*);
    StateAction<EmgRTControl, NoEventData, &EmgRTControl::sf_backdrive> sa_backdrive;

    void sf_init_rps(const NoEventData*);
    StateAction<EmgRTControl, NoEventData, &EmgRTControl::sf_init_rps> sa_init_rps;

    void sf_to_center(const NoEventData*);
    StateAction<EmgRTControl, NoEventData, &EmgRTControl::sf_to_center> sa_to_center;

    void sf_hold_center(const NoEventData*);
    StateAction<EmgRTControl, NoEventData, &EmgRTControl::sf_hold_center> sa_hold_center;

    void sf_hold_for_input(const NoEventData*);
    StateAction<EmgRTControl, NoEventData, &EmgRTControl::sf_hold_for_input> sa_hold_for_input;

    void sf_present_target(const NoEventData*);
    StateAction<EmgRTControl, NoEventData, &EmgRTControl::sf_present_target> sa_present_target;

    void sf_to_target(const NoEventData*);
    StateAction<EmgRTControl, NoEventData, &EmgRTControl::sf_to_target> sa_to_target;

    void sf_hold_target(const NoEventData*);
    StateAction<EmgRTControl, NoEventData, &EmgRTControl::sf_hold_target> sa_hold_target;

    void sf_finish(const NoEventData*);
    StateAction<EmgRTControl, NoEventData, &EmgRTControl::sf_finish> sa_finish;

    void sf_stop(const NoEventData*);
    StateAction<EmgRTControl, NoEventData, &EmgRTControl::sf_stop> sa_stop;

    void sf_fault_stop(const NoEventData*);
    StateAction<EmgRTControl, NoEventData, &EmgRTControl::sf_fault_stop> sa_fault_stop;

    // STATE MAP
    virtual const StateMapRow* get_state_map() {
        static const StateMapRow STATE_MAP[] = {
            &sa_wait_for_gui,
            &sa_init,
            &sa_backdrive,
            &sa_init_rps,
            &sa_to_center,
            &sa_hold_center,
            &sa_hold_for_input,
            &sa_present_target,
            &sa_to_target,
            &sa_hold_target,
            &sa_finish,
            &sa_stop,
            &sa_fault_stop,
        };
        return &STATE_MAP[0];
    }

    //-------------------------------------------------------------------------
    // PRIVATE VARIABLES
    //-------------------------------------------------------------------------

    // OPTIONAL MODES FOR DEBUGGING
    // (set all to false for normal operation)
    bool virtual_exo_ = false; // when true, prevents computing joint kinematics, computing and setting joint torques, and all writing to the DAQ; also prevents any checks dependent on exo motion
    bool virtual_emg_ = true; // when true, prevents any checks based on emg readings
    bool scope_mode_ = true;
    bool emg_signal_check_ = false;
    bool find_neutral_position_ = false;

    // SUBJECT/CONDITION
    int subject_number_ = 0;
    std::vector<std::string> hand_defs = { "L","R" };
    int hand_num_ = 1; // 0 or 1 for Left or Right arm of the user
    std::string hand_def_ = hand_defs[hand_num_];
    int dof_; // 0-3 is single-dof; 4-5 is multi-dof
    int num_classes_;
    int condition_; // 0 = calibration; 1 = training; 2 = blind testing; 3 = full testing

    // STRING NAMES
    std::vector<std::string> str_conditions_long_ = { "Calibration", "Training", "Blind Testing", "Full Testing" };
    std::vector<std::string> str_conditions_ = { "cal", "trng", "blind", "full" };
    std::vector<std::string> str_dofs_long_ = { "Elbow F/E Single-DoF", "Forearm P/S Single-Dof", "Wrist F/E Single-DoF", "Wrist R/U Single-DoF", "Elbow F/E & Forearm P/S Multi-DoF", "Wrist F/E & Wrist R/U Multi-DoF" };
    std::vector<std::string> str_dofs_ = { "EFE", "FPS", "WFE", "WRU", "ELFM", "WMLT" };
    std::vector<std::string> str_states_ = { "ST_WAIT_FOR_GUI", "ST_INIT", "ST_BACKDRIVE", "ST_INIT_RPS", "ST_TO_CENTER", "ST_HOLD_CENTER", "ST_HOLD_FOR_INPUT", "ST_PRESENT_TARGET", "ST_TO_TARGET", "ST_HOLD_TARGET", "ST_FINISH", "ST_STOP", "ST_FAULT_STOP" };
    
    // FILE NAMES & DIRECTORIES
    std::string program_directory_ = "C:\\Users\\Ted\\GitHub\\MEII\\bin";
    std::string project_directory_ = "C:\\Users\\Ted\\GitHub\\MEII\\EmgRTControl";
    std::string subject_directory_;
    std::string subject_dof_directory_;
    std::string emg_active_classifier_filename_;
    std::string training_data_filename_;
    std::string emg_dir_classifier_filename_;

    // UNITY GAME
    ExternalApp game = ExternalApp("2D_targets", "C:\\Users\\Ted\\GitHub\\MEII\\Exo Visualization\\Builds\\Exo_Vis_Build_1.exe");

    // HARDWARE CLOCK
    Clock clock_;
    Timer timer_;

    // HARDWARE
    Daq& daq_;
    Input<voltage>& analog_input_;
    Output<voltage>& analog_output_;
    Watchdog& watchdog_;
    MahiExoIIEmg meii_;

    // PREDEFINED TARGETS
    const std::vector<double> center_pos_ = { -35 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD,  0.09 }; // anatomical joint positions
    const std::vector<std::vector<std::vector<std::vector<double>>>> single_dof_targets_ =     // left arm
                                                                                    { { { {  -5 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 }, { -65 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 } }, // elbow f/e : up, down
                                                                                        { { -35 * DEG2RAD, -30 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 }, { -35 * DEG2RAD,  30 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 } }, // forearm p/s : right, left
                                                                                        { { -35 * DEG2RAD,   0 * DEG2RAD, -15 * DEG2RAD,   0 * DEG2RAD,  0.09 }, { -35 * DEG2RAD,   0 * DEG2RAD,  15 * DEG2RAD,   0 * DEG2RAD,  0.09 } }, // wrist f/e : right, left
                                                                                        { { -35 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  15 * DEG2RAD,  0.09 }, { -35 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD, -15 * DEG2RAD,  0.09 } } }, // wrist r/u : up, down
                                                                                      
                                                                                      // right arm
                                                                                      { { {  -5 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 }, { -65 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 } }, // elbow f/e : up, down
                                                                                        { { -35 * DEG2RAD,  30 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 }, { -35 * DEG2RAD, -30 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 } }, // forearm p/s : left, right
                                                                                        { { -35 * DEG2RAD,   0 * DEG2RAD,  15 * DEG2RAD,   0 * DEG2RAD,  0.09 }, { -35 * DEG2RAD,   0 * DEG2RAD, -15 * DEG2RAD,   0 * DEG2RAD,  0.09 } }, // wrist f/e : left, right
                                                                                        { { -35 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  15 * DEG2RAD,  0.09 }, { -35 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD, -15 * DEG2RAD,  0.09 } } } }; // wrist r/u : up, down

    const std::vector<std::vector<std::vector<std::vector<double>>>>  multi_dof_targets_ =     // left arm
                                                                                    { { { {  -5 * DEG2RAD, -30 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 }, {  -5 * DEG2RAD,  30 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 }, { -65 * DEG2RAD, -30 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 }, { -65 * DEG2RAD,  30 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 } }, // elbow f/e & forearm p/s
                                                                                        { { -35 * DEG2RAD,   0 * DEG2RAD, -15 * DEG2RAD,  15 * DEG2RAD,  0.09 }, { -35 * DEG2RAD,   0 * DEG2RAD,  15 * DEG2RAD,  15 * DEG2RAD,  0.09 }, { -35 * DEG2RAD,   0 * DEG2RAD, -15 * DEG2RAD, -15 * DEG2RAD,  0.09 }, { -35 * DEG2RAD,   0 * DEG2RAD,  15 * DEG2RAD, -15 * DEG2RAD,  0.09 } } }, // wrist f/e & wrist r/u

                                                                                      // right arm
                                                                                      { { {  -5 * DEG2RAD,  30 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 }, {  -5 * DEG2RAD, -30 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 }, { -65 * DEG2RAD,  30 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 }, { -65 * DEG2RAD, -30 * DEG2RAD,   0 * DEG2RAD,   0 * DEG2RAD,  0.09 } }, // elbow f/e & forearm p/s
                                                                                        { { -35 * DEG2RAD,   0 * DEG2RAD,  15 * DEG2RAD,  15 * DEG2RAD,  0.09 }, { -35 * DEG2RAD,   0 * DEG2RAD, -15 * DEG2RAD,  15 * DEG2RAD,  0.09 }, { -35 * DEG2RAD,   0 * DEG2RAD,  15 * DEG2RAD, -15 * DEG2RAD,  0.09 }, { -35 * DEG2RAD,   0 * DEG2RAD, -15 * DEG2RAD, -15 * DEG2RAD,  0.09 } } } }; // wrist f/e & wrist r/u

    // EXPERIMENT TIMING PARAMETERS
    double init_backdrive_time_ = 2.0; // [s] time to be in backdrive mode initially
    double hold_center_time_ = 1.0; // [s] time to hold at center target
    double hold_target_time_ = 1.0; // [s] time to hold at target
    double wait_mvc_time_ = 0.5; // [s] time to wait during MVC
    double detection_expire_time_ = 10.0; // [s] time after which failure to detect active state is marked as misclassification

    // TASK FORCE MEASUREMENT
    int task_force_measurement_mode_ = 0; // 0 = commanded torques, dof/direction specific; 1 = commanded torques, dof specific, direction agnostic; 2 = commanded torques, dof/direction agnostic
    double def_efe_trq = 3.00;
    double def_fps_trq = 0.50;
    double def_wfe_trq = 0.50;
    double def_wru_trq = 0.50;
    const std::vector<std::vector<std::vector<double>>> force_dof_scale_ = { { { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq } }, // elbow f/e single-dof
                                                                    { { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq } }, // forearm p/s single-dof
                                                                    { { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq } }, // wrist f/e single-dof
                                                                    { { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq } }, // wrist r/u single-dof
                                                                    { { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq } }, // elbow f/e & forearm p/s multi-dof
                                                                    { { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq } } }; // wrist f/e & wrist r/u multi-dof
    const std::vector<char> target_dir_L_ = {
        { 1, -1 },
        { -1, 1 },
        { -1, 1 },
        { 1, -1 },
        { 1, 1, -1, -1 },
        { -1, 1, -1, 1 },
        { 1, 1, -1, -1 },
        { 1, -1, 1, -1 } };
    const std::vector<char> target_dir_R_ = {
        { 1, -1 },
        { 1, -1 },
        { 1, -1 },
        { 1, -1 },
        { 1, 1, -1, -1 },
        { 1, -1, 1, -1 },
        { 1, 1, -1, -1 },
        { -1, 1, -1, 1 } };
    const std::vector<double> gravity_offsets_ = { 0.3, 0.0, 0.0, -0.35, 0.0 }; // for all 5 anatomical dofs, due to counterweight elbow can be under 'negative gravity'
    double force_mag_goal_ = 3050.0; // 
    double force_mag_tol_ = 300.0; //
    double force_mag_dwell_time_ = 1.0; // [s]

    // FROCE MAGNITUDE CHECKING ALGORITHM VARIABLES
    double force_mag_maintained_ = 0.0;
    double force_mag_time_now_ = 0.0;
    double force_mag_time_last_ = 0.0;

    // UNITY INPUT/OUTPUT
    int SCENE_NUM_ = 0; // capitalized because it is the external data provided from GUI and should be handled carefully!
    int viz_target_num_ = 0;
    MelShare scene_num_share_;
    MelShare viz_target_num_share_;
    MelShare force_mag_share_;
    MelShare hand_select_;

    // EMG SENSING AND FEATURE EXTRACTION PARAMETERS
    static const int num_emg_channels_ = 8;
    static const int num_features_ = 9;
    static const int emg_calibration_window_length_ = 500;
    static const int emg_classification_window_length_ = 200;

    // STATE TRANSITION EVENT VARIABLES
    bool end_of_label_sequence_ = true;
    bool auto_stop_ = false;
    bool manual_stop_ = false;

    // TEMPORARY EMG DATA CONTAINERS
    std::vector<double> emg_voltages_ = std::vector<double>(meii_.N_emg_);
    std::vector<double> filtered_emg_voltages_ = std::vector<double>(meii_.N_emg_);
    exo::MahiExoIIEmg::EmgDataBuffer emg_calibration_data_buffer_ = exo::MahiExoIIEmg::EmgDataBuffer(meii_.N_emg_, emg_calibration_window_length_);
    exo::MahiExoIIEmg::EmgDataBuffer emg_classification_data_buffer_ = exo::MahiExoIIEmg::EmgDataBuffer(meii_.N_emg_, emg_classification_window_length_);

    // EMG CALIBRATION
    std::vector<std::vector<double>> calibration_data_;
    std::vector<std::vector<double>> tkeo_rest_data_;
    std::vector<std::vector<std::vector<double>>> tkeo_active_data_;
    Eigen::MatrixXd tkeo_W_t_;
    Eigen::VectorXd tkeo_w_0_;
    double tkeo_stat_ = 1.0;
    double active_tkeo_threshold_ = 0.50;
    double rest_tkeo_threshold_ = 0.80;
    int tkeo_stat_buffer_size_ = emg_classification_window_length_ + 50;
    boost::circular_buffer<double> tkeo_stat_buffer_;
    double tkeo_buffer_fill_time_ = static_cast<double>(tkeo_stat_buffer_size_) * clock_.delta_time_;
    

    // TRAINING DATA
    std::vector<double> emg_feature_vec_ = std::vector<double>(num_features_ * meii_.N_emg_, 0.0);
    std::vector<std::vector<double>> emg_training_data_;
    std::vector<std::vector<double>> prev_emg_training_data_;

    // TRAINING PARAMETERS
    double min_cv_score_ = 0.85;
    double min_avg_cv_score_ = 0.95;

    // CLASSIFICATION
    std::vector<int> class_label_sequence_;
    int current_class_label_idx_ = -1;
    std::vector<int> pred_class_label_sequence_;
    std::vector<double> class_posteriors_;
    Eigen::MatrixXd emg_dir_W_tt_;

    // TESTING PARAMETERS
    int num_blind_testing_trials_ = 10;
    int num_full_testing_trials_ = 5;

    // PYTHON COMMUNICATION
    MelShare directory_share_;
    MelShare file_name_share_;
    MelShare cv_results_;
    MelShare lda_training_flag_;


    // MELSCOPE VARIABLES
    MelShare pos_share_;
    MelShare vel_share_;
    MelShare emg_share_;
    MelShare torque_share_;

    // DATA LOG
    //DataLog robot_log_ = DataLog("robot_log", false);
    DataLog emg_log_ = DataLog("emg_log", false);
    DataLog trial_log_ = DataLog("trial_log", false);
    DataLog debug_log_ = DataLog("debug_log", false);


    //-------------------------------------------------------------------------
    // PRIVATE FUNCTIONS
    //-------------------------------------------------------------------------

    // EMG REAL-TIME CONTROL UTILITY FUNCTIONS
    bool process_emg();
    void classify();
    double tkeo_detector(const std::vector<double>& sample_vec);


    // EMG PATTERN RECOGNITION TRAINING UTILITY FUNCTIONS
    void tkeo_model_estimation();
    void estimate_gaussian_params(const std::vector<std::vector<double>>& sample_data, Eigen::VectorXd& sample_mean, Eigen::MatrixXd& sample_cov);


    // EMG FEATURE EXTRACTION FUNCTIONS
    std::vector<double> feature_extract(const exo::MahiExoIIEmg::EmgDataBuffer& emg_data_buffer) const;
    double rms_feature_extract(const std::vector<double>& emg_channel_buffer) const;
    double mav_feature_extract(const std::vector<double>& emg_channel_buffer) const;
    double wl_feature_extract(const std::vector<double>& emg_channel_buffer) const;
    double zc_feature_extract(const std::vector<double>& emg_channel_buffer) const;
    double ssc_feature_extract(const std::vector<double>& emg_channel_buffer) const;
    void ar_4_feature_extract(std::vector<double>& coeffs, const std::vector<double>& emg_channel_buffer) const;


    // EXPERIMENT SETUP/CONDITIONS UTILITY FUNCTIONS
    void set_experiment_conditions(int scene_num);
    std::vector<int> gen_rand_class_labels(int num_labels) const;
    std::vector<int> rand_shuffle_class_labels(int num_labels_per_class) const;
    bool is_single_dof() const;
    bool is_cal() const;
    bool is_training() const;
    bool is_testing() const;
    bool is_blind() const;
    bool is_full() const;


    // EXPERIMENT VISUALIZATION UTILITY FUNCTIONS
    void set_viz_target_num(int class_label);
    std::vector<double> get_target_position(int class_label);


    // EXPERIMENT CONTROL UTILITY FUNCTIONS
    bool check_wait_time_reached(double wait_time, double init_time, double current_time) const;
    double measure_task_force(std::vector<double> commanded_torques, int target_num, int dof, int condition) const;
    bool check_force_mag_reached(double force_mag_goal, double force_mag);


    // USER INPUT/OUTPUT UTILITY FUNCTIONS
    void check_manual_stop();
    int is_any_num_key_pressed() const;


    // FILE READ/WRITE UTILITY FUNCTIONS
    template <typename T> bool read_csv(std::string filename, std::string directory, std::vector<std::vector<T>>& output);
    template <typename T> bool write_csv(std::string filename, std::string directory, const std::vector<std::vector<T>>& input) const;
    void write_emg_active_classifier();
    void load_emg_active_classifier();
    void load_emg_dir_classifier();
    void store_buffer(const MahiExoIIEmg::EmgDataBuffer& data_buffer, std::vector<std::vector<double>>& data_matrix);
    void save_log_data();
    //void init_robot_log();
    void init_emg_log();
    void init_trial_log();
    void init_debug_log();
    //void log_robot_row();
    void log_emg_row();
    void log_trial_row();
    void log_debug_row(std::vector<double> debug_row);

    // DATA ANALYSIS UTILITY FUNCTIONS
    Eigen::MatrixXd gen_confusion_mat(const std::vector<int>& actual_labels, const std::vector<int>& predicted_labels) const;

};