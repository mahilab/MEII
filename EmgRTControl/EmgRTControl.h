#pragma once
#include "StateMachine.h"
#include "Q8Usb.h"
#include "MahiExoIIEmg.h"
#include "MelShare.h"
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
        ST_NUM_STATES
    };

    // STATE FUNCTIONS
    void sf_wait_for_gui(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_wait_for_gui> sa_wait_for_gui;

    void sf_init(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_init> sa_init;

    void sf_backdrive(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_backdrive> sa_backdrive;

    void sf_init_rps(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_init_rps> sa_init_rps;

    void sf_to_center(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_to_center> sa_to_center;

    void sf_hold_center(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_hold_center> sa_hold_center;

    void sf_hold_for_input(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_hold_for_input> sa_hold_for_input;

    void sf_present_target(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_present_target> sa_present_target;

    void sf_to_target(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_to_target> sa_to_target;

    void sf_hold_target(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_hold_target> sa_hold_target;

    void sf_finish(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_finish> sa_finish;

    void sf_stop(const util::NoEventData*);
    util::StateAction<EmgRTControl, util::NoEventData, &EmgRTControl::sf_stop> sa_stop;

    // STATE MAP
    virtual const util::StateMapRow* get_state_map() {
        static const util::StateMapRow STATE_MAP[] = {
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
        };
        return &STATE_MAP[0];
    } 

    //-------------------------------------------------------------------------
    // PRIVATE VARIABLES
    //-------------------------------------------------------------------------
    
    // SUBJECT/CONDITION
    int subject_number_ = 0;
    std::vector<std::string> hand_defs = { "L","R" };
    int hand_num_ = 0; // 0 or 1 for Left or Right arm of the user
    std::string hand_def_ = hand_defs[hand_num_];
    int dof_; // 0-3 is single-dof; 4-5 is multi-dof
    int condition_; // 0 = calibration; 1 = training; 2 = blind testing; 3 = full testing
    

    // FILE NAMES & DIRECTORIES
    std::vector<std::string> str_conditions_long_ = { "Calibration", "Training", "Blind Testing", "Full Testing"  };
    std::vector<std::string> str_conditions_ = { "cal", "trng", "blind", "full"  };
    std::vector<std::string> str_dofs_long_ = { "Elbow F/E Single-DoF", "Forearm P/S Single-Dof", "Wrist F/E Single-DoF", "Wrist R/U Single-DoF", "Elbow F/E & Forearm P/S Multi-DoF", "Wrist F/E & Wrist R/U Multi-DoF" };
    std::vector<std::string> str_dofs_ = { "EFE", "FPS", "WFE", "WRU", "ELFM", "WMLT" };
    std::string program_directory_ = "C:\\Users\\Ted\\GitHub\\MEII\\bin";
    std::string subject_directory_;
    std::string subject_dof_directory_;
    std::string calibration_data_filename_;
    std::string training_data_filename_;
    std::string lda_classifier_filename_;

    // UNITY GAME
    util::ExternalApp game = mel::util::ExternalApp("2D_targets", "C:\\Users\\Ted\\GitHub\\MEII\\Exo Visualization\\Builds\\Exo_Vis_Build_1.exe");

    // HARDWARE CLOCK
    util::Clock clock_;

    // HARDWARE
    core::Daq* daq_;
    exo::MahiExoIIEmg meii_;

    // INPUT CLASS LABELS
    bool class_labels_from_file_ = false;
    std::vector<int> class_label_sequence_;
    int current_class_label_idx_ = -1;
    
    // PREDEFINED TARGETS
    const double_vec center_pos_ = { -35 * math::DEG2RAD, 0 * math::DEG2RAD, 0 * math::DEG2RAD, 0 * math::DEG2RAD,  0.09 }; // anatomical joint positions
    const std::vector<std::vector<std::vector<double_vec>>> single_dof_targets_ = { { { {  -5 * math::DEG2RAD,   0 * math::DEG2RAD,   0 * math::DEG2RAD,   0 * math::DEG2RAD,  0.09 }, { -65 * math::DEG2RAD,   0 * math::DEG2RAD,   0 * math::DEG2RAD,   0 * math::DEG2RAD,  0.09 } },
                                                                                      { { -35 * math::DEG2RAD, -30 * math::DEG2RAD,   0 * math::DEG2RAD,   0 * math::DEG2RAD,  0.09 }, { -35 * math::DEG2RAD,  30 * math::DEG2RAD,   0 * math::DEG2RAD,   0 * math::DEG2RAD,  0.09 } },
                                                                                      { { -35 * math::DEG2RAD,   0 * math::DEG2RAD, -15 * math::DEG2RAD,   0 * math::DEG2RAD,  0.09 }, { -35 * math::DEG2RAD,   0 * math::DEG2RAD,  15 * math::DEG2RAD,   0 * math::DEG2RAD,  0.09 } },
                                                                                      { { -35 * math::DEG2RAD,   0 * math::DEG2RAD,   0 * math::DEG2RAD,  15 * math::DEG2RAD,  0.09 }, { -35 * math::DEG2RAD,   0 * math::DEG2RAD,   0 * math::DEG2RAD, -15 * math::DEG2RAD,  0.09 } } },
            
                                                                                    { { {  -5 * math::DEG2RAD,   0 * math::DEG2RAD,   0 * math::DEG2RAD,   0 * math::DEG2RAD,  0.09 }, { -65 * math::DEG2RAD,   0 * math::DEG2RAD,   0 * math::DEG2RAD,   0 * math::DEG2RAD,  0.09 } },
                                                                                      { { -35 * math::DEG2RAD,  30 * math::DEG2RAD,   0 * math::DEG2RAD,   0 * math::DEG2RAD,  0.09 }, { -35 * math::DEG2RAD, -30 * math::DEG2RAD,   0 * math::DEG2RAD,   0 * math::DEG2RAD,  0.09 } },
                                                                                      { { -35 * math::DEG2RAD,   0 * math::DEG2RAD,  15 * math::DEG2RAD,   0 * math::DEG2RAD,  0.09 }, { -35 * math::DEG2RAD,   0 * math::DEG2RAD, -15 * math::DEG2RAD,   0 * math::DEG2RAD,  0.09 } },
                                                                                      { { -35 * math::DEG2RAD,   0 * math::DEG2RAD,   0 * math::DEG2RAD,  15 * math::DEG2RAD,  0.09 }, { -35 * math::DEG2RAD,   0 * math::DEG2RAD,   0 * math::DEG2RAD, -15 * math::DEG2RAD,  0.09 } } } };

    const std::vector<std::vector<std::vector<double_vec>>>  multi_dof_targets_ = { { { {  -5 * math::DEG2RAD, -30 * math::DEG2RAD,   0 * math::DEG2RAD,   0 * math::DEG2RAD,  0.09 }, {  -5 * math::DEG2RAD,  30 * math::DEG2RAD,   0 * math::DEG2RAD,   0 * math::DEG2RAD,  0.09 }, { -65 * math::DEG2RAD, -30 * math::DEG2RAD,   0 * math::DEG2RAD,   0 * math::DEG2RAD,  0.09 }, { -65 * math::DEG2RAD,  30 * math::DEG2RAD,   0 * math::DEG2RAD,   0 * math::DEG2RAD,  0.09 } },
                                                                                      { { -35 * math::DEG2RAD,   0 * math::DEG2RAD, -15 * math::DEG2RAD,  15 * math::DEG2RAD,  0.09 }, { -35 * math::DEG2RAD,   0 * math::DEG2RAD,  15 * math::DEG2RAD,  15 * math::DEG2RAD,  0.09 }, { -35 * math::DEG2RAD,   0 * math::DEG2RAD, -15 * math::DEG2RAD, -15 * math::DEG2RAD,  0.09 }, { -35 * math::DEG2RAD,   0 * math::DEG2RAD,  15 * math::DEG2RAD, -15 * math::DEG2RAD,  0.09 } } },

                                                                                    { { {  -5 * math::DEG2RAD,  30 * math::DEG2RAD,   0 * math::DEG2RAD,   0 * math::DEG2RAD,  0.09 }, {  -5 * math::DEG2RAD, -30 * math::DEG2RAD,   0 * math::DEG2RAD,   0 * math::DEG2RAD,  0.09 }, { -65 * math::DEG2RAD,  30 * math::DEG2RAD,   0 * math::DEG2RAD,   0 * math::DEG2RAD,  0.09 }, { -65 * math::DEG2RAD, -30 * math::DEG2RAD,   0 * math::DEG2RAD,   0 * math::DEG2RAD,  0.09 } },
                                                                                      { { -35 * math::DEG2RAD,   0 * math::DEG2RAD,  15 * math::DEG2RAD,  15 * math::DEG2RAD,  0.09 }, { -35 * math::DEG2RAD,   0 * math::DEG2RAD, -15 * math::DEG2RAD,  15 * math::DEG2RAD,  0.09 }, { -35 * math::DEG2RAD,   0 * math::DEG2RAD,  15 * math::DEG2RAD, -15 * math::DEG2RAD,  0.09 }, { -35 * math::DEG2RAD,   0 * math::DEG2RAD, -15 * math::DEG2RAD,  15 * math::DEG2RAD,  0.09 } } } };

    // EXPERIMENT TIMING PARAMETERS
    double init_backdrive_time_ = 2.0; // [s] time to be in backdrive mode initially
    double hold_center_time_ = 1.0; // time to hold at center target [s]
    double hold_target_time_ = 1.0; // time to hold at target [s]
    double wait_mvc_time_ = 0.5; // time to wait during MVC [s]

    // TASK FORCE MEASUREMENT
    int task_force_measurement_mode_ = 0; // 0 = commanded torques, dof/direction specific; 1 = commanded torques, dof specific, direction agnostic; 2 = commanded torques, dof/direction agnostic
    double def_efe_trq = 3.00;
    double def_fps_trq = 0.50;
    double def_wfe_trq = 0.50;
    double def_wru_trq = 0.50;
    const std::vector<std::vector<double_vec>> force_dof_scale_ = { { { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq } }, // elbow f/e single-dof
                                                                    { { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq } }, // forearm p/s single-dof
                                                                    { { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq } }, // wrist f/e single-dof
                                                                    { { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq } }, // wrist r/u single-dof
                                                                    { { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq } }, // elbow f/e & forearm p/s multi-dof
                                                                    { { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq } } }; // wrist f/e & wrist r/u multi-dof
    const std::vector<char_vec> target_dir_L_ = {
        { 1, -1 },
        { -1, 1 },
        { -1, 1 },
        { 1, -1 },
        { 1, 1, -1, -1 },
        { -1, 1, -1, 1 },
        { 1, 1, -1, -1 },
        { 1, -1, 1, -1 } };
    const std::vector<char_vec> target_dir_R_ = {
        { 1, -1 },
        { 1, -1 },
        { 1, -1 },
        { 1, -1 },
        { 1, 1, -1, -1 },
        { 1, -1, 1, -1 },
        { 1, 1, -1, -1 },
        { -1, 1, -1, 1 } };
    const double_vec gravity_offsets_ = { -0.0, 0.0, 0.0, -0.35, 0.0}; // for all 5 anatomical dofs, due to counterweight elbow can be under 'negative gravity'
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
    comm::MelShare scene_num_share_ = comm::MelShare("scene_num");
    comm::MelShare viz_target_num_share_ = comm::MelShare("target");
    comm::MelShare force_mag_share_ = comm::MelShare("force_mag");
    comm::MelShare hand_select_ = comm::MelShare("hand");
    
    // EMG SENSING AND FEATURE EXTRACTION PARAMETERS
    static const int num_emg_channels_ = 8;
    static const int num_features_ = 9;
    static const int emg_calibration_window_length_ = 500;
    static const int emg_trigger_window_length_ = 100;
    static const int emg_classification_window_length_ = 200;
   
    // STATE TRANSITION EVENT VARIABLES
    bool end_of_label_sequence_ = true;
    bool stop_ = false;

    // TEMPORARY EMG DATA CONTAINERS
    exo::MahiExoIIEmg::EmgDataBuffer emg_calibration_data_buffer_ = exo::MahiExoIIEmg::EmgDataBuffer(meii_.N_emg_, emg_calibration_window_length_);
    exo::MahiExoIIEmg::EmgDataBuffer emg_trigger_data_buffer_ = exo::MahiExoIIEmg::EmgDataBuffer(meii_.N_emg_, emg_trigger_window_length_);
    exo::MahiExoIIEmg::EmgDataBuffer emg_classification_data_buffer_ = exo::MahiExoIIEmg::EmgDataBuffer(meii_.N_emg_, emg_classification_window_length_);

    // EMG CALIBRATION
    std::vector<double_vec> calibration_data_;
    std::vector<double_vec> tkeo_rest_data_;
    std::vector<double_vec> tkeo_active_data_;
    Eigen::VectorXd rest_sample_mean_ = Eigen::VectorXd::Zero(meii_.N_emg_);
    Eigen::VectorXd active_sample_mean_ = Eigen::VectorXd::Zero(meii_.N_emg_);
    Eigen::MatrixXd rest_sample_cov_ = Eigen::MatrixXd::Zero(meii_.N_emg_, meii_.N_emg_);
    Eigen::MatrixXd active_sample_cov_ = Eigen::MatrixXd::Zero(meii_.N_emg_, meii_.N_emg_);
    Eigen::MatrixXd rest_sample_cov_inv_ = Eigen::MatrixXd::Zero(meii_.N_emg_, meii_.N_emg_);
    Eigen::MatrixXd active_sample_cov_inv_ = Eigen::MatrixXd::Zero(meii_.N_emg_, meii_.N_emg_);
    double tkeo_threshold_;
    //double tkeo_suff_stat_ = 0;
    int tkeo_detector_memory_ = 0;
    
    

    // TRAINING DATA
    double_vec emg_feature_vec_ = double_vec(num_features_ * meii_.N_emg_, 0.0);
    std::vector<std::vector<double>> emg_training_data_;
    std::vector<std::vector<double>> prev_emg_training_data_;

    // CLASSIFICATION
    int pred_class_label_ = 0;
    int classifier_result_;
    std::vector<std::vector<double>> lda_classifier_;
    Eigen::MatrixXd lda_class_eig_;
    Eigen::VectorXd lda_dist_eig_;


    // PYTHON COMMUNICATION
    comm::MelShare directory_share_ = comm::MelShare("file_path");
    comm::MelShare file_name_share_ = comm::MelShare("file_name");
    comm::MelShare cv_results_ = comm::MelShare("cv_results");
    comm::MelShare lda_training_flag_ = comm::MelShare("lda_training_flag");


    // MELSCOPE VARIABLES
    comm::MelShare pos_share_ = comm::MelShare("pos_share");
    comm::MelShare vel_share_ = comm::MelShare("vel_share");
    comm::MelShare emg_share_ = comm::MelShare("emg_share");
    comm::MelShare torque_share_ = comm::MelShare("torque_share");

    // DATA LOG
    util::DataLog robot_log_ = util::DataLog("robot_log", false);
    std::vector<double> robot_data_;
    void log_robot_row();

    util::DataLog training_log_ = util::DataLog("training_log", false);
    void log_training_row();

    util::DataLog lda_log_ = util::DataLog("lda_coeff_log", false);
    std::vector<double> lda_coeff_data_;

    //util::DataLog feature_log_ = util::DataLog("feature_sel_log", false);
    //std::vector<int> feat_sel_data;

    //-------------------------------------------------------------------------
    // PRIVATE FUNCTIONS
    //-------------------------------------------------------------------------

    // EMG REAL-TIME CONTROL UTILITY FUNCTIONS
    bool process_emg();
    int classify();
    void tkeo_model_estimation();
    double tkeo_sufficient_statistic(double_vec& sample_vec);
    int tkeo_detector(double_vec& sample_vec);
    


    // EXPERIMENT SETUP/CONDITIONS UTILITY FUNCTIONS
    void set_experiment_conditions(int scene_num);
    std::vector<int> gen_rand_class_labels(int num_labels) const;
    std::vector<int> rand_shuffle_class_labels(int num_labels_per_class) const;
    bool is_single_dof() const;
    bool is_cal() const;
    bool is_training() const;
    bool is_testing() const;
    bool is_blind() const;
    

    // FILE READ/WRITE UTILITY FUNCTIONS
    template <typename T> bool read_csv(std::string filename, std::string directory, std::vector<std::vector<T>>& output);
    template <typename T> bool write_csv(std::string filename, std::string directory, const std::vector<std::vector<T>>& input) const;
    void write_calibration_data();
    void load_calibration_data();
    void load_classifier();
    
    void set_viz_target_num(int class_label);
    double_vec get_target_position(int class_label) const;
    
    bool check_wait_time_reached(double wait_time, double init_time, double current_time) const;
    double measure_task_force(double_vec commanded_torques, int target_num, int dof, int condition) const;
    bool check_force_mag_reached(double force_mag_goal, double force_mag);
    
    
    void save_data();
    bool check_stop();
    int is_any_num_key_pressed() const;
    //void compute_teager_kaiser_threshold(const std::vector<double_vec>& calibration_data, std::vector<double_vec>& thresholds);
    //bool check_teager_kaiser_threshold(const exo::MahiExoIIEmg::EmgDataBuffer& emg_data_buffer, double threshold) const;
    //void teager_kaiser(const double_vec& emg_channel_buffer, double_vec& teager_kaiser_vec) const;
    void store_buffer(const exo::MahiExoIIEmg::EmgDataBuffer& data_buffer, std::vector<double_vec>& data_matrix);
    
    

    // EMG FEATURE EXTRACTION FUNCTIONS
    double_vec feature_extract(const exo::MahiExoIIEmg::EmgDataBuffer& emg_data_buffer) const;
    double rms_feature_extract(const double_vec& emg_channel_buffer) const;
    double mav_feature_extract(const double_vec& emg_channel_buffer) const;
    double wl_feature_extract(const double_vec& emg_channel_buffer) const;
    double zc_feature_extract(const double_vec& emg_channel_buffer) const;
    double ssc_feature_extract(const double_vec& emg_channel_buffer) const;
    void ar_4_feature_extract(double_vec& coeffs, const double_vec& emg_channel_buffer) const;

    

};