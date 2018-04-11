#ifndef MEII_EMG_REAL_TIME_CONTROL_HPP
#define MEII_EMG_REAL_TIME_CONTROL_HPP

#include "Classification/EmgActiveEnsClassifier.hpp"
#include "Unity/UnityEmgRtc.hpp"
#include "MEL/Utility/StateMachine.hpp"
#include "MEL/Daq/Quanser/Q8Usb.hpp"
#include "MEL/Exoskeletons/MahiExoII/MahiExoII.hpp"
#include "MEL/Communications/Windows/MelShare.hpp"
#include "MEL/Communications/MelNet.hpp"
#include "MEL/Utility/Timer.hpp"
#include "MEL/Utility/Time.hpp"
#include "MEL/Utility/RingBuffer.hpp"
#include "MEL/Logging/DataLogger.hpp"
#include "MEL/Daq/Daq.hpp"
#include "MEL/Utility/Console.hpp"
#include "EMG/MesArray.hpp"


using namespace mel;

class EmgRealTimeControlData : public EventData {

public:

};

class EmgRealTimeControl : public StateMachine {

public:

    /// Constructor
    EmgRealTimeControl( Daq& daq, Watchdog& watchdog, MesArray& mes, MahiExoII& meii, ctrl_bool& manual_stop, bool virtual_hardware = false);

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
        ST_CALIBRATION,
        ST_TRAINING,
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
    StateAction<EmgRealTimeControl, NoEventData, &EmgRealTimeControl::sf_wait_for_gui> sa_wait_for_gui;

    void sf_init(const NoEventData*);
    StateAction<EmgRealTimeControl, NoEventData, &EmgRealTimeControl::sf_init> sa_init;

    void sf_backdrive(const NoEventData*);
    StateAction<EmgRealTimeControl, NoEventData, &EmgRealTimeControl::sf_backdrive> sa_backdrive;

    void sf_init_rps(const NoEventData*);
    StateAction<EmgRealTimeControl, NoEventData, &EmgRealTimeControl::sf_init_rps> sa_init_rps;

    void sf_to_center(const NoEventData*);
    StateAction<EmgRealTimeControl, NoEventData, &EmgRealTimeControl::sf_to_center> sa_to_center;

    void sf_hold_center(const NoEventData*);
    StateAction<EmgRealTimeControl, NoEventData, &EmgRealTimeControl::sf_hold_center> sa_hold_center;

    void sf_hold_for_input(const NoEventData*);
    StateAction<EmgRealTimeControl, NoEventData, &EmgRealTimeControl::sf_hold_for_input> sa_hold_for_input;

    void sf_calibration(const NoEventData*);
    StateAction<EmgRealTimeControl, NoEventData, &EmgRealTimeControl::sf_calibration> sa_calibration;

    void sf_training(const NoEventData*);
    StateAction<EmgRealTimeControl, NoEventData, &EmgRealTimeControl::sf_training> sa_training;

    void sf_present_target(const NoEventData*);
    StateAction<EmgRealTimeControl, NoEventData, &EmgRealTimeControl::sf_present_target> sa_present_target;

    void sf_to_target(const NoEventData*);
    StateAction<EmgRealTimeControl, NoEventData, &EmgRealTimeControl::sf_to_target> sa_to_target;

    void sf_hold_target(const NoEventData*);
    StateAction<EmgRealTimeControl, NoEventData, &EmgRealTimeControl::sf_hold_target> sa_hold_target;

    void sf_finish(const NoEventData*);
    StateAction<EmgRealTimeControl, NoEventData, &EmgRealTimeControl::sf_finish> sa_finish;

    void sf_stop(const NoEventData*);
    StateAction<EmgRealTimeControl, NoEventData, &EmgRealTimeControl::sf_stop> sa_stop;

    void sf_fault_stop(const NoEventData*);
    StateAction<EmgRealTimeControl, NoEventData, &EmgRealTimeControl::sf_fault_stop> sa_fault_stop;

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
            &sa_calibration,
            &sa_training,
            &sa_present_target,
            &sa_to_target,
            &sa_hold_target,
            &sa_finish,
            &sa_stop,
            &sa_fault_stop,
        };
        return &STATE_MAP[0];
    }

private:

    // EXPERIMENT SETUP/CONDITIONS UTILITY FUNCTIONS

    std::vector<int> gen_rand_class_labels(int num_labels) const;
    std::vector<int> rand_shuffle_class_labels(int num_labels_per_class) const;
    bool is_single_dof() const;
    bool is_cal() const;
    bool is_training() const;
    bool is_testing() const;
    bool is_blind() const;
    bool is_full() const;

    // EXPERIMENT CONTROL UTILITY FUNCTIONS
    std::vector<double> get_target_position(int class_label);
    bool check_wait_time_reached(Time wait_time, Time init_time, Time current_time) const;
    double measure_task_force(std::vector<double> commanded_torques, int target_num, int dof, int condition) const;
    //bool check_force_mag_reached(double force_mag_goal, double force_mag);


    // USER INPUT/OUTPUT UTILITY FUNCTIONS
    void check_exit_program();
    int is_any_num_key_pressed() const;


    // FILE READ/WRITE UTILITY FUNCTIONS
    template <typename T> bool read_csv(std::string filename, std::string directory, std::vector<std::vector<T>>& output);
    template <typename T> bool write_csv(std::string filename, std::string directory, const std::vector<std::vector<T>>& input) const;
    bool write_emg_active_classifier();
    bool load_emg_active_classifier();
    bool load_emg_dir_training_data();
    bool load_emg_dir_classifier();
    void save_log_data();
    void init_robot_log();
    void init_emg_log();
    void init_trial_log();
    void log_robot_row();
    void log_emg_row();
    void log_trial_row();

    // DATA ANALYSIS UTILITY FUNCTIONS
    Eigen::MatrixXd gen_confusion_mat(const std::vector<int>& actual_labels, const std::vector<int>& predicted_labels) const;

    // EMG REAL-TIME CONTROL UTILITY FUNCTIONS
    bool collect_emg_dir_sample();
    void classify();
    //double tkeo_detector(const std::vector<double>& sample_vec);

    // EMG FEATURE EXTRACTION FUNCTIONS
    std::vector<double> feature_extract();


private:

    // OPTIONAL MODES FOR DEBUGGING
    bool virtual_exo_; ///< when true, prevents computing joint kinematics, computing and setting joint torques, and all writing to the DAQ; also prevents any checks dependent on exo motion
    bool virtual_emg_; ///< when true, prevents any checks based on emg readings
    bool scope_mode_ = false; ///< when true, write to robot data to melscope

    // SUBJECT/CONDITION
    int subject_number_ = 0; ///< subject number used for saving data, set to 0 when not running official experiment
    int hand_num_ = 1; ///< 0 or 1 for Left or Right arm of the user, respectively
    int dof_; ///< 0-3 is single-dof; 4-5 is multi-dof
    int num_classes_; ///< number of classes based on the dof, 2 for single, 4 for multi
    int condition_; ///< 0 = calibration; 1 = training; 2 = blind testing; 3 = full testing
    
    // HARDWARE  
    Daq& daq_; ///< generic data acquisition class
    Watchdog& watchdog_; ///< watchdog associated with the DAQ
    MesArray& mes_; ///< array of myoelectric signals
    MahiExoII& meii_; ///< MAHI Exo-II
    Time Ts_; ///< sample period for the DAQ
    Timer timer_; ///< timer to control the loop rate for closed-loop-control of the robot and signal processing

    // UNITY GAME
    UnityEmgRtc game_; ///< instance of the custom Unity game made for the EmgRealTimeControl experiment 

    // STATE TRANSITION EVENT VARIABLES
    ctrl_bool& manual_stop_; ///< initialize to false; becomes true when user presses 'Ctrl+C' at the console
    bool auto_stop_; ///< initialize to false; becomes true when any robot limits are exceeded or any other programmed error catch occurs
    bool exit_program_; ///< initialize to false; becomes true when user presses 'Escape' at the console
    bool menu_; ///< initialize to true; remains true when GUI is at the main menu; otherwise false
    bool end_of_label_sequence_; ///< initialize to true; set to false when new sequence of labels is generated; returns to true when end of sequence reached
    bool active_detector_computed_; ///< initialize to false; becomes true when the calibration has been successfully completed, or active detector has been loaded

    // EXPERIMENT TIMING PARAMETERS
    Time init_backdrive_time_ = seconds(2.0); ///< time to be in backdrive mode initially
    Time hold_center_time_ = seconds(1.0); ///< time to hold at center target
    Time hold_target_time_ = seconds(1.0); ///< time to hold at target
    Time wait_mvc_time_ = seconds(0.5); ///< time to wait during MVC
    Time detection_expire_time_ = seconds(10.0); ///< time after which failure to detect active state is marked as misclassification

    // REAL-TIME CLASSIFIERS
    EmgActiveEnsClassifier active_detector_; ///< vector of real-time classifiers that act as one ensemble classifier for detecting when the user is active in a certain DoF

    // STRING NAMES
    std::vector<std::string> str_conditions_long_ = { "Calibration", "Training", "Blind Testing", "Full Testing" };
    std::vector<std::string> str_conditions_ = { "cal", "trng", "blind", "full" };
    std::vector<std::string> str_dofs_long_ = { "Elbow F/E Single-DoF", "Forearm P/S Single-Dof", "Wrist F/E Single-DoF", "Wrist R/U Single-DoF", "Elbow F/E & Forearm P/S Multi-DoF", "Wrist F/E & Wrist R/U Multi-DoF" };
    std::vector<std::string> str_dofs_ = { "EFE", "FPS", "WFE", "WRU", "ELFM", "WMLT" };
    std::vector<std::string> str_states_ = { "ST_WAIT_FOR_GUI", "ST_INIT", "ST_BACKDRIVE", "ST_INIT_RPS", "ST_TO_CENTER", "ST_HOLD_CENTER", "ST_HOLD_FOR_INPUT", "ST_PRESENT_TARGET", "ST_TO_TARGET", "ST_HOLD_TARGET", "ST_FINISH", "ST_STOP", "ST_FAULT_STOP" };
    
    // FILE NAMES & DIRECTORIES
    std::string program_directory_ = "C:\\Users\\Ted\\GitHub\\MEII\\python";
    std::string project_directory_ = "C:\\Users\\Ted\\GitHub\\MEII\\EmgRTControl";
    std::string subject_directory_;
    std::string subject_dof_directory_;
    std::string emg_active_classifier_filename_;
    std::string training_data_filename_;
    std::string emg_dir_classifier_filename_;    

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
                                                                    { { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq } }, /// elbow f/e & forearm p/s multi-dof
                                                                    { { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq }, { def_efe_trq, def_fps_trq, def_wfe_trq, def_wru_trq } } }; /// wrist f/e & wrist r/u multi-dof
    const std::vector<std::vector<int>> target_dir_L_ = {
        { 1, -1 },
        { -1, 1 },
        { -1, 1 },
        { 1, -1 },
        { 1, 1, -1, -1 },
        { -1, 1, -1, 1 },
        { 1, 1, -1, -1 },
        { 1, -1, 1, -1 } };
    const std::vector<std::vector<int>> target_dir_R_ = {
        { 1, -1 },
        { 1, -1 },
        { 1, -1 },
        { 1, -1 },
        { 1, 1, -1, -1 },
        { 1, -1, 1, -1 },
        { 1, 1, -1, -1 },
        { -1, 1, -1, 1 } };
    const std::vector<double> gravity_offsets_ = { 0.3, 0.0, 0.0, -0.35, 0.0 }; // for all 5 anatomical dofs, due to counterweight elbow can be under 'negative gravity'
    double force_mag_goal_ = 1.0;// 3050.0;
    //double force_mag_tol_ = 300.0;
    //double force_mag_dwell_time_ = 1.0; /// [s]

    // FROCE MAGNITUDE CHECKING ALGORITHM VARIABLES
    //double force_mag_maintained_ = 0.0;
    //double force_mag_time_now_ = 0.0;
    //double force_mag_time_last_ = 0.0;


    // EMG SENSING AND FEATURE EXTRACTION PARAMETERS
    //static const int num_emg_channels_ = 8;
    //std::size_t num_emg_channels_;
    static const std::size_t num_features_ = 9;
    static const size_t emg_calibration_window_length_ = 500;
    static const size_t emg_classification_window_length_ = 200;
    

    // TEMPORARY EMG DATA CONTAINERS
    //std::vector<double> emg_voltages_ = std::vector<double>(num_emg_channels_);
    //std::vector<double> filtered_emg_voltages_ = std::vector<double>(num_emg_channels_);
    //std::vector<double> mes_tkeo_env_ = std::vector<double>(num_emg_channels_);


    // EMG CALIBRATION
    //std::vector<std::vector<double>> calibration_data_;
    //std::vector<std::vector<double>> tkeo_rest_data_;
    //std::vector<std::vector<std::vector<double>>> tkeo_active_data_;
    //Eigen::MatrixXd tkeo_W_t_;
    //Eigen::VectorXd tkeo_w_0_;
    //double tkeo_stat_ = 1.0;
    //double active_tkeo_threshold_ = 0.60;
    //double rest_tkeo_threshold_ = 0.80;
    //size_t tkeo_stat_buffer_size_ = emg_classification_window_length_ + (size_t) 50;
    //RingBuffer<double> tkeo_stat_buffer_;
    //Time tkeo_buffer_fill_time_ = static_cast<int64>(tkeo_stat_buffer_size_) * timer_.get_period();
    

    // TRAINING DATA
    std::vector<double> emg_training_data_row_;//emg_feature_vec_;// = std::vector<double>(num_features_ * num_emg_channels_, 0.0);
    std::vector<std::vector<double>> emg_training_data_;
    std::vector<std::vector<double>> prev_emg_training_data_;

    // TRAINING PARAMETERS
    double min_cv_score_ = 0.85;
    double min_avg_cv_score_ = 0.95;

    // CLASSIFICATION
    std::vector<int> class_label_sequence_; ///< sequence of class labels to be presented: 1-2 for single-DoF and 1-4 for multi-DoF
    int current_class_label_idx_;
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


    //// MELSCOPE VARIABLES
    MelShare pos_share_;
    MelShare vel_share_;
    MelShare emg_share_;
    MelShare torque_share_;

    // DATA LOG
    DataLogger robot_log_;
    DataLogger emg_log_;
    DataLogger trial_log_;
    DataLogger debug_log_;

    

};

#endif // MEII_EMG_REAL_TIME_CONTROL_HPP