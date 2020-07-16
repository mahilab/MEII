// MIT License
//
// MEII - MAHI Exo-II Library
// Copyright (c) 2020 Mechatronics and Haptic Interfaces Lab - Rice University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// Author(s): Craig McDonald (craig.g.mcdonald@gmail.com)

#pragma once

#include <MEII/MahiExoII/MeiiParameters.hpp>
#include <MEII/MahiExoII/Joint.hpp>
#include <Mahi/Robo/Control/PdController.hpp>
#include <Mahi/Util/Timing/Time.hpp>
#include <Mahi/Util/Device.hpp>
#include <array>
#include <vector>
#include <atomic>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/StdVector>
#include <MEII/Control/Trajectory.hpp>

namespace meii {
    /// Class for controlling the Mahi Exo II Exoskeleton
    class MahiExoII : public mahi::util::Device{

    ///////////////////////// STANDARD CLASS FUNCTIONS AND PARAMS /////////////////////////
    
    public:
        /// Constructor
        MahiExoII(MeiiParameters parameters = MeiiParameters());
        /// Destructor
        ~MahiExoII();
        /// returns a pointer to robot joint [i]
        Joint* operator[](size_t joint_number){return meii_joints[joint_number].get();}
        /// Manually zero the encoders
        void calibrate(volatile std::atomic<bool>& stop_flag);
        /// Automatically zero the encoders
        void calibrate_auto(volatile std::atomic<bool>& stop_flag);
        /// Disables the robot and stops all smooth reference trajectories
        bool on_disable() override;
        /// Enables each of the joints on the MEII
        bool on_enable() override;

        std::vector<std::shared_ptr<Joint>> meii_joints; // vector of shared pointer of meii joints
        const MeiiParameters params_;                    // parameters used to control the meii

        std::string name_;                 // name of the MEII based on the device
        static const std::size_t n_aj = 5; // number of anatomical joints
        static const std::size_t n_rj = 5; // number of robotic joints

    ///////////////////////// SMOOTH REFERENCE TRAJECTORY CLASS AND INSTANCES /////////////////////////

    public:
        /// Class that generates smooth reference trajectories that can be updated in real time
        class SmoothReferenceTrajectory {

        public:
            /// default constructor
            SmoothReferenceTrajectory() {};
            /// constructor with speed of joints, goal reference position, and specification of active DOFs
            SmoothReferenceTrajectory(std::vector<double> speed, std::vector<double> ref_pos, std::vector<bool> active_dofs = {true, true, true, true, true});

            /// starts a trajectory given the current position, and current time
            void start(std::vector<double> current_pos, mahi::util::Time current_time);
            /// starts a trajectory given the current position, current time, and sets a new reference position
            void start(std::vector<double> ref_pos, std::vector<double> current_pos, mahi::util::Time current_time);
            /// sets a new reference position, sennds the current time, and restarts the trajectory
            void set_ref(std::vector<double> ref_pos, mahi::util::Time current_time);
            /// calculate the new reference by interpolating from the last trajectory at the given speed
            double calculate_smooth_ref(std::size_t dof, mahi::util::Time current_time);
            /// returns whether the reference is reached
            bool is_reached(std::vector<double> current_position, std::vector<double> tolerance);
            /// stops the reference trajectory
            void stop();
            /// returns whether or not the trajectory has started
            bool is_started() { return m_started; };

            size_t n_dof; // number of degrees of freedom in the smooth trajectory
            std::vector<bool> m_active_dofs; // list of the acctive degrees of freedom. should be of size 5
        private:
            bool m_is_valid; // 
            mahi::util::Time start_time_ = mahi::util::seconds(0.0);
            std::vector<double> speed_;
            bool m_started = false;
            std::vector<double> ref_;
            bool m_ref_init = false;
            std::vector<double> prev_ref_;
        };
        
        SmoothReferenceTrajectory rps_init_par_ref_; // rps position controller for initialization
    
        const std::vector<double> robot_joint_speed = { 0.25, 0.35, 0.015, 0.015, 0.015 }; // [rad/s] and [m/s] constant speed at which robot joint reference trajectories are interpolated
        const std::vector<double> anat_joint_speed = { 0.25, 0.35, 0.15, 0.15, 0.015 }; // [rad/s] and [m/s] constant speed at which anatomical joint reference trajectories are interpolated

    ///////////////////////// TORQUE SETTING FUNCTIONS /////////////////////////
    // Use smooth_pos_ctrl_torques when you have a smooth reference trajectory
    // as the input. Use pos_ctrl_torques when you have a position reference to
    // use. Note that this ref should be close to the current position as to not 
    // cause a big jump in torque output. Use raw_joint_torques to provide the 
    // raw joint torques.

    public:
        /// sets the robot joint torques based on a smooth reference trajectory given to the function
        std::vector<double> set_robot_smooth_pos_ctrl_torques(SmoothReferenceTrajectory& robot_ref, mahi::util::Time current_time);
        /// sets the anatomical joint torques based on a smooth reference trajectory given to the function
        std::vector<double> set_anat_smooth_pos_ctrl_torques(SmoothReferenceTrajectory& anat_ref, mahi::util::Time current_time);
        /// sets the robot joint torques based on a reference given to the function
        std::vector<double> set_robot_pos_ctrl_torques(std::vector<double> ref, std::vector<bool> active = std::vector<bool>(n_rj,true));
        /// sets the anatomical joint torques based on a reference given to the function
        std::vector<double> set_anat_pos_ctrl_torques(std::vector<double> ref, std::vector<bool> active = std::vector<bool>(n_aj,true));
        /// sets the robot joint torques  to the input torque
        void set_robot_raw_joint_torques(std::vector<double> new_torques);
        /// sets the anatomical joint torques to input torque
        void set_anatomical_raw_joint_torques(std::vector<double> new_torques);

    private:
        /// solving for static equilibrium joint torques HAS NOT BEEN TESTED
        void solve_static_rps_torques(std::vector<mahi::util::uint8> select_q, const Eigen::VectorXd& tau_b, const Eigen::VectorXd& qp, Eigen::VectorXd& tau_s) const;
        /// solving for static equilibrium joint torques HAS NOT BEEN TESTED
        void solve_static_rps_torques(std::vector<mahi::util::uint8> select_q, const Eigen::VectorXd& tau_b, const Eigen::VectorXd& qp, Eigen::VectorXd& tau_s, Eigen::VectorXd& tau_p) const;
        /// converts anatomical joint torques to robot joint torques for the rps mechanism
        void set_rps_ser_torques(std::vector<double>& tau_ser);

    /////////////////// GOAL CHECKING FUNCTIONS ///////////////////

    public:
        /// set rps_init_pos for the initialization smooth ref trajectory
		bool set_rps_init_pos(std::vector<double> new_rps_init_par_pos);
        /// compares the current rps config with the rps_init_pos specified
        bool check_rps_init(bool print_output = false) const;

        /// compares the current anatomical position with a specified goal position for specified DOFs
        bool check_goal_anat_pos(std::vector<double> goal_anat_pos, std::vector<char> check_dof, bool print_output = false) const;
        /// compares the current robot position with a specified goal position for specified DOFs
        bool check_goal_robot_pos(std::vector<double> goal_robot_pos, std::vector<char> check_dof, bool print_output = false) const;

    private:
        double m_rps_init_err_tol = 0.01; // [m] tolerance for initializing rps mechansim in robot joint space
        std::vector<double> m_rps_init_pos = { 0.12, 0.12, 0.12 }; // [m] position to initialize rps mechanism to

        std::vector<double> m_robot_goal_err_tol = { 2.0 * mahi::util::DEG2RAD, 3.0 * mahi::util::DEG2RAD, 0.01, 0.01, 0.01 }; // [rad] and [m] tolerance for robot joint errors 
        std::vector<double> m_anat_goal_err_tol = { 2.0 * mahi::util::DEG2RAD, 3.0 * mahi::util::DEG2RAD, 5.0 * mahi::util::DEG2RAD, 5.0 * mahi::util::DEG2RAD, 0.01 }; // [rad] and [m] tolerance for anatomical joint errors         

    /////////////////// LIMIT CHECKING ON THE MEII ///////////////////

    public:
        /// loops through joints and checks if any of them have exceeded their velocity or torque limit
        bool any_limit_exceeded();
        /// loops through joints and checks if any of them have exceeded their velocity limit
        bool any_velocity_limit_exceeded();
        /// loops through joints and checks if any of them have exceeded their torque limit
        bool any_torque_limit_exceeded();
        
    /////////////////// PUBLIC FACING ROBOT STATE ACCESS ///////////////////

    public:
        /// get vector of all anatomical joint positions
        std::vector<double> get_anatomical_joint_positions() const { return m_anatomical_joint_positions;};
        /// get single anatomical joint position
        double get_anatomical_joint_position(std::size_t index) const {return m_anatomical_joint_positions[index];};
        /// get vector of all anatomical joint velocities
        std::vector<double> get_anatomical_joint_velocities() const {return m_anatomical_joint_velocities;};
        /// get single anatomical joint velocity
        double get_anatomical_joint_velocity(std::size_t index) const {return m_anatomical_joint_velocities[index];};
        /// get vector of all robot joint positions
        std::vector<double> get_robot_joint_positions() const { return m_robot_joint_positions;};
        /// get single robot joint position
        double get_robot_joint_position(std::size_t index) const {return m_robot_joint_positions[index];};
        /// get vector of all robot joint velocities
        std::vector<double> get_robot_joint_velocities() const {return m_robot_joint_velocities;};
        /// get single robot joint velocity
        double get_robot_joint_velocity(std::size_t index) const {return m_robot_joint_velocities[index];};
        /// read all commanded joint torques (in joint space) from the desired joint. NOTE THAT THESE ARE COMMANDED TORQUE, SO IT IS NOT YET CLAMPED
        std::vector<double> get_robot_joint_command_torques(std::size_t index) const {return m_robot_joint_torques;};
        /// return the commanded joint torque (in joint space) from the desired joint. NOTE THAT THESE ARE COMMANDED TORQUE, SO IT IS NOT YET CLAMPED
        double get_robot_joint_command_torque(std::size_t index) const {return m_robot_joint_torques[index];};
        /// read all commanded anatomical joint torques from the desired joint (ONLY VALID IF ANATOMICAL SET TORQUE FUNCTION CALLED)
        std::vector<double> get_anatomical_joint_command_torques(std::size_t index) const {return m_robot_joint_torques;};
        /// return the commanded anatomical joint torque from the desired joint (ONLY VALID IF ANATOMICAL SET TORQUE FUNCTION CALLED)
        double get_anatomical_joint_command_torque(std::size_t index) const {return m_robot_joint_torques[index];};
        /// read wrist parallel positions after using update_kinematics
        std::vector<double> get_wrist_parallel_positions() const;
        /// read wrist serial positions after using update_kinematics
        std::vector<double> get_wrist_serial_positions() const;
    
    private:
        std::vector<double> m_anatomical_joint_positions; // vector of anatomical joint positions
        std::vector<double> m_anatomical_joint_velocities; // vector of anatomical joint velocities
        std::vector<double> m_anatomical_joint_torques; // vector of anatomical joint torquescd

        std::vector<double> m_robot_joint_positions; // vector of robot joint positions
        std::vector<double> m_robot_joint_velocities; // vector of robot joint velociti
        std::vector<double> m_robot_joint_torques; // vector of robot joint torques

    /////////////////// ROBOT AND ANATOMICAL PD CONTROLLERS ///////////////////
    
    public:
        double elbow_P = 100.0; // tuned 9/13/2017
        double elbow_D = 1.25; // tuned 9/13/2017
        double forearm_P = 28.0; // tuned 9/13/2017
        double forearm_D = 0.20; // tuned 9/13/2017
        double prismatic_P = 2200.0; // tuned 9//12/2017
        double prismatic_D = 30.0; // tuned 9//12/2017
        double wrist_fe_P = 15.0; // previously tried 25.0;
        double wrist_fe_D = 0.01; // previously tried 0.05;
        double wrist_ru_P = 15.0; // previously tried 30.0;
        double wrist_ru_D = 0.01; // previously tried 0.08;
        double wrist_ph_P = 1000.0;
        double wrist_ph_D = 10.0;

        std::array<mahi::robo::PdController, n_rj> robot_joint_pd_controllers_ = {
            {
                mahi::robo::PdController(elbow_P, elbow_D),
                mahi::robo::PdController(forearm_P, forearm_D),
                mahi::robo::PdController(prismatic_P, prismatic_D),
                mahi::robo::PdController(prismatic_P, prismatic_D),
                mahi::robo::PdController(prismatic_P, prismatic_D)
            }
        };

        std::array<mahi::robo::PdController, n_aj> anatomical_joint_pd_controllers_ = {
            {
                mahi::robo::PdController(elbow_P, elbow_D),
                mahi::robo::PdController(forearm_P, forearm_D),
                mahi::robo::PdController(wrist_fe_P, wrist_fe_D),
                mahi::robo::PdController(wrist_ru_P, wrist_ru_D),
                mahi::robo::PdController(wrist_ph_P, wrist_ph_D)
            }
        };

    ///////////// KINEMATIC UPDATE FUNCTIONS & VARIABLES ///////////////

    public:
        /// update robot forward kinematics from encoder readings
        void update_kinematics();
        
        static const std::size_t n_qp = 12; // number of rps dependent DoF 
        static const std::size_t n_qs = 3; // number of rps independent DoF
    private:
        /// compute the positions of the serial positions (wrist f/e, r/u deviation, forearm length) given the  measurements of the encoders
        void forward_rps_kinematics(const Eigen::VectorXd& q_par_in, Eigen::VectorXd& q_ser_out, Eigen::VectorXd& qp_out, Eigen::MatrixXd& rho_fk, Eigen::MatrixXd& jac_fk) const;
        /// compute the positions and velocities of the serial positions (wrist f/e, r/u deviation, forearm length) given the  measurements of the encoders
        void forward_rps_kinematics_velocity(const Eigen::VectorXd& q_par_in, Eigen::VectorXd& q_ser_out, Eigen::VectorXd& qp_out, Eigen::MatrixXd& rho_fk, Eigen::MatrixXd& jac_fk, const Eigen::VectorXd& q_par_dot_in, Eigen::VectorXd& q_ser_dot_out, Eigen::VectorXd& qp_dot_out) const;
        /// compute the positions of the parallel positions (the motors) given desired serial values (wrist f/e, r/u deviation, forearm length)
        void inverse_rps_kinematics(const Eigen::VectorXd& q_ser_in, Eigen::VectorXd& q_par_out, Eigen::VectorXd& qp_out, Eigen::MatrixXd& rho_ik, Eigen::MatrixXd& jac_ik) const;
        /// compute the positions and velocities of the parallel positions (the motors) given desired serial values (wrist f/e, r/u deviation, forearm length)
        void inverse_rps_kinematics_velocity(const Eigen::VectorXd& q_ser_in, Eigen::VectorXd& q_par_out, Eigen::VectorXd& qp_out, Eigen::MatrixXd& rho_ik, Eigen::MatrixXd& jac_ik, const Eigen::VectorXd& q_ser_dot_in, Eigen::VectorXd& q_par_dot_out, Eigen::VectorXd& qp_dot_out) const;
        /// generic function to solve for some set of the 12 variables given 3 variables
        void solve_rps_kinematics(std::vector<mahi::util::uint8> select_q, const Eigen::VectorXd& qs, Eigen::VectorXd& qp, Eigen::MatrixXd& rho, Eigen::MatrixXd& rho_s, mahi::util::uint32 max_it, double tol) const;
        /// generates the variable *rho* to use in solve_rps_kinematics
        void generate_rho(std::vector<mahi::util::uint8> select_q, const Eigen::VectorXd& qp, Eigen::MatrixXd& rho) const;
        /// updates the variable psi
        void psi_update(const Eigen::MatrixXd& A, const Eigen::VectorXd& qs, const Eigen::VectorXd& qp, Eigen::VectorXd& phi, Eigen::VectorXd& psi) const;
        /// updates psi_d_qp
        void psi_d_qp_update(const Eigen::MatrixXd& A, const Eigen::VectorXd& qp, Eigen::MatrixXd& phi_d_qp, Eigen::MatrixXd& psi_d_qp) const;
        /// updates phi
        void phi_update(const Eigen::VectorXd& qp, Eigen::VectorXd& phi) const;
        /// update phi_d_qp
        void phi_d_qp_update(const Eigen::VectorXd& qp, Eigen::MatrixXd& phi_d_qp) const;
        /// removes the select_q indices from the vector {0,1,2,3,4,5,6,7,8,9,10,11}
        std::vector<mahi::util::uint8> select_q_invert(std::vector<mahi::util::uint8> select_q) const;

        // geometric parameters
        static const double R_; // [m]
        static const double r_; // [m]
        static const double a56_; // [m]
        static const double alpha5_; // [rad]
        static const double alpha13_; // [rad]

        // continuously updated kinematics variables
        Eigen::VectorXd m_qp = Eigen::VectorXd::Zero(n_qp);
        Eigen::VectorXd m_q_par = Eigen::VectorXd::Zero(n_qs);
        Eigen::VectorXd m_q_ser = Eigen::VectorXd::Zero(n_qs);
        Eigen::VectorXd m_qp_dot = Eigen::VectorXd::Zero(n_qp);
        Eigen::VectorXd m_q_par_dot = Eigen::VectorXd::Zero(n_qs);
        Eigen::VectorXd m_q_ser_dot = Eigen::VectorXd::Zero(n_qs);
        Eigen::VectorXd m_tau_par_rob = Eigen::VectorXd::Zero(n_qs);
        Eigen::VectorXd m_tau_ser_rob = Eigen::VectorXd::Zero(n_qs);
        Eigen::MatrixXd m_rho_fk = Eigen::MatrixXd::Zero(n_qp - n_qs, n_qs);
        Eigen::MatrixXd m_jac_fk = Eigen::MatrixXd::Zero(n_qs, n_qs);

        // kinematics solver setup variables
        const mahi::util::uint32 m_max_it = 10; // max iterations to perform for kinematics solver
        const double m_tol = 1e-12; // tolerance for kinematics solver
        const std::vector<mahi::util::uint8> m_select_q_par = { 3, 4, 5 }; // which q values are used for parallel joints
        const std::vector<mahi::util::uint8> m_select_q_ser = { 6, 7, 9 }; // which q values are used for anatomical joints

    //////////////// MISC USEFUL UTILITY FUNCTIONS ////////////////
    
    private:
        /// compare a goal position with the current position for given joints and tolerances
        static bool check_goal_pos(std::vector<double> goal_pos, std::vector<double> current_pos, std::vector<char> check_joint, std::vector<double> error_tol, bool print_output = false);
        /// converts an eigen vector to a std vector
        std::vector<double> copy_eigvec_to_stdvec(const Eigen::VectorXd& eigen_vec);
        /// converts a std vector to an eigen vector
        Eigen::VectorXd copy_stdvec_to_eigvec(const std::vector<double>& std_vec);

    //////////////// PURE VIRTUAL FUNCTIONS FOR DERIVED CLASSES ////////////////
    public:
        /// enables the daq
        virtual bool daq_enable()=0;
        /// disables the daq
        virtual bool daq_disable()=0;
        /// opens the daq
        virtual bool daq_open()=0;
        /// closes the daq
        virtual bool daq_close()=0;
        /// starts the watchdog on the daq
        virtual bool daq_watchdog_start()=0;
        /// starts the watchdog on the daq
        virtual bool daq_watchdog_kick()=0;
        /// reads all from the daq
        virtual bool daq_read_all()=0;
        /// writes all from the daq
        virtual bool daq_write_all()=0;
        /// sets encoders to input position (in counts)
        virtual bool daq_encoder_write(int index, mahi::util::int32 encoder_offset)=0;
    };
} // namespace meii