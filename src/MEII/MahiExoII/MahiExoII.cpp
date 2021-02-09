#include <MEII/MahiExoII/MahiExoII.hpp>
#include <MEII/MahiExoII/Joint.hpp>
#include <Mahi/Daq/Quanser/Q8Usb.hpp>
#include <Mahi/Util/Math/Functions.hpp>
#include <Mahi/Util/Timing/Timer.hpp>
#include <iomanip>
#include <Mahi/Util/Print.hpp>
#include <Mahi/Util/Logging/Log.hpp>
#include <Mahi/Com/MelShare.hpp>

using namespace mahi::util;
using namespace mahi::daq;
using namespace mahi::robo;

namespace meii {  

    // geometric parameters
    const double MahiExoII::R_ = 0.1044956;
    const double MahiExoII::r_ = 0.05288174521;
    const double MahiExoII::a56_ = 0.0268986 - 0.0272820;
    const double MahiExoII::alpha5_ = 0.094516665054824;
    const double MahiExoII::alpha13_ = 5 * DEG2RAD;

    ///////////////////////// STANDARD CLASS FUNCTIONS AND PARAMS /////////////////////////

    MahiExoII::MahiExoII(MeiiParameters parameters) :
        Device("mahi_exo_ii"),
        params_(parameters)
    {

        for (int i = 0; i < n_aj; i++) {
            m_anatomical_joint_positions.push_back(0.0);
            m_anatomical_joint_velocities.push_back(0.0);
            m_anatomical_joint_torques.push_back(0.0);
            m_robot_joint_positions.push_back(0.0);
            m_robot_joint_velocities.push_back(0.0);
            m_robot_joint_torques.push_back(0.0);
        }
        std::vector<double> rps_par_joint_speed_(robot_joint_speed.begin()+2,robot_joint_speed.end());
        rps_init_par_ref_ = SmoothReferenceTrajectory(rps_par_joint_speed_, m_rps_init_pos,{false,false,true,true,true});
    }

    MahiExoII::~MahiExoII() {
        if (is_enabled()) {
            disable();
        }
    }

    bool MahiExoII::on_enable() {
        for(auto it = meii_joints.begin(); it != meii_joints.end(); ++it){
            if (!(*it)->enable()){
                LOG(Error) << "Failed to enable joints. Disabling MEII.";
                disable();
                return false;
            } 
        }
        return true;
    }

    bool MahiExoII::on_disable() {
        bool successful = true;
        for(auto it = meii_joints.begin(); it != meii_joints.end(); ++it){
            if (!(*it)->disable()){
                LOG(Error) << "Failed to disable joints. Take proper precautions before handling.";
                successful = false;
            } 
        }
        return successful;
    }

    void MahiExoII::calibrate(volatile std::atomic<bool>& stop) {
        //enable DAQ
        daq_enable();
        std::vector<int32> encoder_offsets = { 0, -33259, 29125, 29125, 29125 };
        for (int i = 0; i < n_rj; i++) {
            daq_encoder_write(i, encoder_offsets[i]);
        }
        daq_disable();
        stop = true;
    }

    // see bottom of file fore calibrate_auto because it is a long function

    ///////////////////////// SMOOTH REFERENCE TRAJECTORY CLASS AND INSTANCES /////////////////////////

    MahiExoII::SmoothReferenceTrajectory::SmoothReferenceTrajectory(std::vector<double> speed, std::vector<double> ref_pos, std::vector<bool> active_dofs) :
        speed_(speed),
        n_dof(speed.size()),
        ref_(ref_pos),
        m_ref_init(true),
        m_active_dofs(active_dofs)
        {
            n_dof = std::count(m_active_dofs.begin(), m_active_dofs.end(), true);
            m_is_valid = (n_dof == speed_.size() && n_dof == ref_.size());
        }

    void MahiExoII::SmoothReferenceTrajectory::start(std::vector<double> current_pos, Time current_time) {
        if (m_ref_init) {
            m_started = true;
            prev_ref_ = current_pos;
            start_time_ = current_time;
        }
        else {
            print("ERROR: Reference position was not initialized. Must provide reference position to start().");
        }
    }

    void MahiExoII::SmoothReferenceTrajectory::start(std::vector<double> ref_pos, std::vector<double> current_pos, Time current_time) {
        m_started = true;
        prev_ref_ = current_pos;
        ref_ = ref_pos;
        start_time_ = current_time;
    }

    void MahiExoII::SmoothReferenceTrajectory::set_ref(std::vector<double> ref_pos, Time current_time) {
        if (!m_started) {
            print("ERROR: Cannot call set_ref() before start().");
        }
        else if (ref_pos.size() != n_dof){
            print("ERROR: ref_pos is wrong size for the trajectory");
        }
        else {
            for (auto i = 0; i < ref_pos.size(); ++i) {
                prev_ref_[i] = calculate_smooth_ref(i, current_time);
            }
            ref_ = ref_pos;
            start_time_ = current_time;
        }
    }

    double MahiExoII::SmoothReferenceTrajectory::calculate_smooth_ref(std::size_t dof, Time current_time) {
        if (m_started) {
            if (ref_[dof] == prev_ref_[dof]) {
                return ref_[dof];
            }
            return prev_ref_[dof] + (ref_[dof] - prev_ref_[dof]) * (current_time.as_seconds() - start_time_.as_seconds()) * speed_[dof] / std::abs(ref_[dof] - prev_ref_[dof]);
        }
        else {
            print("ERROR: Must give reference point first.");
            return NAN;
        }
    }

    bool MahiExoII::SmoothReferenceTrajectory::is_reached(std::vector<double> current_position, std::vector<double> tolerance){
        std::vector<char> check_dofs(n_dof,1);
        return check_goal_pos(ref_, current_position, check_dofs, tolerance, false);
    }

    void MahiExoII::SmoothReferenceTrajectory::stop() {
        m_started = false;
    }

    ///////////////////////// TORQUE SETTING FUNCTIONS /////////////////////////

    std::vector<double> MahiExoII::set_robot_smooth_pos_ctrl_torques(SmoothReferenceTrajectory& robot_ref, Time current_time) {

        std::vector<double> command_torques(n_aj, 0.0);
        
        size_t num_active = 0;

        for (size_t i = 0; i < 5; i++){
            // if the dof is active, calculate the torque to use, else it remains 0
            if (robot_ref.m_active_dofs[i]){
                // calculate the new reference position
                double smooth_ref = robot_ref.calculate_smooth_ref(num_active, current_time);
                // calculate the new torque based on the reference position
                command_torques[i] = robot_joint_pd_controllers_[i].calculate(smooth_ref, get_robot_joint_position(i), 0, get_robot_joint_velocity(i));

                num_active++;
            }
        }
        set_robot_raw_joint_torques(command_torques);

        return command_torques;
    }

    std::vector<double> MahiExoII::set_anat_smooth_pos_ctrl_torques(SmoothReferenceTrajectory& anat_ref, Time current_time) {

        std::vector<double> command_torques(n_aj, 0.0);

        size_t num_active = 0;

        for (size_t i = 0; i < 5; i++){
            // if the dof is active, calculate the torque to use, else it remains 0
            if (anat_ref.m_active_dofs[i]){
                // calculate the new reference position
                double smooth_ref = anat_ref.calculate_smooth_ref(num_active, current_time);
                // calculate the new torque based on the reference position
                command_torques[i] = anatomical_joint_pd_controllers_[i].calculate(smooth_ref, get_anatomical_joint_position(i), 0, get_anatomical_joint_velocity(i));

                num_active++;
            }
        }
        set_anatomical_raw_joint_torques(command_torques);

        return command_torques;
    }

    std::vector<double> MahiExoII::set_robot_pos_ctrl_torques(std::vector<double> ref, std::vector<bool> active){
        
        std::vector<double> robot_command_torques(n_aj, 0.0);

        if(std::count(active.begin(), active.end(), true) != ref.size()){
            LOG(Error) << "Size of 'ref' param must equal number of true values in 'active' param (default 5). Commanding 0 torques.";
            return robot_command_torques;
        }
        else if(size(active) != 5){
            LOG(Error) << "Size of 'active' param must be 5. Commanding 0 torques.";
            return robot_command_torques;
        }
        
        for (std::size_t i = 0; i < n_aj; ++i) {
            if (active[i]){
                robot_command_torques[i] = robot_joint_pd_controllers_[i].calculate(ref[i], m_robot_joint_positions[i], 0, m_robot_joint_velocities[i]);
            }
        }
        set_robot_raw_joint_torques(robot_command_torques);

        return robot_command_torques;
    }

    std::vector<double> MahiExoII::set_anat_pos_ctrl_torques(std::vector<double> ref, std::vector<bool> active){
        
        std::vector<double> anat_command_torques(n_aj, 0.0);

        if(std::count(active.begin(), active.end(), true) != ref.size()){
            LOG(Error) << "Size of 'ref' param must equal number of true values in 'active' param (default 5). Commanding 0 torques.";
            return anat_command_torques;
        }
        else if(size(active) != 5){
            LOG(Error) << "Size of 'active' param must be 5. Commanding 0 torques.";
            return anat_command_torques;
        }
        
        for (std::size_t i = 0; i < n_aj; ++i) {
            if (active[i]){
                anat_command_torques[i] = anatomical_joint_pd_controllers_[i].calculate(ref[i], m_anatomical_joint_positions[i], 0, m_anatomical_joint_velocities[i]);
            }
        }
        set_anatomical_raw_joint_torques(anat_command_torques);

        return anat_command_torques;
    }

        void MahiExoII::set_robot_raw_joint_torques(std::vector<double> new_torques) {
        
        // update reference
        m_robot_joint_torques = new_torques;

        for (auto it = meii_joints.begin(); it != meii_joints.end(); ++it) {
            (*it)->set_torque(new_torques[it - meii_joints.begin()]);
        }
    }

    void MahiExoII::set_anatomical_raw_joint_torques(std::vector<double> new_torques) {
        
        // update reference
        m_anatomical_joint_torques = new_torques;
        m_robot_joint_torques[0] = new_torques[0];
        m_robot_joint_torques[1] = new_torques[1];
        
        // set torques for first two anatomical joints, which have actuators
        meii_joints[0]->set_torque(new_torques[0]);
        meii_joints[1]->set_torque(new_torques[1]);

        // write the parallel torques using set_rps_ser_torques which converts serial to parallel
        std::vector<double> tau_ser = std::vector<double>(new_torques.begin() + 2, new_torques.end());
        set_rps_ser_torques(tau_ser);
    }

    void MahiExoII::solve_static_rps_torques(std::vector<uint8> select_q, const Eigen::VectorXd& tau_b, const Eigen::VectorXd& qp, Eigen::VectorXd& tau_s) const {
        Eigen::MatrixXd rho = Eigen::MatrixXd::Zero(n_qp - n_qs, n_qs);
        generate_rho(select_q, qp, rho);
        tau_s = -rho.transpose() * tau_b;
    }

    void MahiExoII::solve_static_rps_torques(std::vector<uint8> select_q, const Eigen::VectorXd& tau_b, const Eigen::VectorXd& qp, Eigen::VectorXd& tau_s, Eigen::VectorXd& tau_p) const {
        solve_static_rps_torques(select_q, tau_b, qp, tau_s);
        for (int i = 0; i < n_qs; ++i) {
            tau_p[select_q[i]] = tau_s[i];
        }
        std::vector<uint8> indices = select_q_invert(select_q);
        for (int i = 0; i < n_qp - n_qs; ++i) {
            tau_p[indices.at(i)] = tau_b[i];
        }
    }

    void MahiExoII::set_rps_ser_torques(std::vector<double>& tau_ser) {
        Eigen::VectorXd tau_ser_eig = copy_stdvec_to_eigvec(tau_ser);
        m_tau_par_rob = m_jac_fk.transpose() * tau_ser_eig;
        for (int i = 0; i < n_qs; ++i) {
            meii_joints[i + 2]->set_torque(m_tau_par_rob[i]);
            m_robot_joint_torques[i+2] = m_tau_par_rob[i];
        }
        m_tau_ser_rob = -tau_ser_eig;
    }

    /////////////////// GOAL CHECKING FUNCTIONS ///////////////////

	bool MahiExoII::set_rps_init_pos(std::vector<double> new_rps_init_par_pos) {
		bool new_pos_valid = true;
		if (new_rps_init_par_pos.size() != 3) {
			new_pos_valid = false;
			LOG(Warning) << "Invalid size of input argument given to MEII::set_rps_init_pos(). Must be of size 3.";
		}
		for (std::size_t i = 0; i < new_rps_init_par_pos.size(); ++i) {
			if (new_rps_init_par_pos[i] < params_.pos_limits_min_[i + 2] || new_rps_init_par_pos[i] > params_.pos_limits_max_[i + 2]) {
				new_pos_valid = false;
			}			
		}
		if (!new_pos_valid) {
			LOG(Warning) << "Input argument given to MEII::set_rps_init_pos() contains value outside valid range.";
		}
		m_rps_init_pos = new_rps_init_par_pos;
        std::vector<double> rps_par_joint_speed_(robot_joint_speed.begin()+2,robot_joint_speed.end());
		rps_init_par_ref_ = SmoothReferenceTrajectory(rps_par_joint_speed_, m_rps_init_pos,{false,false,true,true,true});
		return new_pos_valid;
	}

    bool MahiExoII::check_rps_init(bool print_output) const {
        std::vector<double> rps_pos_tol_vec(n_qs, m_rps_init_err_tol);
        return check_goal_pos(m_rps_init_pos, get_wrist_parallel_positions(), { 1,1,1 }, rps_pos_tol_vec, print_output);
    }

    bool MahiExoII::check_goal_anat_pos(std::vector<double> goal_anat_pos, std::vector<char> check_dof, bool print_output) const {
        return check_goal_pos(goal_anat_pos, get_anatomical_joint_positions(), check_dof, m_anat_goal_err_tol, print_output);
    }

    bool MahiExoII::check_goal_robot_pos(std::vector<double> goal_robot_pos, std::vector<char> check_dof, bool print_output) const {
        return check_goal_pos(goal_robot_pos, get_robot_joint_positions(), check_dof, m_robot_goal_err_tol, print_output);
    }

    /////////////////// LIMIT CHECKING ON THE MEII ///////////////////

    bool MahiExoII::any_limit_exceeded(){
        return (any_velocity_limit_exceeded() || any_torque_limit_exceeded());
    }

    bool MahiExoII::any_velocity_limit_exceeded(){
        bool exceeded = false;
        for (auto it = meii_joints.begin(); it != meii_joints.end(); ++it) {
            if ((*it)->velocity_limit_exceeded())
                exceeded = true;
        }
        return exceeded;
    }

    bool MahiExoII::any_torque_limit_exceeded(){
        bool exceeded = false;
        for (auto it = meii_joints.begin(); it != meii_joints.end(); ++it) {
            if ((*it)->torque_limit_exceeded())
                exceeded = true;
        }
        return exceeded;
    }

    /////////////////// PUBLIC FACING ROBOT STATE ACCESS ///////////////////
    // see MahiExoII.hpp for the remaining single line functions 

    std::vector<double> MahiExoII::get_wrist_parallel_positions() const {
        return std::vector<double>(m_robot_joint_positions.begin()+2,m_robot_joint_positions.end());
    }

    std::vector<double> MahiExoII::get_wrist_serial_positions() const {
        return std::vector<double>(m_anatomical_joint_positions.begin()+2,m_anatomical_joint_positions.end());
    }

    ///////////// KINEMATIC UPDATE FUNCTIONS ///////////////

    void MahiExoII::update_kinematics() {
        // update m_q_par (q parallel) with the three prismatic link positions
        m_q_par << meii_joints[2]->get_position(), meii_joints[3]->get_position(), meii_joints[4]->get_position();
        m_q_par_dot << meii_joints[2]->get_velocity(), meii_joints[3]->get_velocity(), meii_joints[4]->get_velocity();

        // update joint velocities if necessary (only if using hardware version and filtering is done in software) 
        // otherwise this does nothing
        for (size_t i = 0; i < n_rj; i++){
            meii_joints[i]->filter_velocity();
        }

        for (size_t i = 0; i < n_rj; i++){
            m_robot_joint_positions[i] = meii_joints[i]->get_position();
            m_robot_joint_velocities[i] = meii_joints[i]->get_velocity();
        }

        // run forward kinematics solver to update q_ser (q serial) and m_qp (q prime), which contains all 12 RPS positions
        forward_rps_kinematics_velocity(m_q_par, m_q_ser, m_qp, m_rho_fk, m_jac_fk, m_q_par_dot, m_q_ser_dot, m_qp_dot);
        
        // get positions from first two anatomical joints, which have encoders
        m_anatomical_joint_positions[0] = m_robot_joint_positions[0]; // elbow flexion/extension
        m_anatomical_joint_positions[1] = m_robot_joint_positions[1]; // forearm pronation/supination

        // get positions from forward kinematics solver for three wrist anatomical joints 
        m_anatomical_joint_positions[2] = m_q_ser[0]; // wrist flexion/extension
        m_anatomical_joint_positions[3] = m_q_ser[1]; // wrist radial/ulnar deviation
        m_anatomical_joint_positions[4] = m_q_ser[2]; // arm translation

        // get velocities from first two anatomical joints, which have encoders
        m_anatomical_joint_velocities[0] = m_robot_joint_velocities[0]; // elbow flexion/extension
        m_anatomical_joint_velocities[1] = m_robot_joint_velocities[1]; // forearm pronation/supination

        // get velocities from forward kinematics solver for three wrist anatomical joints 
        m_anatomical_joint_velocities[2] = m_q_ser_dot[0]; // wrist flexion/extension
        m_anatomical_joint_velocities[3] = m_q_ser_dot[1]; // wrist radial/ulnar deviation
        m_anatomical_joint_velocities[4] = m_q_ser_dot[2]; // arm translation
    }

    void MahiExoII::forward_rps_kinematics(const Eigen::VectorXd& q_par_in, Eigen::VectorXd& q_ser_out, Eigen::VectorXd& qp_out, Eigen::MatrixXd& rho_fk, Eigen::MatrixXd& jac_fk) const {
        Eigen::MatrixXd rho_s = Eigen::MatrixXd::Zero(n_qp, n_qs);
        solve_rps_kinematics(m_select_q_par, q_par_in, qp_out, rho_fk, rho_s, m_max_it, m_tol);
        q_ser_out << qp_out(m_select_q_ser[0]), qp_out(m_select_q_ser[1]), qp_out(m_select_q_ser[2]);
        for (int i = 0; i < n_qs; ++i) {
            jac_fk.row(i) = rho_s.row(m_select_q_ser[i]);
        }
    }

    void MahiExoII::forward_rps_kinematics_velocity(const Eigen::VectorXd& q_par_in, Eigen::VectorXd& q_ser_out, Eigen::VectorXd& qp_out, Eigen::MatrixXd& rho_fk, Eigen::MatrixXd& jac_fk, const Eigen::VectorXd& q_par_dot_in, Eigen::VectorXd& q_ser_dot_out, Eigen::VectorXd& qp_dot_out) const {
        forward_rps_kinematics(q_par_in, q_ser_out, qp_out, rho_fk, jac_fk);
        // here on is velocity
        q_ser_dot_out = jac_fk * q_par_dot_in;
        Eigen::VectorXd qb_dot = Eigen::VectorXd::Zero(n_qp - n_qs);
        qb_dot = rho_fk * q_par_dot_in;
        for (int i = 0; i < n_qs; ++i) {
            qp_dot_out[m_select_q_ser[i]] = q_ser_dot_out[i];
        }
        std::vector<uint8> indices = select_q_invert(m_select_q_ser);
        for (int i = 0; i < n_qp - n_qs; ++i) {
            qp_dot_out[indices[i]] = qb_dot[i];
        }
    }

    void MahiExoII::inverse_rps_kinematics(const Eigen::VectorXd& q_ser_in, Eigen::VectorXd& q_par_out, Eigen::VectorXd& qp_out, Eigen::MatrixXd& rho_ik, Eigen::MatrixXd& jac_ik) const {
        Eigen::MatrixXd rho_s = Eigen::MatrixXd::Zero(n_qp, n_qs);
        solve_rps_kinematics(m_select_q_ser, q_ser_in, qp_out, rho_ik, rho_s, m_max_it, m_tol);
        q_par_out << qp_out(m_select_q_par[0]), qp_out(m_select_q_par[1]), qp_out(m_select_q_par[2]);
        for (int i = 0; i < n_qs; ++i) {
            jac_ik.row(i) = rho_s.row(m_select_q_par[i]);
        }
    }

    void MahiExoII::inverse_rps_kinematics_velocity(const Eigen::VectorXd& q_ser_in, Eigen::VectorXd& q_par_out, Eigen::VectorXd& qp_out, Eigen::MatrixXd& rho_ik, Eigen::MatrixXd& jac_ik, const Eigen::VectorXd& q_ser_dot_in, Eigen::VectorXd& q_par_dot_out, Eigen::VectorXd& qp_dot_out) const {
        inverse_rps_kinematics(q_ser_in, q_par_out, qp_out, rho_ik, jac_ik);
        q_par_dot_out = jac_ik * q_ser_dot_in;
        Eigen::VectorXd qb_dot = Eigen::VectorXd::Zero(n_qp - n_qs);
        qb_dot = rho_ik * q_ser_dot_in;
        for (int i = 0; i < n_qs; ++i) {
            qp_dot_out[m_select_q_par.at(i)] = q_par_dot_out[i];
        }
        std::vector<uint8> indices = select_q_invert(m_select_q_par);
        for (int i = 0; i < n_qp - n_qs; ++i) {
            qp_dot_out[indices.at(i)] = qb_dot[i];
        }
    }

    void MahiExoII::solve_rps_kinematics(std::vector<uint8> select_q, const Eigen::VectorXd& qs, Eigen::VectorXd& qp, Eigen::MatrixXd& rho, Eigen::MatrixXd& rho_s, uint32 max_it, double tol) const {

        // build selection matrix
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_qs, n_qp);
        for (int i = 0; i < n_qs; ++i) {
            A(i, select_q[i]) = 1;
        }

        // initialize variable containing solution
        qp << PI / 4, PI / 4, PI / 4, 0.1305, 0.1305, 0.1305, 0, 0, 0, 0.0923, 0, 0;

        // initialize temporary variables containing kinematic constraints etc.
        Eigen::VectorXd phi = Eigen::VectorXd::Zero(n_qp - n_qs);
        Eigen::MatrixXd phi_d_qp = Eigen::MatrixXd::Zero(n_qp - n_qs, n_qp);
        Eigen::VectorXd psi = Eigen::VectorXd::Zero(n_qp);
        Eigen::MatrixXd psi_d_qp = Eigen::MatrixXd::Zero(n_qp, n_qp);
        Eigen::MatrixXd rho_rhs = Eigen::MatrixXd::Zero(n_qp, n_qs);
        rho_rhs.bottomRows(n_qs) = Eigen::MatrixXd::Identity(n_qs, n_qs);

        // initialize variables for keeping track of error
        double err = 2 * tol;
        bool first_non_zero = true;

        // run no more than max_it iterations of updating the solution for m_qp
        // exit loop once the error is below the input tolerance
        uint32 it = 0;
        while (it < max_it && err > tol) {
            psi_update(A, qs, qp, phi, psi); // calculate 9 constraints and tracking error on specified qs_
            psi_d_qp_update(A, qp, phi_d_qp, psi_d_qp); // derivative of psi w.r.t. qp, giving a 12x12 matrix      
            qp -= psi_d_qp.fullPivLu().solve(psi);

            // update the error. this ends up being sqrt of sum of squares
            err = 0;
            for (auto j = 0; j != n_qp; ++j) {
                err += psi[j]*psi[j];
            }
            err = sqrt(err);

            // while iterator
            it++;
        }

        rho_s = psi_d_qp.fullPivLu().solve(rho_rhs);

        // remove rows corresponding to q_select indices
        std::vector<uint8> indices = select_q_invert(select_q);
        for (int i = 0; i < n_qp - n_qs; ++i) {
            rho.row(i) = rho_s.row(indices.at(i));
        }
    }

    void MahiExoII::generate_rho(std::vector<uint8> select_q, const Eigen::VectorXd& qp, Eigen::MatrixXd& rho) const {

        // build selection matrix
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_qs, n_qp);
        for (int i = 0; i < n_qs; ++i) {
            A(i, select_q[i]) = 1;
        }

        // initialize temporary variables containing kinematic constraints etc.
        Eigen::MatrixXd phi_d_qp = Eigen::MatrixXd::Zero(n_qp - n_qs, n_qp);
        Eigen::MatrixXd psi_d_qp = Eigen::MatrixXd::Zero(n_qp, n_qp);
        Eigen::MatrixXd rho_s = Eigen::MatrixXd::Zero(n_qp, n_qs);
        Eigen::MatrixXd rho_rhs = Eigen::MatrixXd::Zero(n_qp, n_qs);
        rho_rhs.bottomRows(n_qs) = Eigen::MatrixXd::Identity(n_qs, n_qs);

        // compute rho_s
        psi_d_qp_update(A, qp, phi_d_qp, psi_d_qp);
        rho_s = psi_d_qp.fullPivLu().solve(rho_rhs);

        // remove rows corresponding to q_select indices
        std::vector<uint8> indices = select_q_invert(select_q);
        for (int i = 0; i < n_qp - n_qs; ++i) {
            rho.row(i) = rho_s.row(indices.at(i));
        }

    }

    void MahiExoII::psi_update(const Eigen::MatrixXd& A, const Eigen::VectorXd& qs, const Eigen::VectorXd& qp, Eigen::VectorXd& phi, Eigen::VectorXd& psi) const {
        phi_update(qp, phi);
        psi.head(n_qp - n_qs) = phi;
        psi.tail(n_qs) = A*qp - qs;
    }


    void MahiExoII::psi_d_qp_update(const Eigen::MatrixXd& A, const Eigen::VectorXd& qp, Eigen::MatrixXd& phi_d_qp, Eigen::MatrixXd& psi_d_qp) const {
        phi_d_qp_update(qp, phi_d_qp);
        psi_d_qp.block<n_qp - n_qs, n_qp>(0, 0) = phi_d_qp;
        psi_d_qp.block<n_qs, n_qp>(n_qp - n_qs, 0) = A;
    }

    void MahiExoII::phi_update(const Eigen::VectorXd& qp, Eigen::VectorXd& phi) const {

        phi << qp[3] * sin(qp[0]) - qp[9] - r_*cos(alpha13_)*(sin(qp[6])*sin(qp[8]) - cos(qp[6])*cos(qp[8])*sin(qp[7])) - r_*sin(alpha13_)*(cos(qp[8])*sin(qp[6]) + cos(qp[6])*sin(qp[7])*sin(qp[8])),
        R_*cos(alpha5_) - qp[10] - a56_*sin(alpha5_) - qp[3] * cos(alpha5_)*cos(qp[0]) - r_*cos(alpha13_)*cos(qp[7])*cos(qp[8]) + r_*cos(qp[7])*sin(alpha13_)*sin(qp[8]),
        a56_*cos(alpha5_) - qp[11] + R_*sin(alpha5_) - qp[3] * sin(alpha5_)*cos(qp[0]) - r_*cos(alpha13_)*(cos(qp[6])*sin(qp[8]) + cos(qp[8])*sin(qp[6])*sin(qp[7])) - r_*sin(alpha13_)*(cos(qp[6])*cos(qp[8]) - sin(qp[6])*sin(qp[7])*sin(qp[8])),
        qp[4] * sin(qp[1]) - qp[9] - r_*cos(alpha13_ - (2 * PI) / 3)*(sin(qp[6])*sin(qp[8]) - cos(qp[6])*cos(qp[8])*sin(qp[7])) - r_*sin(alpha13_ - (2 * PI) / 3)*(cos(qp[8])*sin(qp[6]) + cos(qp[6])*sin(qp[7])*sin(qp[8])),
        R_*cos(alpha5_ - (2 * PI) / 3) - qp[10] - a56_*sin(alpha5_ - (2 * PI) / 3) - qp[4] * cos(alpha5_ - (2 * PI) / 3)*cos(qp[1]) - r_*cos(qp[7])*cos(qp[8])*cos(alpha13_ - (2 * PI) / 3) + r_*cos(qp[7])*sin(qp[8])*sin(alpha13_ - (2 * PI) / 3),
        a56_*cos(alpha5_ - (2 * PI) / 3) - qp[11] + R_*sin(alpha5_ - (2 * PI) / 3) - qp[4] * cos(qp[1])*sin(alpha5_ - (2 * PI) / 3) - r_*cos(alpha13_ - (2 * PI) / 3)*(cos(qp[6])*sin(qp[8]) + cos(qp[8])*sin(qp[6])*sin(qp[7])) - r_*sin(alpha13_ - (2 * PI) / 3)*(cos(qp[6])*cos(qp[8]) - sin(qp[6])*sin(qp[7])*sin(qp[8])),
        qp[5] * sin(qp[2]) - qp[9] - r_*cos((2 * PI) / 3 + alpha13_)*(sin(qp[6])*sin(qp[8]) - cos(qp[6])*cos(qp[8])*sin(qp[7])) - r_*sin((2 * PI) / 3 + alpha13_)*(cos(qp[8])*sin(qp[6]) + cos(qp[6])*sin(qp[7])*sin(qp[8])),
        R_*cos((2 * PI) / 3 + alpha5_) - qp[10] - a56_*sin((2 * PI) / 3 + alpha5_) - qp[5] * cos((2 * PI) / 3 + alpha5_)*cos(qp[2]) - r_*cos(qp[7])*cos(qp[8])*cos((2 * PI) / 3 + alpha13_) + r_*cos(qp[7])*sin(qp[8])*sin((2 * PI) / 3 + alpha13_),
        a56_*cos((2 * PI) / 3 + alpha5_) - qp[11] + R_*sin((2 * PI) / 3 + alpha5_) - qp[5] * cos(qp[2])*sin((2 * PI) / 3 + alpha5_) - r_*cos((2 * PI) / 3 + alpha13_)*(cos(qp[6])*sin(qp[8]) + cos(qp[8])*sin(qp[6])*sin(qp[7])) - r_*sin((2 * PI) / 3 + alpha13_)*(cos(qp[6])*cos(qp[8]) - sin(qp[6])*sin(qp[7])*sin(qp[8]));
    }

    void MahiExoII::phi_d_qp_update(const Eigen::VectorXd& qp, Eigen::MatrixXd& phi_d_qp) const {

        phi_d_qp << qp[3] * cos(qp[0]), 0, 0, sin(qp[0]), 0, 0, -r_*cos(alpha13_)*(cos(qp[6])*sin(qp[8]) + cos(qp[8])*sin(qp[6])*sin(qp[7])) - r_*sin(alpha13_)*(cos(qp[6])*cos(qp[8]) - sin(qp[6])*sin(qp[7])*sin(qp[8])), r_*cos(qp[6])*cos(alpha13_)*cos(qp[7])*cos(qp[8]) - r_*cos(qp[6])*cos(qp[7])*sin(alpha13_)*sin(qp[8]), r_*sin(alpha13_)*(sin(qp[6])*sin(qp[8]) - cos(qp[6])*cos(qp[8])*sin(qp[7])) - r_*cos(alpha13_)*(cos(qp[8])*sin(qp[6]) + cos(qp[6])*sin(qp[7])*sin(qp[8])), -1, 0, 0,
        qp[3] * cos(alpha5_)*sin(qp[0]), 0, 0, -cos(alpha5_)*cos(qp[0]), 0, 0, 0, r_*cos(alpha13_)*cos(qp[8])*sin(qp[7]) - r_*sin(alpha13_)*sin(qp[7])*sin(qp[8]), r_*cos(alpha13_)*cos(qp[7])*sin(qp[8]) + r_*cos(qp[7])*cos(qp[8])*sin(alpha13_), 0, -1, 0,
        qp[3] * sin(alpha5_)*sin(qp[0]), 0, 0, -sin(alpha5_)*cos(qp[0]), 0, 0, r_*cos(alpha13_)*(sin(qp[6])*sin(qp[8]) - cos(qp[6])*cos(qp[8])*sin(qp[7])) + r_*sin(alpha13_)*(cos(qp[8])*sin(qp[6]) + cos(qp[6])*sin(qp[7])*sin(qp[8])), r_*cos(qp[7])*sin(qp[6])*sin(alpha13_)*sin(qp[8]) - r_*cos(alpha13_)*cos(qp[7])*cos(qp[8])*sin(qp[6]), r_*sin(alpha13_)*(cos(qp[6])*sin(qp[8]) + cos(qp[8])*sin(qp[6])*sin(qp[7])) - r_*cos(alpha13_)*(cos(qp[6])*cos(qp[8]) - sin(qp[6])*sin(qp[7])*sin(qp[8])), 0, 0, -1,
        0, qp[4] * cos(qp[1]), 0, 0, sin(qp[1]), 0, -r_*cos(alpha13_ - (2 * PI) / 3)*(cos(qp[6])*sin(qp[8]) + cos(qp[8])*sin(qp[6])*sin(qp[7])) - r_*sin(alpha13_ - (2 * PI) / 3)*(cos(qp[6])*cos(qp[8]) - sin(qp[6])*sin(qp[7])*sin(qp[8])), r_*cos(qp[6])*cos(qp[7])*cos(qp[8])*cos(alpha13_ - (2 * PI) / 3) - r_*cos(qp[6])*cos(qp[7])*sin(qp[8])*sin(alpha13_ - (2 * PI) / 3), r_*sin(alpha13_ - (2 * PI) / 3)*(sin(qp[6])*sin(qp[8]) - cos(qp[6])*cos(qp[8])*sin(qp[7])) - r_*cos(alpha13_ - (2 * PI) / 3)*(cos(qp[8])*sin(qp[6]) + cos(qp[6])*sin(qp[7])*sin(qp[8])), -1, 0, 0,
        0, qp[4] * cos(alpha5_ - (2 * PI) / 3)*sin(qp[1]), 0, 0, -cos(alpha5_ - (2 * PI) / 3)*cos(qp[1]), 0, 0, r_*cos(qp[8])*cos(alpha13_ - (2 * PI) / 3)*sin(qp[7]) - r_*sin(qp[7])*sin(qp[8])*sin(alpha13_ - (2 * PI) / 3), r_*cos(qp[7])*cos(qp[8])*sin(alpha13_ - (2 * PI) / 3) + r_*cos(qp[7])*cos(alpha13_ - (2 * PI) / 3)*sin(qp[8]), 0, -1, 0,
        0, qp[4] * sin(alpha5_ - (2 * PI) / 3)*sin(qp[1]), 0, 0, -cos(qp[1])*sin(alpha5_ - (2 * PI) / 3), 0, r_*cos(alpha13_ - (2 * PI) / 3)*(sin(qp[6])*sin(qp[8]) - cos(qp[6])*cos(qp[8])*sin(qp[7])) + r_*sin(alpha13_ - (2 * PI) / 3)*(cos(qp[8])*sin(qp[6]) + cos(qp[6])*sin(qp[7])*sin(qp[8])), r_*cos(qp[7])*sin(qp[6])*sin(qp[8])*sin(alpha13_ - (2 * PI) / 3) - r_*cos(qp[7])*cos(qp[8])*cos(alpha13_ - (2 * PI) / 3)*sin(qp[6]), r_*sin(alpha13_ - (2 * PI) / 3)*(cos(qp[6])*sin(qp[8]) + cos(qp[8])*sin(qp[6])*sin(qp[7])) - r_*cos(alpha13_ - (2 * PI) / 3)*(cos(qp[6])*cos(qp[8]) - sin(qp[6])*sin(qp[7])*sin(qp[8])), 0, 0, -1,
        0, 0, qp[5] * cos(qp[2]), 0, 0, sin(qp[2]), -r_*cos((2 * PI) / 3 + alpha13_)*(cos(qp[6])*sin(qp[8]) + cos(qp[8])*sin(qp[6])*sin(qp[7])) - r_*sin((2 * PI) / 3 + alpha13_)*(cos(qp[6])*cos(qp[8]) - sin(qp[6])*sin(qp[7])*sin(qp[8])), r_*cos(qp[6])*cos(qp[7])*cos(qp[8])*cos((2 * PI) / 3 + alpha13_) - r_*cos(qp[6])*cos(qp[7])*sin(qp[8])*sin((2 * PI) / 3 + alpha13_), r_*sin((2 * PI) / 3 + alpha13_)*(sin(qp[6])*sin(qp[8]) - cos(qp[6])*cos(qp[8])*sin(qp[7])) - r_*cos((2 * PI) / 3 + alpha13_)*(cos(qp[8])*sin(qp[6]) + cos(qp[6])*sin(qp[7])*sin(qp[8])), -1, 0, 0,
        0, 0, qp[5] * cos((2 * PI) / 3 + alpha5_)*sin(qp[2]), 0, 0, -cos((2 * PI) / 3 + alpha5_)*cos(qp[2]), 0, r_*cos(qp[8])*cos((2 * PI) / 3 + alpha13_)*sin(qp[7]) - r_*sin(qp[7])*sin(qp[8])*sin((2 * PI) / 3 + alpha13_), r_*cos(qp[7])*cos(qp[8])*sin((2 * PI) / 3 + alpha13_) + r_*cos(qp[7])*cos((2 * PI) / 3 + alpha13_)*sin(qp[8]), 0, -1, 0,
        0, 0, qp[5] * sin((2 * PI) / 3 + alpha5_)*sin(qp[2]), 0, 0, -cos(qp[2])*sin((2 * PI) / 3 + alpha5_), r_*cos((2 * PI) / 3 + alpha13_)*(sin(qp[6])*sin(qp[8]) - cos(qp[6])*cos(qp[8])*sin(qp[7])) + r_*sin((2 * PI) / 3 + alpha13_)*(cos(qp[8])*sin(qp[6]) + cos(qp[6])*sin(qp[7])*sin(qp[8])), r_*cos(qp[7])*sin(qp[6])*sin(qp[8])*sin((2 * PI) / 3 + alpha13_) - r_*cos(qp[7])*cos(qp[8])*cos((2 * PI) / 3 + alpha13_)*sin(qp[6]), r_*sin((2 * PI) / 3 + alpha13_)*(cos(qp[6])*sin(qp[8]) + cos(qp[8])*sin(qp[6])*sin(qp[7])) - r_*cos((2 * PI) / 3 + alpha13_)*(cos(qp[6])*cos(qp[8]) - sin(qp[6])*sin(qp[7])*sin(qp[8])), 0, 0, -1;
    }

    std::vector<uint8> MahiExoII::select_q_invert(std::vector<uint8> select_q) const {
        std::vector<uint8> indices{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
        for (int i = 0; i < n_qs; ++i) {
            indices.erase(indices.begin() + select_q[i]);
        }
        return indices;
    }

    //////////////// MISC USEFUL UTILITY FUNCTIONS ////////////////

    bool MahiExoII::check_goal_pos(std::vector<double> goal_pos, std::vector<double> current_pos, std::vector<char> check_dof, std::vector<double> error_tol, bool print_output) {
        
        assert((goal_pos.size() == current_pos.size()) && ((goal_pos.size() == check_dof.size())) && (goal_pos.size() == error_tol.size()));

        bool goal_reached = true;
        for (size_t i = 0; i < goal_pos.size(); ++i) {
            if (check_dof.at(i)) {
                if (std::abs(goal_pos[i] - current_pos[i]) > std::abs(error_tol[i])) {
                    if (print_output && goal_reached) {
                        std::cout << "Joint " << std::to_string(i) << " error is " << (std::abs(goal_pos[i] - current_pos[i])*RAD2DEG) << std::endl;
                    }
                    goal_reached = false;
                }
            }
        }
        return goal_reached;
    }

    std::vector<double> MahiExoII::copy_eigvec_to_stdvec(const Eigen::VectorXd& eigen_vec) {
        std::vector<double> std_vec(eigen_vec.size());
        for (int i = 0; i < eigen_vec.size(); ++i) {
            std_vec[i] = eigen_vec[i];
        }
        return std_vec;
    }

    Eigen::VectorXd MahiExoII::copy_stdvec_to_eigvec(const std::vector<double>& std_vec) {
        Eigen::VectorXd eigen_vec(std_vec.size());
        for (size_t i = 0; i < std_vec.size(); ++i) {
            eigen_vec[i] = std_vec[i];
        }
        return eigen_vec;
    }

    // TODO: Make this much more efficient
    void MahiExoII::calibrate_auto(volatile std::atomic<bool>& stop) {

        // calibration offsets for the joints
        std::array<int32, 5>  encoder_offsets = { 0, -33259, 29125, 29125, 29125 };

        // destinations for the joints after setting calibration
        std::array<double, 5> neutral_points  = { -35 * DEG2RAD, 00 * DEG2RAD, 0.13, 0.13, 0.13 };
        
        // create needed variables
        std::vector<double> zeros = { 0, 0, 0, 0, 0 }; // determined zero positions for each joint
        std::array<int, 5> dir = { 1 , -1, 1, 1, 1 };    // direction to rotate each joint
        uint32 calibrating_joint = 0;               // joint currently calibrating
        bool returning = false;                          // bool to track if calibrating joint is return to zero
        double pos_ref = 0;                              // desired position

        std::array<double, 5> vel_ref = {10 * DEG2RAD, 20 * DEG2RAD, 0.01, 0.01, 0.01}; // desired velocities
        
        std::vector<double> stored_positions;  // stores past positions
        stored_positions.reserve(100000);

        std::vector<std::vector<double>> par_stored_positions;  // stores past positions
        for (size_t i = 0; i < 3; i++)
        {
            par_stored_positions.push_back({});
            par_stored_positions[i].reserve(10000);
        }
        std::vector<double> par_pos_ref = {0.0, 0.0, 0.0};
        std::vector<bool>   par_returning = {false, false, false};

        std::array<double, 5> sat_torques = { 2.0, 2.0, 15.0, 15.0, 15.0 }; // temporary saturation torques

        Time timeout = seconds(60); // max amout of time we will allow calibration to occur for

        // enable DAQs, zero encoders, and start watchdog
        daq_enable();
        for (size_t i = 0; i < 5; i++){
            daq_encoder_write(i,0);
        }

        enable_realtime();

        // enable MEII
        enable();

        // daq_watchdog_start();

        daq_read_all();
        update_kinematics();

        zeros = get_robot_joint_positions();
        pos_ref = zeros[calibrating_joint];
        par_pos_ref = {zeros[2], zeros[3], zeros[4]};

        // start the clock
        Timer timer(milliseconds(1), Timer::WaitMode::Hybrid);
        
        // start the calibration control loop
        while (!stop && timer.get_elapsed_time() < timeout) {

            // read and reload DAQs
            daq_read_all();
            update_kinematics();
            // daq_watchdog_kick();

            if (calibrating_joint < 2){
                // iterate over all joints
                for (std::size_t i = 0; i < 2; i++) {

                    // get positions and velocities
                    double pos_act = meii_joints[i]->get_position();
                    double vel_act = meii_joints[i]->get_velocity();

                    double torque = 0;
                    if (i == calibrating_joint) {
                        if (!returning) {

                            // calculate torque req'd to move the calibrating joint forward at constant speed
                            pos_ref += dir[i] * vel_ref[i] * timer.get_period().as_seconds();
                            torque = robot_joint_pd_controllers_[i].calculate(pos_ref, pos_act, 0, vel_act);
                            torque = clamp(torque, sat_torques[i]);

                            // check if the calibrating joint is still moving
                            stored_positions.push_back(pos_act);
                            bool moving = true;
                            if (stored_positions.size() > 100) {
                                moving = false;
                                for (size_t j = stored_positions.size() - 100; j < stored_positions.size(); j++) {
                                    moving = stored_positions[j] != stored_positions[j - 1];
                                    // daq_watchdog_kick();
                                    if (moving)
                                        break;
                                }
                            }

                            // if it's not moving, it's at a hardstop so record the position and deduce the zero location
                            if (!moving) {
                                daq_encoder_write(i,encoder_offsets[i]);
                                returning = true;
                                // update the reference position to be the current one
                                pos_ref = meii_joints[i]->get_position();
                            }
                        }

                        else {
                            // calculate torque req'd to retur the calibrating joint back to zero
                            pos_ref -= dir[i] * vel_ref[i] *  timer.get_period().as_seconds();
                            torque = robot_joint_pd_controllers_[i].calculate(pos_ref, pos_act, 0, vel_act);
                            torque = clamp(torque, sat_torques[i]);


                            if (dir[i] * pos_ref <= dir[i] * neutral_points[i]) {
                                // reset for the next joint
                                calibrating_joint += 1;
                                pos_ref = zeros[calibrating_joint];
                                returning = false;
                                LOG(Info) << "Joint " << meii_joints[i]->get_name() << " calibrated";
                            }
                        }
                    }
                    else {
                        // lock all other joints at their zero positions
                        if (i > calibrating_joint){
                            torque = robot_joint_pd_controllers_[i].calculate(zeros[i], pos_act, 0, vel_act);
                        }
                        else{
                            torque = robot_joint_pd_controllers_[i].calculate(neutral_points[i], pos_act, 0, vel_act);
                        }
                        torque = clamp(torque, sat_torques[i]);
                    }
                    // print("joint {} - pos: {}, ref: {}, torque: {}", i, pos_act, pos_ref, torque);
                    meii_joints[i]->set_torque(torque);
                }

                // set rps joint torques
                for (std::size_t i = 0; i < 3; ++i) {
					double torque = robot_joint_pd_controllers_[i + 2].calculate(zeros[i+2], meii_joints[i + 2]->get_position(), 0, meii_joints[i + 2]->get_velocity());
                    torque = clamp(torque, sat_torques[i]+2);
                    meii_joints[i + 2]->set_torque(torque);
				}
            }
            else{
                for (size_t i = 0; i < 2; i++){
                    double torque = robot_joint_pd_controllers_[i].calculate(neutral_points[i], meii_joints[i]->get_position(), 0, meii_joints[i]->get_velocity());
                    torque = clamp(torque, sat_torques[i]);
                    meii_joints[i]->set_torque(torque);
                }

                std::vector<bool> par_moving = {true, true, true};
                
                for (std::size_t i = 0; i < 3; i++) {
                    double torque = 0;
                    size_t dof_num = i+2;
                    // get positions and velocities
                    double pos_act = get_robot_joint_position(dof_num);
                    double vel_act = get_robot_joint_velocity(dof_num);
                    
                    if (std::all_of(par_returning.begin(), par_returning.end(), [](bool v) { return !v; })) {
                        par_pos_ref[i] += dir[dof_num] * vel_ref[dof_num] * timer.get_period().as_seconds();
                        // print("{} - pos: {}, ref: {}", dof_num, pos_act, par_pos_ref[i]);

                        torque = robot_joint_pd_controllers_[dof_num].calculate(par_pos_ref[i], pos_act, 0, vel_act);
                        torque = clamp(torque, sat_torques[dof_num]);

                        // check if the calibrating joint is still moving
                        par_stored_positions[i].push_back(pos_act);

                        if (par_stored_positions[i].size() > 500) {
                            par_moving[i] = false;
                            for (size_t j = par_stored_positions[i].size() - 500; j < par_stored_positions[i].size(); j++) {
                                par_moving[i] = par_stored_positions[i][j] != par_stored_positions[i][j-1];
                                // daq_watchdog_kick();
                                if (par_moving[i])
                                    break;
                            }
                        }
                        
                        // if it's not moving, it's at a hardstop so record the position and deduce the zero location
                        if (std::all_of(par_moving.begin(), par_moving.end(), [](bool v) { return !v; })) {
                            for (size_t j = 0; j < 3; j++){
                                daq_encoder_write((int32)j+2,encoder_offsets[j+2]);
                                // update the reference position to be the current one
                                par_pos_ref[j] = meii_joints[j+2]->get_position();
                            }                        
                            par_returning = {true, true, true};
                        }
                    }

                    else{
                        if (par_returning[i]){
                            // calculate torque req'd to retur the calibrating joint back to zero
                            par_pos_ref[i] -= dir[dof_num] * vel_ref[dof_num] *  timer.get_period().as_seconds();
                            torque = robot_joint_pd_controllers_[dof_num].calculate(par_pos_ref[i], pos_act, 0, vel_act);
                            torque = clamp(torque, sat_torques[dof_num]);

                            if (i == 2) print_var(par_pos_ref[i]);

                            if (dir[dof_num] * par_pos_ref[i] <= dir[dof_num] * neutral_points[dof_num]) {
                                // reset for the next joint
                                par_returning[i] = false;
                                par_pos_ref[i] = 0;
                                LOG(Info) << "Joint " << meii_joints[dof_num]->get_name() << " calibrated";
                                if (std::all_of(par_returning.begin(), par_returning.end(), [](bool v) { return !v; })){
                                    print("shoulda stopped!");
                                    stop = true;
                                }
                            }
                        }
                        else{
                            // LOG(Info) << "Joint " << meii_joints[dof_num]->get_name() << " waiting";
                            torque = robot_joint_pd_controllers_[dof_num].calculate(neutral_points[dof_num], pos_act, 0, vel_act);
                            torque = clamp(torque, sat_torques[dof_num]);
                        }
                    }
                    meii_joints[dof_num]->set_torque(torque);
                }
            }
            
            // write all DAQs
            daq_write_all();

            // check joint velocity limits
            if (any_limit_exceeded()) {
                stop = true;
                break;
            }

            // wait the clock
            timer.wait();
        }

        // disable MEII
        disable();

        // disable DAQ
        daq_disable();

        disable_realtime();
    }

    ///////////////////////////////////////////////////
    ////////////////// EXPERIMENTAL ///////////////////
    ///////////////////////////////////////////////////

} // namespace meii