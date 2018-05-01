#include <MEII/Control/MinimumJerk.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Math/Functions.hpp>

using namespace mel;

namespace meii {

	MinimumJerk::MinimumJerk(const Time &sample_period, const WayPoint &start, const WayPoint &goal) :
		Ts_(sample_period),
		q_0_(start),
		g_(goal),
		T_(g_.when().as_seconds()-q_0_.when().as_seconds()),
		path_dim_(start.get_dim())
	{
		if (!check_param_dim()) {
			LOG(Warning) << "Path dimensions of input parameters to DynamicMotionPrimitive are inconsistent. Parameters not set.";
			clear();
			return;
		}

		if (g_.when() <= q_0_.when()) {
			LOG(Warning) << "Goal WayPoint must be at a time after start WayPoint. Parameters not set.";
			clear();
			return;
		}

		set_timing_parameters();

		generate_trajectory();
	}

	const Trajectory& MinimumJerk::trajectory() {
		return trajectory_;
	}

	const Trajectory& MinimumJerk::update(const std::vector<double> &theta) {
		generate_trajectory();
		return trajectory_;
	}

	void MinimumJerk::clear() {
		Ts_ = Time::Zero;
		q_0_.clear();
		g_.clear();
		tau_ = double();
		path_dim_ = 0;
		path_size_ = 0;
		trajectory_.clear();
	}

	bool MinimumJerk::set_start(const WayPoint &start) {
		if (g_.when() <= start.when()) {
			LOG(Warning) << "Goal WayPoint must be at a time after start WayPoint. Parameters not set.";
			return false;
		}
		if (start.get_dim() != path_dim_) {
			LOG(Warning) << "Path dimensions of input parameters to DynamicMotionPrimitive::set_start() are inconsistent. Parameters not set.";
			return false;
		}
		q_0_ = start;
		set_timing_parameters();
		generate_trajectory();
		return true;
	}

	bool MinimumJerk::set_goal(const WayPoint &goal) {
		if (goal.when() <= q_0_.when()) {
			LOG(Warning) << "Goal WayPoint must be at a time after start WayPoint. Parameters not set.";
			return false;
		}
		if (goal.get_dim() != path_dim_) {
			LOG(Warning) << "Path dimensions of input parameters to DynamicMotionPrimitive::set_goal() are inconsistent. Parameters not set.";
			return false;
		}
		g_ = goal;
		set_timing_parameters();
		generate_trajectory();
		return true;
	}

	bool MinimumJerk::set_endpoints(const WayPoint &start, const WayPoint &goal) {
		if (goal.when() <= start.when()) {
			LOG(Warning) << "Goal WayPoint must be at a time after start WayPoint. Parameters not set.";
			return false;
		}
		if (start.get_dim() != path_dim_ || goal.get_dim() != path_dim_) {
			LOG(Warning) << "Path dimensions of input parameters to DynamicMotionPrimitive::set_endpoints() are inconsistent. Parameters not set.";
			return false;
		}
		q_0_ = start;
		g_ = goal;
		T_ = g_.when().as_seconds() - q_0_.when().as_seconds();
		set_timing_parameters();
		generate_trajectory();
		return true;
	}

	void MinimumJerk::set_trajectory_params(Trajectory::Interp interp_method, const std::vector<double> &max_diff) {
		trajectory_.set_interp_method(interp_method);
		trajectory_.set_max_diff(max_diff);
	}

	double MinimumJerk::get_tau() const {
		return tau_;
	}

	bool MinimumJerk::check_param_dim() {
		if (g_.get_dim() != path_dim_) {
			return false;
		}
		return true;
	}

	void MinimumJerk::set_timing_parameters() {
		tau_ = g_.when().as_seconds() - q_0_.when().as_seconds();
		if (tau_ != (int)(tau_ / Ts_.as_seconds()) * Ts_.as_seconds()) {
			tau_ = (int)(tau_ / Ts_.as_seconds()) * Ts_.as_seconds();
			g_.set_time(q_0_.when() + seconds(tau_));
			LOG(Warning) << "Trajectory duration not evenly divisible by sample period. Shortening trajectory duration.";
		}
		path_size_ = (std::size_t)((unsigned)std::floor(tau_ / Ts_.as_seconds())) + 1;
		times_ = linspace(q_0_.when().as_seconds(), g_.when().as_seconds(), path_size_);
	}

	void MinimumJerk::generate_trajectory() {

		std::vector<double> a0_ = q_0_.get_pos();
		std::vector<double> a1_(path_dim_,0.0);
		std::vector<double> a2_(path_dim_,0.0);
		std::vector<double> a3_(path_dim_,0.0);
		std::vector<double> a4_(path_dim_,0.0);
		std::vector<double> a5_(path_dim_,0.0);

		std::vector<std::vector<double>> a_ { a0_, a1_, a2_, a3_, a4_, a5_ };

		double xT2 = 0.0;
		double xT3 = 0.0;

		for (size_t i = 0; i < path_dim_; i++)
		{
			a_[3][i] = -(20 * a0_[i] + 20 * a1_[i] + 20 * a2_[i] - 20 * g_.get_pos()[i] - 8 * T_*a1_[i] - 16 * T_*a2_[i] + 8 * T_*xT2 + 2 * pow(T_,2) * a2_[i] - pow(T_,2) * xT3) / (2 * pow(T_,3));
			a_[4][i] = (15 * a0_[i] + 15 * a1_[i] + 15 * a2_[i] - 15 * g_.get_pos()[i] - 7 * T_*a1_[i] - 14 * T_*a2_[i] + 7 * T_*xT2 + 2 * pow(T_,2) * a2_[i] - pow(T_,2) * xT3) / pow(T_,4);
			a_[5][i] = -(12 * a0_[i] + 12 * a1_[i] + 12 * a2_[i] - 12 * g_.get_pos()[i] - 6 * T_*a1_[i] - 12 * T_*a2_[i] + 6 * T_*xT2 + 2 * pow(T_,2) * a2_[i] - pow(T_,2) * xT3) / (2 * pow(T_,5));
		}
		
		// reset
		trajectory_.resize(path_size_);
		current_time_idx_ = 0;

		// forward integration of states
		for (std::size_t i = 0; i < path_size_; ++i) {
			
			std::vector<double> traj_current_(path_dim_,0.0);

			for (size_t j = 0; j < path_dim_; j++)
			{
				for (size_t k = 0; k < a_.size(); k++)
				{
					traj_current_[j] += a_[k][j] * pow(times_[i], k);
				}
			}

			trajectory_.add_waypoint(i, WayPoint(seconds(times_[current_time_idx_]), traj_current_));
			current_time_idx_++;

			//s_ = std::exp(-gamma_ * (times_[current_time_idx_] - times_[0]) / tau_);
			//if (has_field_) {
			//	q_ddot_mat_ = (K_ * (g_mat_ - q_mat_) - D_ * q_dot_mat_ * tau_ - K_ * (g_mat_ - q_0_mat_) * s_ + K_ * nonlin_function_(q_mat_, theta_mat_) * s_) * (1 / (tau_ * tau_));
			//}
			//else {
			//	q_ddot_mat_ = (K_ * (g_mat_ - q_mat_) - D_ * q_dot_mat_ * tau_ - K_ * (g_mat_ - q_0_mat_) * s_) * (1 / (tau_ * tau_));
			//}
			//for (std::size_t j = 0; j < path_dim_; ++j) {
			//	q_mat_(j) = integrator_[j].update(q_dot_mat_(j), seconds(times_[current_time_idx_]));
			//}
			//for (std::size_t j = 0; j < path_dim_; ++j) {
			//	q_dot_mat_(j) = integrator_[j + path_dim_].update(q_ddot_mat_(j), seconds(times_[current_time_idx_]));
			//}
			//trajectory_.add_waypoint(current_time_idx_, WayPoint(seconds(times_[current_time_idx_]), q_mat_.get_col(0)));
			//current_time_idx_++;
		}

		if (!trajectory_.validate()) {
			LOG(Error) << "Trajectory generated by DMP was invalid.";
			return;
		}
	}

}