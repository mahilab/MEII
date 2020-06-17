#include <MEII/Control/DynamicMotionPrimitive.hpp>
#include <Mahi/Util/Logging/Log.hpp>
#include <Mahi/Util/Math/Functions.hpp>

using namespace mahi::util;

namespace meii {

	DynamicMotionPrimitive::DynamicMotionPrimitive(const Time &sample_period, const WayPoint &start, const WayPoint &goal, double gamma) :
		Ts_(sample_period),
		q_0_(start),
		g_(goal),
		gamma_(gamma),
		s_(1.0),
		path_dim_(start.get_dim()),
		current_time_idx_(0),
		integrator_(2 * path_dim_, Integrator(0.0, Integrator::Technique::Trapezoidal)),
		q_dot_mat_(Eigen::VectorXd::Zero(path_dim_)),
		q_ddot_mat_(Eigen::VectorXd::Zero(path_dim_))
	{
		K_ = Eigen::MatrixXd::Zero(path_dim_,path_dim_);
		D_ = Eigen::MatrixXd::Zero(path_dim_,path_dim_);

		q_0_mat_ = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(q_0_.get_pos().data(), q_0_.get_pos().size());
		g_mat_ = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(g_.get_pos().data(), g_.get_pos().size());
		q_mat_ = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(q_0_.get_pos().data(), q_0_.get_pos().size());

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

		// if K_ and D_ empty use defaults
		for (std::size_t i = 0; i < path_dim_; ++i) {
			K_(i, i) = 25.0 * 25.0 / 4.0;
			D_(i, i) = 25.0;
		}

		generate_trajectory();

	}

    const Trajectory& DynamicMotionPrimitive::trajectory() {
        return trajectory_;
    }

	const Trajectory& DynamicMotionPrimitive::update(const std::vector<double> &theta) {
		theta_mat_ = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(theta.data(), theta.size());
		generate_trajectory();	
		return trajectory_;
    }

    void DynamicMotionPrimitive::clear() {
		Ts_ = Time::Zero;
        q_0_.clear();
        g_.clear();
        K_ = Eigen::MatrixXd::Zero(path_dim_,path_dim_);
        D_ = Eigen::MatrixXd::Zero(path_dim_,path_dim_);
		gamma_ = double();
        tau_ = double();
        s_ = double();
        path_dim_ = 0;
		path_size_ = 0;
        integrator_.clear();
		q_0_mat_ = Eigen::VectorXd::Zero(path_dim_);
		g_mat_ = Eigen::VectorXd::Zero(path_dim_);
		q_mat_ = Eigen::VectorXd::Zero(path_dim_);
		q_dot_mat_ = Eigen::VectorXd::Zero(path_dim_);
		q_ddot_mat_ = Eigen::VectorXd::Zero(path_dim_);
		theta_mat_ = Eigen::VectorXd::Zero(path_dim_);
		trajectory_.clear();
    }

	bool DynamicMotionPrimitive::set_start(const WayPoint &start)  {
		if (g_.when() <= start.when()) {
			LOG(Warning) << "Goal WayPoint must be at a time after start WayPoint. Parameters not set.";
			return false;
		}
		if (start.get_dim() != path_dim_) {
			LOG(Warning) << "Path dimensions of input parameters to DynamicMotionPrimitive::set_start() are inconsistent. Parameters not set.";
			return false;
		}
		q_0_ = start;
		q_0_mat_ = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(q_0_.get_pos().data(), q_0_.get_pos().size());
		set_timing_parameters();
		generate_trajectory();
		return true;
	}

	bool DynamicMotionPrimitive::set_goal(const WayPoint &goal) {
		if (goal.when() <= q_0_.when()) {
			LOG(Warning) << "Goal WayPoint must be at a time after start WayPoint. Parameters not set.";
			return false;
		}
		if (goal.get_dim() != path_dim_) {
			LOG(Warning) << "Path dimensions of input parameters to DynamicMotionPrimitive::set_goal() are inconsistent. Parameters not set.";
			return false;
		}
		g_ = goal;
		g_mat_ = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(g_.get_pos().data(), g_.get_pos().size());
		set_timing_parameters();
		generate_trajectory();
		return true;
	}

	bool DynamicMotionPrimitive::set_endpoints(const WayPoint &start, const WayPoint &goal) {
		if (goal.when() <= start.when()) {
			LOG(Warning) << "Goal WayPoint must be at a time after start WayPoint. Parameters not set.";
			return false;
		}
		if (start.get_dim() != path_dim_ || goal.get_dim() != path_dim_) {
			LOG(Warning) << "Path dimensions of input parameters to DynamicMotionPrimitive::set_endpoints() are inconsistent. Parameters not set.";
			return false;
		}
		q_0_ = start;
		q_0_mat_ = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(q_0_.get_pos().data(), q_0_.get_pos().size());
		g_ = goal;
		g_mat_ = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(g_.get_pos().data(), g_.get_pos().size());
		set_timing_parameters();
		generate_trajectory();
		return true;
	}

	void DynamicMotionPrimitive::set_trajectory_params(Trajectory::Interp interp_method, const std::vector<double> &max_diff) {
		trajectory_.set_interp_method(interp_method);
		trajectory_.set_max_diff(max_diff);
	}

	double DynamicMotionPrimitive::get_gamma() const {
		return gamma_;
	}

	double DynamicMotionPrimitive::get_tau() const {
		return tau_;
	}

    bool DynamicMotionPrimitive::check_param_dim() {
        if (g_.get_dim() != path_dim_) {
			LOG(Warning) << "g_.get_dim() != path_dim_";
            return false;
        }
        if (K_.rows() != path_dim_ || K_.cols() != path_dim_) {
			LOG(Warning) << "K_.rows() != path_dim_ || K_.cols() != path_dim_";
			LOG(Warning) << K_.rows() << ", " << K_.cols() << ", " << path_dim_;
            return false;
        }
        if (D_.rows() != path_dim_ || D_.cols() != path_dim_) {
			LOG(Warning) << "D_.rows() != path_dim_ || D_.cols() != path_dim_";
            return false;
        }
		return true;
    }

	void DynamicMotionPrimitive::set_timing_parameters() {
		tau_ = g_.when().as_seconds() - q_0_.when().as_seconds();
		if (tau_ != (int)(tau_ / Ts_.as_seconds()) * Ts_.as_seconds()) {
			tau_ = (int)(tau_ / Ts_.as_seconds()) * Ts_.as_seconds();
			g_.set_time(q_0_.when() + seconds(tau_));
			LOG(Warning) << "Trajectory duration not evenly divisible by sample period. Shortening trajectory duration.";
		}
		path_size_ = (std::size_t)((unsigned)std::floor(tau_ / Ts_.as_seconds())) + 1;
		// times_ = linspace(q_0_.when().as_seconds(), g_.when().as_seconds(), path_size_);
		times_.resize(path_size_);
		linspace(q_0_.when().as_seconds(), g_.when().as_seconds(), times_);
	}

	void DynamicMotionPrimitive::generate_trajectory() {
		// reset
		trajectory_.resize(path_size_);
		current_time_idx_ = 0;
		for (std::size_t i = 0; i < integrator_.size(); ++i) {
			integrator_[i].reset();
		}

		// initial conditions
		q_mat_ = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(q_0_.get_pos().data(), q_0_.get_pos().size());
		q_dot_mat_ = Eigen::VectorXd::Zero(path_dim_);
		q_ddot_mat_ = Eigen::VectorXd::Zero(path_dim_);
		for (std::size_t i = 0; i < path_dim_; ++i) {
			integrator_[i].set_init(q_mat_(i));
			integrator_[i + path_dim_].set_init(q_dot_mat_(i));
		}

		// forward integration of states
		for (std::size_t i = 0; i < path_size_; ++i) {
			s_ = std::exp(-gamma_ * (times_[current_time_idx_] - times_[0]) / tau_);
			q_ddot_mat_ = (K_ * (g_mat_ - q_mat_) - D_ * q_dot_mat_ * tau_ - K_ * (g_mat_ - q_0_mat_) * s_) * (1 / (tau_ * tau_));
			for (std::size_t j = 0; j < path_dim_; ++j) {
				q_mat_(j) = integrator_[j].update(q_dot_mat_(j), seconds(times_[current_time_idx_]));
			}
			for (std::size_t j = 0; j < path_dim_; ++j) {
				q_dot_mat_(j) = integrator_[j + path_dim_].update(q_ddot_mat_(j), seconds(times_[current_time_idx_]));
			}
			std::vector<double> vec(q_mat_.data(), q_mat_.data() + q_mat_.rows() * q_mat_.cols());
			trajectory_.add_waypoint(current_time_idx_, WayPoint(seconds(times_[current_time_idx_]), vec));
			current_time_idx_++;
		}

		if (!trajectory_.validate()) {
			LOG(Error) << "Trajectory generated by DMP was invalid.";
			return;
		}
	}

}