#include <MEII/PhriLearning/DynamicMotionPrimitive.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Math/Functions.hpp>

using namespace mel;

namespace meii {

    DynamicMotionPrimitive::DynamicMotionPrimitive(mel::Time &sample_period, WayPoint &start, WayPoint &goal, Matrix &K, Matrix &D, double tau, Matrix(*nonlin_function)(const Matrix&, const Matrix&), std::vector<double> &theta, std::vector<double> &goal_tol) :
        Ts_(sample_period),
        q_0_(start),
        g_(goal),
        K_(K),
        D_(D),
        tau_(tau),
        nonlin_function_(nonlin_function),
        theta_(theta),
        s_(1.0),
        goal_tol_(goal_tol),
        path_dim_(start.get_dim()),
        integrator_(path_dim_, Integrator(0.0, Integrator::Technique::Trapezoidal))
    {
        if (!check_param_dim()) {
            LOG(Warning) << "Path dimensions of input parameters to DynamicMotionPrimitive are inconsistent. Parameters not set.";
            clear();
            return;
        }
        
        //path_size_ = (std::size_t)((unsigned)(std::ceil((g_.when().as_seconds() - q_0_.when().as_seconds()) / Ts_.as_seconds()))) + 1;
        //print(std::ceil((q_0_.when().as_seconds() - g_.when().as_seconds()) / Ts_.as_seconds()));
        //t_ = linspace(q_0_.when().as_seconds(), g_.when().as_seconds(), path_size_);
        //print(t_);
        //q_ = q_0_.get_pos();
        std::vector<double> new_q(path_dim_);
        for (std::size_t i = 0; i < path_dim_; ++i) {
            integrator_[i].set_init(q_0_[i]);
            
        }

        step();

    }

    Trajectory DynamicMotionPrimitive::trajectory() {
        return trajectory_;
    }

    void DynamicMotionPrimitive::update(std::vector<double> theta) {

    }

    void DynamicMotionPrimitive::clear() {
        q_0_.clear();
        g_.clear();
        K_.clear();
        D_.clear();
        tau_ = double();
        s_ = double();
        path_dim_ = 0;
        integrator_.clear();
    }

    bool DynamicMotionPrimitive::check_param_dim() {
        if (g_.get_dim() != path_dim_) {
            return false;
        }
        if (K_.rows() != path_dim_ || K_.cols() != path_dim_) {
            return false;
        }
        if (D_.rows() != path_dim_ || D_.cols() != path_dim_) {
            return false;
        }
        if (goal_tol_.size() != path_dim_) {
            if (goal_tol_.size() == 1) {
                goal_tol_ = std::vector<double>(path_dim_, goal_tol_[0]);
            }
            else {
                LOG(Warning) << "Input goal_tol given to DynamicMotionPrimitive constructor must either be of size 1 or of size path_dim.";
                return false;
            }
        }
    }

    void DynamicMotionPrimitive::step() {
        Matrix mat(path_dim_, 1);
        G_ = Matrix(g_.get_pos());
        Q_0_ = Matrix(q_0_.get_pos());
        Q_ = Matrix(q_0_.get_pos());
        Q_dot_ = Matrix(path_dim_, 1, 0.0);
        Matrix theta_mat = Matrix(theta_);
        mat = (K_ * (G_ - Q_) - D_ * Q_dot_ * tau_ - K_ * (G_ - Q_0_) * s_);// +K_ * nonlin_function_(Q_, theta_mat) * s_) * (1 / (tau_ * tau_));
        //std::vector<double> vec(path_dim_);
        //for (std::size_t i = 0; i < path_dim_; ++i) {
        //    vec[i] = g_[i] - q_[i];
        //}
        //print(g_.get_pos());
        //print(q_);
        //print(vec);
        //vec = K_ * vec;
        //print(vec);
        std::cout << mat << std::endl;
    }

}