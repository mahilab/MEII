#include <MEII/PhriLearning/DynamicMotionPrimitive.hpp>
#include <MEII/PhriLearning/PhriFeatures.hpp>
#include <MEII/Control/Trajectory.hpp>
#include <MEII/MahiExoII/MahiExoII.hpp>
#include <MEII/Utility/Matrix.hpp>
#include <MEL/Logging/Log.hpp>

using namespace mel;
using namespace meii;

int main() {

    // initialize default logger
    init_logger();

    mel::Time Ts = milliseconds(1);
    std::size_t dim = MahiExoII::N_aj_;
    WayPoint start(Time::Zero, std::vector<double>(dim, 0.0));
    WayPoint goal(seconds(5), { -1.2, 0.75, 0.17, -0.17, 0.1 });
    std::vector<double> goal_tol = {0.01};
    Matrix K(dim);
    Matrix D(dim);
    for (std::size_t i = 0; i < dim; ++i) {
        K(i, i) = 1.0;
        D(i, i) = 1.0;
    }
    double tau = 0.5;   
    std::vector<double> theta = { 1.0 };
    DynamicMotionPrimitive dmp(Ts, start, goal, K, D, tau, &feature_gradient, theta, goal_tol);

    return 0;
}