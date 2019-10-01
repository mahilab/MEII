#include <MEII/Control/DisturbanceObserver.hpp>
#include <MEL/Logging/Log.hpp>

using namespace mel;

namespace meii {

	DisturbanceObserver::DisturbanceObserver(mel::Time Ts_):
		z(0),
		c(10),
		J(0.2084 + 0.0693),
        d_hat(0),
        Ts(Ts_)
	{}

    DisturbanceObserver::DisturbanceObserver(mel::Time Ts_, double z_0):
		z(z_0),
		c(10),
		J(0.2084 + 0.0693),
        d_hat(0),
        Ts(Ts_)
	{}

	void DisturbanceObserver::update(const double &x_dot, const double &T_prev) {
        double p = c*x_dot;
        double L = c/J;
        d_hat = z + p;
        double z_dot = -L*z+L*(-T_prev - p);
        z = z+z_dot*Ts.as_seconds();
	}
    
    double DisturbanceObserver::get_d_hat() const{
        return d_hat;
    }

}