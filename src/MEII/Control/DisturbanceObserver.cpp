#include <MEII/Control/DisturbanceObserver.hpp>
#include <Mahi/Util/Logging/Log.hpp>

using namespace mahi::util;

namespace meii {

	DisturbanceObserver::DisturbanceObserver(Time Ts_):
		z(0),
		c(20),
		J(0.2084),// + 0.0693),
        C(0.1215),
        d_hat(0),
        Ts(Ts_)
	{
        L = c/J;
        Butterworth butt(2,hertz(10),hertz(1000));
    }

    DisturbanceObserver::DisturbanceObserver(Time Ts_, double z_0):
		z(z_0),
		c(20),
		J(0.2084),// + 0.0693),
        C(0.1215),
        d_hat(0),
        Ts(Ts_)
	{
        L = c/J;
        Butterworth butt(2,hertz(10),hertz(1000));
    }

	void DisturbanceObserver::update(const double x, const double x_dot, const double T_command, const double delta_t, const Time &t) {
        double p = c*x_dot;
        double d_hat_unf = z + p;
        d_hat = butt.update(d_hat_unf);
        double z_dot = -L*z+L*(-C*x_dot - T_command - p);
        
        z = z+z_dot*delta_t;
	}
    
    double DisturbanceObserver::get_d_hat() const{
        return d_hat;
    }

}