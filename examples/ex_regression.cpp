#include <MEL/Core/Timer.hpp>
#include <MEL/Core/Clock.hpp>
#include <MEL/Core/Console.hpp>
#include <MEL/Utility/System.hpp>
#include <MEL/Utility/Options.hpp>
#include <MEL/Devices/Windows/Keyboard.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Communications/MelShare.hpp>
#include <MEL/Math/Butterworth.hpp>
#include <MEII/Regression/RealTimeRegressor.hpp>
#include <MEL/Logging/DataLogger.hpp>
#include <chrono>
#include <random>

using namespace mel;
using namespace meii;

ctrl_bool stop(false);
bool handler(CtrlEvent event) {
    stop = true;
    return true;
}

int main(int argc, char *argv[]) {

    // make options
    Options options("ex_regression.exe", "Example of Real-Time Regression");
	options.add_options()
		("u,uni", "Demonstrates univariate real-time regressor")
		("m,multi", "Demonstrates multivariate real-time regressor");

    auto result = options.parse(argc, argv);

    // enable Windows realtime
    enable_realtime();

    // register ctrl-c handler
    register_ctrl_handler(handler); 
   
	// initialize testing conditions
	//bool auto_train = false;
	//bool load_classifier = true;
	std::size_t training_data_size = 50;
	Time Ts = milliseconds(1); // sample period
	std::size_t sample_dim = 3; // size of each sample
	std::vector<double> a = { 1.5, 0.5, 3.0 }; // signal amplitudes
	std::vector<double> f = { 1.0, 0.3, 2.2 }; // signal frequencies  
	double n = 0.5; // noise amplitude   
	Butterworth lpf(2, 0.001);


	if (result.count("uni") > 0) {

	}

	if (result.count("multi") > 0) {

	}

    disable_realtime();
    return 0;
}

