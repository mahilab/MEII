#include <MEII/PhriLearning/DynamicMotionPrimitive.hpp>
#include <MEII/PhriLearning/PhriFeatures.hpp>
#include <MEII/Control/Trajectory.hpp>
#include <MEII/MahiExoII/MahiExoII.hpp>
#include <MEII/Utility/Matrix.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Logging/DataLogger.hpp>
#include <MEL/Core/Timer.hpp>


using namespace mel;
using namespace meii;

int main() {

    // initialize default logger
    init_logger();

	// construct a clock for measuring computation times
	Clock clock;

	// construct a Dynamic Motion Primitive (DMP) that could be used by the MAHI Exo-II, with the feature gradient
    Time dmp_Ts = milliseconds(50);
    std::size_t dim = MahiExoII::N_aj_;
    WayPoint start(Time::Zero, std::vector<double>(dim, 0.0));
    WayPoint goal(seconds(5), { -1.2, 0.75, 0.17, -0.17, 0.1 });   
    std::vector<double> theta = { 0.0 };
	clock.restart();
    DynamicMotionPrimitive dmp_field(dmp_Ts, start, goal, &feature_gradient, theta);
	Time comp_time = clock.get_elapsed_time();
	std::cout << "DMP construction time was " << comp_time << std::endl;

	// save the trajectory generated by the DMP
	Table dmp_log("DmpTrajectory");
	dmp_log.set_col_names({"Time (s)", "q_0", "q_1", "q_2", "q_3" ,"q_4"});
	for (std::size_t i = 0; i < dmp_field.trajectory().size(); ++i) {
		dmp_log.push_back_row(dmp_field.trajectory()[i].get_point());
	}
	DataLogger::write_to_csv(dmp_log, "ex_dmp_log_0.csv", ".", false);


	// update DMP with new theta value
	theta[0] = 1.0;
	clock.restart();
	dmp_field.update(theta);
	comp_time = clock.get_elapsed_time();
	std::cout << "DMP update time was " << comp_time << std::endl;

	// save the trajectory generated by the DMP
	dmp_log.clear_values();
	for (std::size_t i = 0; i < dmp_field.trajectory().size(); ++i) {
		dmp_log.push_back_row(dmp_field.trajectory()[i].get_point());
	}
	DataLogger::write_to_csv(dmp_log, "ex_dmp_log_1.csv", ".", false);

	// construct a Dynamic Motion Primitive (DMP) that could be used by the MAHI Exo-II, without the feature gradient
	dmp_Ts = milliseconds(50);
	dim = MahiExoII::N_aj_;
	start = WayPoint(Time::Zero, { -1.2, 0.75, 0.17, -0.17, 0.1 });
	goal = WayPoint(seconds(5), { -1.2, 0.75, 0.17, -0.17, 0.1 });
	clock.restart();
	DynamicMotionPrimitive dmp(dmp_Ts, start, goal);
	comp_time = clock.get_elapsed_time();
	std::cout << "DMP construction time was " << comp_time << std::endl;

	// save the trajectory generated by the DMP
	dmp_log.clear_values();
	for (std::size_t i = 0; i < dmp.trajectory().size(); ++i) {
		dmp_log.push_back_row(dmp.trajectory()[i].get_point());
	}
	DataLogger::write_to_csv(dmp_log, "ex_dmp_log_2.csv", ".", false);


	//// update the DMP in real time
	//enable_realtime();
	//Timer timer(milliseconds(1), Timer::Hybrid);
	//theta[0] = 0.0;
	//for (std::size_t i = 0; i < 1000; ++i) {

	//	// update theta
	//	theta[0] += 0.001;

	//	// update the DMP trajectory
	//	dmp.update(theta);

	//	// wait for remainder of sample period
	//	timer.wait();
	//}
	//disable_realtime();

    return 0;
}