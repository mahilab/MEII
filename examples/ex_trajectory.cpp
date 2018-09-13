#include <MEII/Control/Trajectory.hpp>
#include <MEII/MahiExoII/MahiExoII.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Utility/System.hpp>

using namespace mel;
using namespace meii;

int main() {

    // construct waypoints vector
    std::vector<WayPoint> waypoints;
    waypoints.push_back(WayPoint(seconds(0.0), { 1.0, 2.0, 3.0 }));
    waypoints.push_back(WayPoint(seconds(1.0), { 2.0, 4.0, 6.0 }));
    waypoints.push_back(WayPoint(seconds(2.0), { 5.0, 5.0, 8.0 }));
    waypoints.push_back(WayPoint(seconds(2.5), { 6.0, 6.0, 6.0 }));
    std::size_t path_dim = waypoints[0].get_dim();

    LOG(Info) << "Display waypoints:";
    for (std::size_t j = 0; j < waypoints.size(); ++j) {
        print(waypoints[j].when());
        for (std::size_t i = 0; i < waypoints[j].get_dim(); ++i) {
            print(waypoints[j][i]);
        }
        print("");
    }

    Time interp_time = seconds(0.5);
    LOG(Info) << "Linear time interpolation between first two waypoints at " + stringify(interp_time.as_seconds()) + " s:";
    WayPoint interp_point = Trajectory::linear_time_interpolate(waypoints[0], waypoints[1], seconds(0.5));
    print(interp_point.when());
    for (std::size_t i = 0; i < interp_point.get_dim(); ++i) {
        print(interp_point[i]);
    }
    print("");

    std::size_t num_waypoints = 11;
    LOG(Info) << "Generate " + stringify(num_waypoints) + " linearly spaced waypoints between first two waypoints:";
    std::vector<WayPoint> interp_points = Trajectory::linspace_points(waypoints[0], waypoints[1], 11);
    for (std::size_t i = 0; i < interp_points.size(); ++i) {
		std::cout << interp_points[i];
        print("");
    }

    Time Ts = milliseconds(50);
    Time t = milliseconds(0);
    std::vector<double> max_diff = { 3.0, 2.0, 4.0 };   
    Trajectory trajectory(path_dim, waypoints, Trajectory::Interp::Linear, max_diff);
	LOG(Info) << "Generate trajectory passing through all waypoints and sample at a rate of " << Ts;
    print("");
    if (!trajectory.empty() && trajectory.validate()) {
        for (std::size_t i = 0; i < 51; ++i) {
            print(trajectory.at_time(t));
            t += Ts;
        }
    }
	print("");


	LOG(Info) << "Add waypoints to trajectory and validate.";
	trajectory.resize(trajectory.size() + 2);
	trajectory.add_waypoint(trajectory.size() - 2, WayPoint(seconds(3.0), { 4.5, 5, 4 }));
	trajectory.add_waypoint(trajectory.size() - 1, WayPoint(seconds(5.0), {-1.5, 2, 3}));
	trajectory.validate();
	std::cout << trajectory;

   

    return 0;
}