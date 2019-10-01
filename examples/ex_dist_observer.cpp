#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEII/MahiExoII/MahiExoII.hpp>
#include <MEL/Utility/System.hpp>
#include <MEL/Communications/MelShare.hpp>
#include <MEL/Utility/Options.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Logging/Csv.hpp>
#include <MEL/Core/Console.hpp>
#include <MEL/Devices/Windows/Keyboard.hpp>
#include <MEII/Control/Trajectory.hpp>
#include <MEII/Control/DynamicMotionPrimitive.hpp>
#include <MEII/Control/MinimumJerk.hpp>
#include <MEL/Math/Integrator.hpp>
#include <MEII/OpenSim/osim_utility.hpp>
#include <MEII/Control/DisturbanceObserver.hpp>
#include <vector>

using namespace mel;
using namespace meii;


// create global stop variable CTRL-C handler function
ctrl_bool stop(false);
bool handler(CtrlEvent event) {
	stop = true;
	return true;
}

int main(int argc, char *argv[]) {

    register_ctrl_handler(handler);

	// make options
	Options options("ex_pos_control_nathan.exe", "Nathan's Position Control Demo");
	options.add_options()
		("c,calibrate", "Calibrates the MAHI Exo-II")
		("s,single", "MAHI Exo-II follows a single-DoF trajectory with different trajectoy generators")
		("m,multi", "MAHI Exo-II follows a multi-DoF trajectory with different trajectoy generators")
		("i,int","Enter an interger", value<int>())
		("h,help", "Prints this help message");

	auto result = options.parse(argc, argv);

	// if -h, print the help option
	if (result.count("help") > 0) {
		print(options.help());
		return 0;
	}

	// enable Windows realtime
	enable_realtime();

	// register ctrl-c handler
	register_ctrl_handler(handler);

	// sample period for DAQ
	Time Ts = milliseconds(1); 

	// make MelShares
	MelShare ms_pos("ms_pos");
	MelShare ms_vel("ms_vel");
	MelShare ms_trq("ms_trq");
	MelShare ms_ref("ms_ref");

	// trajectory following
	if (result.count("single") > 0) {
		LOG(Info) << "MAHI Exo-II Trajectory Following.";

		// create ranges
		std::vector<std::vector<double>> setpoint_rad_ranges = { { -90 * DEG2RAD, 0 * DEG2RAD },
		{ -90 * DEG2RAD, 90 * DEG2RAD },
		{ -15 * DEG2RAD, 15 * DEG2RAD },
		{ -15 * DEG2RAD, 15 * DEG2RAD },
		{ 0.08, 0.115 } };

		// construct timer in hybrid mode to avoid using 100% CPU
		Timer timer(Ts, Timer::Hybrid);
		Timer FuncTimer(Ts, Timer::Hybrid);

		// construct clock for regulating keypress
		Clock keypress_refract_clock;
		Time keypress_refract_time = seconds(0.5);

        enum Phase {
            Backdrive, // Backdrive = 0
            Move,      // Move      = 1
            Stop       // Stop      = 2
        };

        Phase DO_Phase = Backdrive;

		// define DOFs
		enum DoF {
			ElbowFE, // ElbowFE = 0 by default
			WristPS, // WristPS = 1
			WristFE, // WristFE = 2
			WristRU, // WristRU = 3
			LastDoF
		};

		// define DOFs
		enum TrajType {
			traj_dmp, // dynamic motion primitive trajectory
			traj_mj, // minimum jerk trajectory
			traj_linear // linear trajectory
		};

		bool save_data = false;
		std::string filepath = "example_DO_robot_data_log.csv";

		// construct robot data log
		std::vector<double> robot_log_row(6);
		std::vector<std::vector<double>> robot_log;
		std::vector<std::string> header = { "Time [s]", "ref 1 [rad/s]", "ref 2 [rad/s]",  "ref 3 [rad/s]", "ref 4 [rad/s]", "ref 5 [rad/s]" };

		// setup trajectories
		WayPoint neutral_point = WayPoint(Time::Zero,{ -35 * DEG2RAD, 00 * DEG2RAD, 00 * DEG2RAD, 00 * DEG2RAD, 0.09 });

		std::vector<WayPoint> extreme_points = { WayPoint(Time::Zero,{ -05 * DEG2RAD, 00 * DEG2RAD, 00 * DEG2RAD, 00 * DEG2RAD, 0.09 }), 
			  									  WayPoint(Time::Zero,{ -65 * DEG2RAD, 00 * DEG2RAD, 00 * DEG2RAD, 00 * DEG2RAD, 0.09 })};

		Time dmp_duration = seconds(3.0);
		
		std::vector<double> traj_max_diff = { 50 * mel::DEG2RAD, 50 * mel::DEG2RAD, 25 * mel::DEG2RAD, 25 * mel::DEG2RAD, 0.1 };
		Time time_to_start = seconds(3.0);
		Time dmp_Ts = milliseconds(50);

		// default dmp traj
		DynamicMotionPrimitive dmp(dmp_Ts, neutral_point, extreme_points[0].set_time(dmp_duration));
		dmp.set_trajectory_params(Trajectory::Interp::Linear, traj_max_diff);

		// Initializing variables for linear travel
		WayPoint initial_waypoint;
		std::vector<WayPoint> waypoints(2);
		Trajectory ref_traj;
		WayPoint current_wp;
		WayPoint next_wp;

		//default mj traj
		MinimumJerk mj(dmp_Ts, neutral_point, extreme_points[0].set_time(dmp_duration));
		mj.set_trajectory_params(Trajectory::Interp::Linear, traj_max_diff);

		// Initializing variables
		DoF dof = ElbowFE; // default
		bool traj_selected = false;

		TrajType traj_type = ;
		std::size_t current_extreme_idx = 0;

		// construct clocks for waiting and trajectory
		Clock state_clock;
		Clock ref_traj_clock;

		// set up state machine
		std::size_t state = 0;
		Time backdrive_time = seconds(1);
		Time wait_at_neutral_time = seconds(1);
		Time wait_at_extreme_time = seconds(1);


		// create data containers
		std::vector<double> ref = { -35 * DEG2RAD, 00 * DEG2RAD, 00 * DEG2RAD, 00 * DEG2RAD, 0.09 };

		// prompt user for input
		print("Press 'Escape' to exit the program.");
		print("Press 'Enter' to exit the program and save data.");

		print("Choose your trajectory type.");
		print("Press 'L' for a linear trajectory.");
		print("Press 'D' for a dmp trajectory.");
		print("Press 'M' for a minimum jerk trajectory.");

		// start loop
		LOG(Info) << "Robot Backdrivable.";
		/*q8.watchdog.start();*/
		state_clock.restart();
		while (!stop) {

			// begin switch state
			switch (state) {
				case 0: // backdrive

					// prompt user for input to select which trajectory
					if (!traj_selected) {

						// press D for dmp trajectory
						if (Keyboard::is_key_pressed(Key::D)) {
							traj_selected = true;
							traj_type = traj_dmp;
						}

						// press L for dmp trajectory
						if (Keyboard::is_key_pressed(Key::L)) {
							traj_selected = true;
							traj_type = traj_linear;
						}

						// press L for dmp trajectory
						if (Keyboard::is_key_pressed(Key::M)) {
							traj_selected = true;
							traj_type = traj_mj;
						}

						// check for exit key
						if (Keyboard::is_key_pressed(Key::Escape)) {
							stop = true;
							save_data = false;
						}
					}

					// Make sure trajectory is valid
					if (!dmp.trajectory().validate()) {
						LOG(Warning) << "DMP trajectory invalid.";
						return 0;
					}

					// check for wait period to end
					if (traj_selected) {

						traj_selected = false;

						// generate new trajectories
						if (traj_type == traj_linear)
						{
							waypoints[0] = neutral_point.set_time(Time::Zero);
							waypoints[1] = extreme_points[current_extreme_idx].set_time(dmp_duration);
							ref_traj.set_waypoints(5, waypoints, Trajectory::Interp::Linear, traj_max_diff);
						}
						else if (traj_type == traj_dmp)
						{
							dmp.set_endpoints(neutral_point.set_time(Time::Zero), extreme_points[current_extreme_idx].set_time(dmp_duration));
							ref_traj = dmp.trajectory();
						}
						else if (traj_type == traj_mj)
						{
							mj.set_endpoints(neutral_point.set_time(Time::Zero), extreme_points[current_extreme_idx].set_time(dmp_duration));
							ref_traj = mj.trajectory();
						}

						ref_traj_clock.restart();
						state = 1;
						LOG(Info) << "Going to Extreme position";
						state_clock.restart();
					}
					break;


				case 1: // go to extreme position

					ref = ref_traj.at_time(ref_traj_clock.get_elapsed_time());

					if (ref_traj_clock.get_elapsed_time() >= dmp.trajectory().back().when()) {
						
						// set the ref to the last point of the trajectory
						ref = ref_traj.back().get_pos();

						if (!dmp.trajectory().validate()) {
							LOG(Warning) << "DMP trajectory invalid.";
							stop = true;
						}

						state = 2;
						LOG(Info) << "Waiting at extreme position";
						state_clock.restart();
					}
					break;

				case 2: // wait at extreme position

					// update reference from trajectory

					if (state_clock.get_elapsed_time() > wait_at_extreme_time) {

						// generate new trajectories
						if (traj_type == "linear")
						{
							waypoints[0] = extreme_points[current_extreme_idx].set_time(Time::Zero);
							waypoints[1] = neutral_point.set_time(dmp_duration);
							ref_traj.set_waypoints(5, waypoints, Trajectory::Interp::Linear, traj_max_diff);
						}

						else if (traj_type == "dmp")
						{
							dmp.set_endpoints(extreme_points[current_extreme_idx].set_time(Time::Zero), neutral_point.set_time(dmp_duration));
							ref_traj = dmp.trajectory();
						}

						else if (traj_type == "min_jerk")
						{
							mj.set_endpoints(extreme_points[current_extreme_idx].set_time(Time::Zero), neutral_point.set_time(dmp_duration));
							ref_traj = mj.trajectory();
						}

						// change which extreme position will be visited next
						if (current_extreme_idx == 0)
						{
							current_extreme_idx++;
						}
						else
						{
							current_extreme_idx--;
						}
						
						state = 3;
						
						LOG(Info) << "Going to neutral position.";
						state_clock.restart();
						ref_traj_clock.restart();
					}
					break;

				case 3: // go to neutral position

					// move along reference trajectory for either linear or dmp
					ref = ref_traj.at_time(ref_traj_clock.get_elapsed_time());

					if (ref_traj_clock.get_elapsed_time() >= dmp.trajectory().back().when()) {

						// set the ref to the last point of the trajectory
						ref = ref_traj.back().get_pos();
						

						if (!dmp.trajectory().validate()) {
							LOG(Warning) << "DMP trajectory invalid.";
							stop = true;
						}

						LOG(Info) << "Waiting at neutral position.";

						state = 4;
						state_clock.restart();
					}
					break;

				case 4: // wait at neutral position

					// check if wait time has passed
					if (state_clock.get_elapsed_time() > wait_at_extreme_time) {

						// if only the first extreme has been visited
						if (current_extreme_idx == 1) {

							// generate new trajectories
							if (traj_type == "linear")
							{
								waypoints[0] = neutral_point.set_time(Time::Zero);
								waypoints[1] = extreme_points[current_extreme_idx].set_time(dmp_duration);
								ref_traj.set_waypoints(5, waypoints, Trajectory::Interp::Linear, traj_max_diff);
							}
							else if (traj_type == "dmp")
							{
								dmp.set_endpoints(neutral_point.set_time(Time::Zero), extreme_points[current_extreme_idx].set_time(dmp_duration));
								ref_traj = dmp.trajectory();
							}
							else if (traj_type == "min_jerk")
							{
								mj.set_endpoints(neutral_point.set_time(Time::Zero), extreme_points[current_extreme_idx].set_time(dmp_duration));
								ref_traj = mj.trajectory();
							}


							state = 1;
							LOG(Info) << "Going to extreme position.";
						}

						// if both extrema have been visited, prompt for next trajectory
						else {
							state = 0;

							LOG(Info) << "Waiting at neutral position for user input.";

							print("Press number key for selecting single DoF trajectory.");
							print("1 = Elbow Flexion/Extension");
							print("2 = Wrist Pronation/Supination");
							print("3 = Wrist Flexion/Extension");
							print("4 = Wrist Radial/Ulnar Deviation");
							print("Press 'Escape' to exit the program.");
						}

						state_clock.restart();
						ref_traj_clock.restart();
					}
					break;
			}

			
			// write ref to MelShares
			ms_ref.write_data(ref);
			
			// check for save key
			if (Keyboard::is_key_pressed(Key::Enter)) {
				stop = true;
				save_data = true;
			}
			
			// check for exit key
			if (Keyboard::is_key_pressed(Key::Escape)) {
				stop = true;
				save_data = false;
			}
			
			// store the time and ref data to log to a csv
			robot_log_row[0] = timer.get_elapsed_time().as_seconds();
			for (std::size_t i = 0; i < 5; ++i) {
				robot_log_row[i+1] = ref[i];
			}
			robot_log.push_back(robot_log_row);
			
			// wait for remainder of sample period
			timer.wait();
		}

		// save the data if the user wants
		if (save_data) {
			print("Do you want to save the robot data log? (Y/N)");
			Key key = Keyboard::wait_for_any_keys({ Key::Y, Key::N });
			if (key == Key::Y) {
				csv_write_row(filepath, header);
            	csv_append_rows(filepath, robot_log);
			}
		}
	}

	disable_realtime();
	Keyboard::clear_console_input_buffer();
    return 0;
}