#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEII/MahiExoII/MahiExoII.hpp>
#include <MEL/Utility/System.hpp>
#include <MEL/Communications/MelShare.hpp>
#include <MEL/Utility/Options.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Core/Console.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Logging/Csv.hpp>
#include <MEL/Devices/Windows/Keyboard.hpp>
#include <MEII/Control/Trajectory.hpp>
#include <MEII/Control/DynamicMotionPrimitive.hpp>
#include <MEII/Control/MinimumJerk.hpp>
#include <MEL/Math/Integrator.hpp>
#include <MEII/OpenSim/osim_utility.hpp>
#include <MEII/Control/DisturbanceObserver.hpp>
#include <vector>
#include <MEL/Math/Butterworth.hpp>

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
		("s,single", "MAHI Exo-II follows a trajectory of choice")
		("i,int", "Enter an interger", value<int>())
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

	// construct Q8 USB and configure    
	Q8Usb q8;
	q8.open();
	q8.DO.set_enable_values(std::vector<Logic>(8, High));
	q8.DO.set_disable_values(std::vector<Logic>(8, High));
	q8.DO.set_expire_values(std::vector<Logic>(8, High));
	if (!q8.identify(7)) {
		LOG(Error) << "Incorrect DAQ";
		return 0;
	}
	Time Ts = milliseconds(1); // sample period for DAQ

	// create MahiExoII and bind Q8 channels to it
	std::vector<Amplifier> amplifiers;
	std::vector<double> amp_gains;
	for (uint32 i = 0; i < 2; ++i) {
		amplifiers.push_back(
			Amplifier("meii_amp_" + std::to_string(i),
				Low,
				q8.DO[i + 1],
				1.8,
				q8.AO[i + 1])
		);
	}
	for (uint32 i = 2; i < 5; ++i) {
		amplifiers.push_back(
			Amplifier("meii_amp_" + std::to_string(i),
				Low,
				q8.DO[i + 1],
				0.184,
				q8.AO[i + 1])
		);
	}
	MeiiConfiguration config(q8, q8.watchdog, q8.encoder[{1, 2, 3, 4, 5}], amplifiers);
	MahiExoII meii(config);

	bool rps_is_init = false;

	// calibrate - manually zero the encoders (right arm supinated)
	if (result.count("calibrate") > 0) {
		meii.calibrate_auto(stop);
		LOG(Info) << "MAHI Exo-II encoders calibrated.";
		return 0;
	}

	// make MelShares
	MelShare ms_pos("ms_pos");
	MelShare ms_vel("ms_vel");
	MelShare ms_trq("ms_trq");
	MelShare ms_ref("ms_ref");
	MelShare ms_dob("ms_dob");

	// create ranges
	std::vector<std::vector<double>> setpoint_rad_ranges = { { -90 * DEG2RAD, 0 * DEG2RAD },
	{ -90 * DEG2RAD, 90 * DEG2RAD },
	{ -15 * DEG2RAD, 15 * DEG2RAD },
	{ -15 * DEG2RAD, 15 * DEG2RAD },
	{ 0.08, 0.115 } };

	// construct timer in hybrid mode to avoid using 100% CPU
	Timer timer(Ts, Timer::Hybrid);
	timer.set_acceptable_miss_rate(0.05);
	mel::Time t;

	// construct clock for regulating keypress
	Clock keypress_refract_clock;
	Time keypress_refract_time = seconds(0.5);

    enum Phase {
        Backdrive,   // Backdrive   = 0
		InitRPS,     // InitRPS     = 1
		MoveToStart, // MoveToStart = 2
        MoveFlex,    // MoveFlex    = 3
		MoveWait,    // MoveWait    = 4
		MoveExtend,  // MoveExtend  = 5
        Stop         // Stop        = 6
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

	bool save_data = true;
    std::string filepath = "example_DO_robot_data_log.csv";

	// construct robot data log
	std::vector<double> robot_log_row(6);
	std::vector<std::vector<double>> robot_log;
	std::vector<std::string> header = { "Time [s]", "ref 1 [rad]", "pos 1 [rad]", "vel 1 [rad/s]", "Torque 1 [Nm]", "DO Torque [Nm]" };

	DisturbanceObserver DO(Ts);
	Butterworth butt(2,hertz(10),hertz(1000));
	double delta_t;
	double t_last = 0.0;
	double d_hat = 0.0;
	double d_hat_smooth = 0.0;

	// trajectory following
	if (result.count("single") > 0) {
		LOG(Info) << "MAHI Exo-II Trajectory Following.";

		// setup trajectories
		std::size_t num_full_cycles = 2;
		std::size_t current_cycle = 0;

		std::vector<WayPoint> extreme_points = { WayPoint(Time::Zero,{ -05 * DEG2RAD, 00 * DEG2RAD, 00 * DEG2RAD, 00 * DEG2RAD, 0.09 }),  // flexed
			  									  WayPoint(Time::Zero,{ -65 * DEG2RAD, 00 * DEG2RAD, 00 * DEG2RAD, 00 * DEG2RAD, 0.09 })}; // extended

        Time mj_duration = seconds(5.0);

		std::vector<double> traj_max_diff = { 50 * mel::DEG2RAD, 50 * mel::DEG2RAD, 25 * mel::DEG2RAD, 25 * mel::DEG2RAD, 0.1 };
		Time time_to_start = seconds(3.0);
		Time mj_Ts = milliseconds(50);

		// Initializing variables for linear travel
		WayPoint initial_waypoint;
		std::vector<WayPoint> waypoints(2);
		Trajectory ref_traj;
		WayPoint current_wp;
		WayPoint next_wp;

		//default mj traj
		MinimumJerk mj(mj_Ts, extreme_points[0], extreme_points[1].set_time(mj_duration));
		mj.set_trajectory_params(Trajectory::Interp::Linear, traj_max_diff);

		// Initializing variables for dmp
		DoF dof = ElbowFE; // default
		bool traj_selected = false;

		TrajType traj_type = traj_mj;
		std::size_t current_extreme_idx = 0;

		// construct clocks for waiting and trajectory
		Clock state_clock;
		Clock ref_traj_clock;

		// set up state machine
		Phase state = Backdrive;
		Time backdrive_time = seconds(1);
		Time wait_at_extreme_time = seconds(1);

		// create data containers
		std::vector<double> rj_positions(meii.N_rj_);
		std::vector<double> rj_velocities(meii.N_rj_);
		std::vector<double> aj_positions(meii.N_aj_);
		std::vector<double> aj_velocities(meii.N_aj_);
		std::vector<double> command_torques(meii.N_aj_, 0.0);
		std::vector<double> rps_command_torques(meii.N_qs_, 0.0);
		std::vector<double> ref(meii.N_aj_, 0.0);

		// enable DAQ and exo
		q8.enable();
		meii.enable();

		// prompt user for input
		print("Press 'Escape' to exit the program.");
		print("Press 'Enter' to exit the program and save data.");

		print("Press 'S' to start the trajectory.");
		
		// start loop
		LOG(Info) << "Robot Backdrivable.";
		q8.watchdog.start();
		state_clock.restart();
		while (!stop) {

			// update all DAQ input channels
			q8.update_input();

			// update MahiExoII kinematics
			meii.update_kinematics();

			// store most recent readings from DAQ
			for (int i = 0; i < meii.N_rj_; ++i) {
				rj_positions[i] = meii[i].get_position();
				rj_velocities[i] = meii[i].get_velocity();
			}
			for (int i = 0; i < meii.N_aj_; ++i) {
				aj_positions[i] = meii.get_anatomical_joint_position(i);
				aj_velocities[i] = meii.get_anatomical_joint_velocity(i);
			}

			// begin switch state
			switch (state) {
			case Backdrive: // backdrive

				// update ref, though not being used
				ref = meii.get_anatomical_joint_positions();

				for (size_t i = 0; i < meii.N_aj_; i++){
					command_torques[i] = 0.0;
				}

				// command zero torque
				meii.set_joint_torques(command_torques);

				// prompt user for input to select which trajectory
				if (!traj_selected) {

					// press D for dmp trajectory
					if (Keyboard::is_key_pressed(Key::S)) {
						traj_selected = true;
					}

					// check for exit key
					if (Keyboard::is_key_pressed(Key::Escape)) {
						stop = true;
						save_data = false;
					}
				}

				// check for wait period to end
				if (traj_selected) {
					
					if (!rps_is_init) {
						meii.rps_init_par_ref_.start(meii.get_wrist_parallel_positions(), timer.get_elapsed_time());
						LOG(Info) << "Initializing RPS Mechanism";
					}

					traj_selected = false;

					ref_traj_clock.restart();
					state = InitRPS;
					
					state_clock.restart();
				}
				break;

			case InitRPS: // initialize rps                

				if (!rps_is_init) {
					// update ref, though not being used
					ref = meii.get_anatomical_joint_positions();

					// calculate commanded torques
					rps_command_torques = meii.set_rps_pos_ctrl_torques(meii.rps_init_par_ref_, timer.get_elapsed_time());
					std::copy(rps_command_torques.begin(), rps_command_torques.end(), command_torques.begin() + 2);
				}
				else {
					// set zero torque
					for (size_t i = 0; i < meii.N_aj_; i++) {
						command_torques[i] = 0.0;
					}
					// command zero torque
					meii.set_joint_torques(command_torques);
				}

				// check for RPS Initialization target reached
				if (meii.check_rps_init() || rps_is_init == true) {
					
					/*meii.rps_init_par_ref_.stop();*/
					state = MoveExtend;
					if (!rps_is_init) {
						LOG(Info) << "RPS initialization complete.";
						meii.set_rps_control_mode(2); // platform height NON-backdrivable   
					}
					
					rps_is_init = true;

					LOG(Info) << "Going to Extended Position.";
					
					// define new waypoints
					waypoints[0] = WayPoint(Time::Zero, meii.get_anatomical_joint_positions());
					waypoints[1] = extreme_points[1].set_time(mj_duration);
					mj.set_endpoints(waypoints[0], waypoints[1]);
					if (!mj.trajectory().validate()) {
						LOG(Warning) << "MJ trajectory invalid.";
						stop = true;
					}
					ref_traj = mj.trajectory();

					ref_traj_clock.restart();
					state_clock.restart();
				}
				break;

			case MoveFlex: // wait at neutral position
				// update reference from trajectory
				ref = ref_traj.at_time(ref_traj_clock.get_elapsed_time());

				// constrain trajectory to be within range
				for (std::size_t i = 0; i < meii.N_aj_; ++i) {
					ref[i] = saturate(ref[i], setpoint_rad_ranges[i][0], setpoint_rad_ranges[i][1]);
				}

				// calculate anatomical command torques
				command_torques[0] = meii.anatomical_joint_pd_controllers_[0].calculate(ref[0], meii[0].get_position(), 0, meii[0].get_velocity());
				command_torques[1] = meii.anatomical_joint_pd_controllers_[1].calculate(ref[1], meii[1].get_position(), 0, meii[1].get_velocity());
				for (std::size_t i = 0; i < meii.N_qs_; ++i) {
					rps_command_torques[i] = meii.anatomical_joint_pd_controllers_[i + 2].calculate(ref[i + 2], meii.get_anatomical_joint_position(i + 2), 0, meii.get_anatomical_joint_velocity(i + 2));
				}
				std::copy(rps_command_torques.begin(), rps_command_torques.end(), command_torques.begin() + 2);

				// command_torques[0] = command_torques[0] - d_hat_smooth;

				// set anatomical command torques
				meii.set_anatomical_joint_torques(command_torques);

				// check for wait period to end
				if (state_clock.get_elapsed_time() > mj_duration) {
					state = MoveWait;
					LOG(Info) << "Waiting at Flex Position.";

					state_clock.restart();
					ref_traj_clock.restart();
				}

				break;
			
			case MoveExtend: // go to extreme position

				// update reference from trajectory
				ref = ref_traj.at_time(ref_traj_clock.get_elapsed_time());

				// constrain trajectory to be within range
				for (std::size_t i = 0; i < meii.N_aj_; ++i) {
					ref[i] = saturate(ref[i], setpoint_rad_ranges[i][0], setpoint_rad_ranges[i][1]);
				}

				// calculate anatomical command torques
				command_torques[0] = meii.anatomical_joint_pd_controllers_[0].calculate(ref[0], meii[0].get_position(), 0, meii[0].get_velocity());
				command_torques[1] = meii.anatomical_joint_pd_controllers_[1].calculate(ref[1], meii[1].get_position(), 0, meii[1].get_velocity());
				for (std::size_t i = 0; i < meii.N_qs_; ++i) {
					rps_command_torques[i] = meii.anatomical_joint_pd_controllers_[i + 2].calculate(ref[i + 2], meii.get_anatomical_joint_position(i + 2), 0, meii.get_anatomical_joint_velocity(i + 2));
				}
				std::copy(rps_command_torques.begin(), rps_command_torques.end(), command_torques.begin() + 2);

				// set anatomical command torques
				meii.set_anatomical_joint_torques(command_torques);

				// command_torques[0] = command_torques[0] - d_hat_smooth;

				// check for wait period to end
				if (state_clock.get_elapsed_time() > mj_duration) {
					if (current_cycle < num_full_cycles) {
						current_cycle++;
						state = MoveFlex;
						LOG(Info) << "Moving to Flexed Position";

						// generate new trajectory
						waypoints[0] = WayPoint(Time::Zero, ref);
						waypoints[1] = extreme_points[0].set_time(mj_duration);
						mj.set_endpoints(waypoints[0], waypoints[1]);
						if (!mj.trajectory().validate()) {
							LOG(Warning) << "MJ trajectory invalid.";
							stop = true;
						}
						ref_traj = mj.trajectory();
					}
					else {
						state = Backdrive;
						current_cycle = 0;
						LOG(Info) << "Trajectory finished.";
					
						// prompt user for input
						print("Press 'Escape' to exit the program.");
						print("Press 'Enter' to exit the program and save data.");

						print("Press 'S' to start the trajectory.");

						// start loop
						LOG(Info) << "Robot Backdrivable.";
					}
					state_clock.restart();
					ref_traj_clock.restart();
				}

				break;

			case MoveWait: // wait at extreme position

				// constrain trajectory to be within range
				ref = extreme_points[0].get_pos();

				// calculate anatomical command torques
				command_torques[0] = meii.anatomical_joint_pd_controllers_[0].calculate(ref[0], meii[0].get_position(), 0, meii[0].get_velocity());
				command_torques[1] = meii.anatomical_joint_pd_controllers_[1].calculate(ref[1], meii[1].get_position(), 0, meii[1].get_velocity());
				for (std::size_t i = 0; i < meii.N_qs_; ++i) {
					rps_command_torques[i] = meii.anatomical_joint_pd_controllers_[i + 2].calculate(ref[i + 2], meii.get_anatomical_joint_position(i + 2), 0, meii.get_anatomical_joint_velocity(i + 2));
				}
				std::copy(rps_command_torques.begin(), rps_command_torques.end(), command_torques.begin() + 2);

				// set anatomical command torques
				meii.set_anatomical_joint_torques(command_torques);

				// check for wait period to end
				if (state_clock.get_elapsed_time() > wait_at_extreme_time) {
					state = MoveExtend;
					LOG(Info) << "Going to Extended Position.";

					// generate new trajectory
					waypoints[0] = WayPoint(Time::Zero, ref);
					waypoints[1] = extreme_points[1].set_time(mj_duration);
					mj.set_endpoints(waypoints[0], waypoints[1]);
					if (!mj.trajectory().validate()) {
						LOG(Warning) << "MJ trajectory invalid.";
						stop = true;
					}
					ref_traj = mj.trajectory();
				
					ref_traj_clock.restart();
					state_clock.restart();
				}

				break;
			}

			DO.update(aj_positions[0], aj_velocities[0], command_torques[0], delta_t, t);
			d_hat = DO.get_d_hat();
			d_hat_smooth = butt.update(DO.get_d_hat(),t);

			// write ref to MelShares
			ms_pos.write_data(aj_positions);
			ms_vel.write_data(aj_velocities);
			ms_trq.write_data(command_torques);
			ms_ref.write_data(ref);
			ms_dob.write_data({d_hat, d_hat_smooth});

			
			// update all DAQ output channels
			q8.update_output();

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
			robot_log_row[1] = ref[0];
			robot_log_row[2] = aj_positions[0];
			robot_log_row[3] = aj_velocities[0];
			robot_log_row[4] = command_torques[0];
			robot_log_row[5] = d_hat_smooth;
			robot_log.push_back(robot_log_row);

			// kick watchdog
			if (!q8.watchdog.kick() || meii.any_limit_exceeded()) {
				stop = true;
			}
			t_last = t.as_seconds();
			// wait for remainder of sample period
			t = timer.wait();
			delta_t = t.as_seconds()-t_last;
			
		}
		meii.disable();
        q8.disable();
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
	disable_realtime();
	Keyboard::clear_console_input_buffer();
	return 0;
}