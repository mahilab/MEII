#include <MEL/Utility/System.hpp>
#include <MEL/Communications/MelShare.hpp>
#include <MEL/Utility/Options.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Logging/DataLogger.hpp>
#include <MEL/Utility/Console.hpp>
#include <MEL/Utility/Windows/Keyboard.hpp>
#include <MEII/Utility/logging_util.hpp>
#include <MEII/EMG/MesArray.hpp>
#include <MEL/Devices/Myo/MyoBand.hpp>
#include <MEII/EMG/EmgDataCapture.hpp>
#include <MEII/EmgRealTimeControl/EmgRealTimeControl.hpp>
#include <MEII/EMG/EmgDirectMapping.hpp>
#include <MEII/Unity/UnityMyoML.hpp>
#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEII/MahiExoII/MahiExoII.hpp>
#include <MEL/Math/Integrator.hpp>
#include <MEII/Control/DynamicMotionPrimitive.hpp>
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

	// make options
	Options options("emg_real_time_control.exe", "EMG controlling the MAHI Exo-II in real time");
	options.add_options()
		("z,zero", "Zeros the MAHI Exo-II encoders.")
		("c,calibrate", "Calibrates EMG.")
		("r,run", "Runs the direct control demonstration.")
		("h,help", "Prints this help message");

	auto result = options.parse(argc, argv);

	if (result.count("help") > 0) {
		std::cout << options.help() << "\r\n";
		return 0;
	}

	// enable Windows realtime
	enable_realtime();

	// initialize logger
	init_logger();

	// register ctrl-c handler
	register_ctrl_handler(handler);

	// set emg channel numbers
	std::vector<uint32> emg_channel_numbers = { 0, 1, 2, 3, 4, 5, 6, 7 };
	std::size_t emg_channel_count = emg_channel_numbers.size();

	// set sample rate
	Time Ts = milliseconds(1);

	// initialize data capture variables		
	Time mes_baseline_capture_period = seconds(1);
	Time mes_active_capture_period = seconds(5);

	// set data capture variables
	std::size_t mes_baseline_capture_window_size = (std::size_t)((unsigned)(mes_baseline_capture_period.as_seconds() / Ts.as_seconds()));
	std::size_t mes_active_capture_window_size = (std::size_t)((unsigned)(mes_active_capture_period.as_seconds() / Ts.as_seconds()));
	Clock baseline_capture_clock;
	Clock active_capture_clock;

	// construct Q8 USB and configure    
	Q8Usb q8;
	q8.digital_output.set_enable_values(std::vector<Logic>(8, High));
	q8.digital_output.set_disable_values(std::vector<Logic>(8, High));
	q8.digital_output.set_expire_values(std::vector<Logic>(8, High));
	if (!q8.identify(7)) {
		LOG(Error) << "Incorrect DAQ";
		mel::disable_realtime();
		return 0;
	}

	// create MahiExoII and bind Q8 channels to it
	std::vector<Amplifier> amplifiers;
	std::vector<double> amp_gains;
	for (uint32 i = 0; i < 2; ++i) {
		amplifiers.push_back(
			Amplifier("meii_amp_" + std::to_string(i),
				Low,
				q8.digital_output[i + 1],
				1.8,
				q8.analog_output[i + 1])
		);
	}
	for (uint32 i = 2; i < 5; ++i) {
		amplifiers.push_back(
			Amplifier("meii_amp_" + std::to_string(i),
				Low,
				q8.digital_output[i + 1],
				0.184,
				q8.analog_output[i + 1])
		);
	}
	MeiiConfiguration config(q8, q8.watchdog, q8.encoder[{1, 2, 3, 4, 5}], q8.velocity[{1, 2, 3, 4, 5}], amplifiers);
	MahiExoII meii(config);

	// zero - manually zero the encoders (right arm supinated)
	if (result.count("zero") > 0) {
		meii.calibrate(stop);
		LOG(Info) << "MAHI Exo-II encoders zeroed.";
		mel::disable_realtime();
		return 0;
	}

	// initialize robot velocity control variables
	DoF active_dof = WristFE;
	std::vector<std::string> dof_str = { "ElbowFE", "WristPS", "WristFE", "WristRU" };
	//std::vector<DoF> active_dofs = { ElbowFE };
	std::vector<double> velocity_control_scalars = {
		20 * mel::DEG2RAD,
		20 * mel::DEG2RAD,
		20 * mel::DEG2RAD,
		20 * mel::DEG2RAD,
		20 * mel::DEG2RAD,
		20 * mel::DEG2RAD,
		20 * mel::DEG2RAD,
		20 * mel::DEG2RAD };
	//std::map <DoF, double> velocity_control_scalars_map 
	//{ std::make_pair(ElbowFE, 10 * mel::DEG2RAD),
	//  std::make_pair(WristPS, 10 * mel::DEG2RAD),
	//  std::make_pair(WristFE, 10 * mel::DEG2RAD),
	//  std::make_pair(WristRU, 10 * mel::DEG2RAD)
	//};
	std::vector<double> ref_vel(MahiExoII::N_aj_, 0.0);
	std::vector<double> ref = { -35 * DEG2RAD, 00 * DEG2RAD, 00 * DEG2RAD, 00 * DEG2RAD, 0.09 };
	Integrator active_ref_integrator;
	Clock ref_clock;
	std::vector<double> command_torques(meii.N_aj_, 0.0);
	std::vector<double> rps_command_torques(meii.N_qs_, 0.0);

	// create robot anatomical joint space ranges
	std::vector<std::vector<double>> setpoint_rad_ranges = { { -90 * DEG2RAD, 0 * DEG2RAD },
	{ -90 * DEG2RAD, 90 * DEG2RAD },
	{ -15 * DEG2RAD, 15 * DEG2RAD },
	{ -15 * DEG2RAD, 15 * DEG2RAD },
	{ 0.08, 0.115 } };

	// change rps init position from default
	meii.set_rps_init_pos({ 0.12, 0.12, 0.12 });

	// initialize trajectory
	WayPoint neutral_point(Time::Zero, { -35 * DEG2RAD , 00 * DEG2RAD , 00 * DEG2RAD , 00 * DEG2RAD , 0.09 });	
	WayPoint final_point(Time::Zero, { -35 * DEG2RAD , 00 * DEG2RAD , 00 * DEG2RAD , 00 * DEG2RAD , 0.09 });
	std::vector<Time> dmp_durations = { seconds(4.0), seconds(4.0), seconds(3.0), seconds(3.0), seconds(4.0), seconds(3.0) };
	std::vector<double> traj_max_diff = { 60 * mel::DEG2RAD, 60 * mel::DEG2RAD, 45 * mel::DEG2RAD, 45 * mel::DEG2RAD, 0.1 };
	Time time_to_start = seconds(3.0);
	Time time_to_final = seconds(3.0);
	Time dmp_Ts = milliseconds(50);

	// set trajectory parameters based on selected dof				
	Time dmp_duration = dmp_durations[active_dof];
	DynamicMotionPrimitive dmp(dmp_Ts, neutral_point, final_point.set_time(dmp_duration));
	dmp.set_trajectory_params(Trajectory::Interp::Linear, traj_max_diff);
	if (!dmp.trajectory().validate()) {
		LOG(Warning) << "DMP trajectory invalid.";
		return 0;
	}

	// construct and enable myo
	MyoBand myo("my_myo");
	myo.enable();

	// construct Myoelectric Signal (MES) Array
	std::size_t mes_buffer_capacity = std::max(mes_baseline_capture_window_size, mes_active_capture_window_size);
	MesArray mes(myo.get_channels(emg_channel_numbers), mes_buffer_capacity);

	// construct regressor
	EmgDirectMapping mes_map(emg_channel_count, Ts);
	mes_map.set_scaling(velocity_control_scalars);
	std::vector<double> pred(emg_channel_count, 0.0);

	// construct data log for calibration
	//Table emg_cal_log("EmgCalibration");
	//for (std::size_t i = 0; i < emg_channel_count; ++i) {

	//}
	//std::string emg_cal_log_filename = dof_str[active_dof] + "_" + "active_cal";

	// make MelShares
	MelShare ms_emg("emg");
	MelShare ms_pred("pred");
	MelShare ms_ref("ref");

	// construct global timer in hybrid mode to avoid using 100% CPU
	Timer global_timer(Ts, Timer::Hybrid);

	// construct clock for regulating keypress
	Clock keypress_refract_clock;
	Time keypress_refract_time = seconds(0.5);

	// set up state machine
	enum State {
		Backdrive,        // Backdrive = 0
		InitRPS,		  // InitRPS = 1
		GoToNeutral,	  // GoToNeutral = 2
		WaitAtNeutral,    // WaitAtNeutral = 3
		DirectControl,    // DirectControl = 4
		Finish,           // Finish = 5
		Last
	};
	std::vector<std::string> state_str = { "Backdrive", "InitRPS", "GoToNeutral", "WaitAtNeutral", "DirectControl", "Finish", "Last" };
	State state = Backdrive;
	Clock state_clock;
	Time backdrive_time = seconds(1);
	Time wait_at_neutral_time = seconds(0.2);
	bool stop = false;
	bool save_data = false;
	
	// calibrate - calibrates the EMG direct mapping
	if (result.count("calibrate") > 0) {

		// Calibrate Direct Mapping
		LOG(Info) << "Recording Myo Armband EMG to Calibrate Direct Mapping.";
		std::cout << "Press 'A + 0' to add 'baseline' data to fit direct mapping." << "\r\n"
			<< "Press 'C + 0' to clear 'baseline' data previously added." << "\r\n"
			<< "Press 'A + target #' to add 'active' data to fit direct mapping." << "\r\n"
			<< "Press 'C + target #' to clear 'active' data previously added." << "\r\n"
			<< "Press 'F' to fit direct mapping and begin real-time prediction." << "\r\n"
			<< "Press 'Enter' to save data and use fitted mapping." << "\r\n"
			<< "Press 'Escape' to exit." << "\r\n";
		while (!stop) {

			// update all input channels
			myo.update();

			// update EMG signal processing
			mes.update_and_buffer();

			// predict dependent variable
			if (mes_map.update(mes.get_envelope())) {
				pred = mes_map.get_pred();
			}

			// write prediction to melshare
			ms_pred.write_data(pred);

			// clear baseline data
			if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
				if (Keyboard::are_all_keys_pressed({ Key::C, Key::Num0 })) {
					mes_map.clear_baseline_data();
					LOG(Info) << "Cleared baseline data.";
					keypress_refract_clock.restart();
				}
			}

			// capture baseline data
			if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
				if (Keyboard::are_all_keys_pressed({ Key::A, Key::Num0 })) {
					if (mes.is_buffer_full()) {
						if (baseline_capture_clock.get_elapsed_time() > mes_baseline_capture_period) {
							if (mes_map.add_baseline_data(mes.get_env_buffer_data(mes_baseline_capture_window_size))) {
								LOG(Info) << "Added baseline data";
							}
							baseline_capture_clock.restart();
						}
					}
					else {
						LOG(Warning) << "MES buffer not full. Cannot add baseline data.";
					}
					keypress_refract_clock.restart();
				}
			}

			// clear active data
			if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
				if (Keyboard::are_all_keys_pressed({ Key::C, Key::Num1 })) {
					mes_map.clear_active_data();
					LOG(Info) << "Cleared active data.";
					keypress_refract_clock.restart();
				}
			}

			// capture active data
			if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
				if (Keyboard::are_all_keys_pressed({ Key::A, Key::Num1 })) {
					if (mes.is_buffer_full()) {
						if (active_capture_clock.get_elapsed_time() > mes_active_capture_period) {
							if (mes_map.add_active_data(mes.get_env_buffer_data(mes_active_capture_window_size))) {
								LOG(Info) << "Added active data";
							}
							active_capture_clock.restart();
						}
					}
					else {
						LOG(Warning) << "MES buffer not full. Cannot add active data.";
					}
					keypress_refract_clock.restart();
				}
			}

			// fit the direct mapping to the stored data
			if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
				if (Keyboard::is_key_pressed(Key::F)) {
					if (mes_map.fit()) {
						LOG(Info) << "Fit new direct mappping based on given data.";
					}
					keypress_refract_clock.restart();
				}
			}

			// write to MelShares
			ms_emg.write_data(mes.get_envelope());

			// check for finish fitting
			if (Keyboard::is_key_pressed(Key::Enter)) {
				stop = true;
				save_data = true;
			}

			// check for exit key
			if (Keyboard::is_key_pressed(Key::Escape)) {
				stop = true;
			}

			// wait for remainder of sample period
			global_timer.wait();

		}

		// save the calibration data
		if (save_data) {
			if (mes_map.export_training_data(dof_str[active_dof] + "_" + "emg_dm_cal", ".", false)) {
				LOG(Info) << "EMG calibration data successfully saved.";
			}
			//DataLogger::write_to_csv(emg_cal_log, emg_cal_log_filename, ".", false);
		}

	}  // end calibration of direct mapping


	if (result.count("run") > 0) {

		// begin state machine
		LOG(Info) << "Robot Backdrivable";

		// enable Q8 and MEII
		q8.enable();
		meii.enable();

		// initialize controller
		meii.set_rps_control_mode(0);

		// start robot control loop
		q8.watchdog.start();
		state_clock.restart();
		while (!stop) {

			// update all input channels
			q8.update_input();
			myo.update();

			// update MahiExoII kinematics
			meii.update_kinematics();

			// update EMG signal processing
			mes.update_and_buffer();

			// predict dependent variable
			if (mes_map.update(mes.get_envelope())) {
				pred = mes_map.get_pred();
			}

			// begin switch state
			switch (state) {
			case Backdrive:

				// update ref, though not being used
				ref = meii.get_anatomical_joint_positions();

				// command zero torque
				meii.set_joint_torques(command_torques);

				// check for wait period to end
				if (state_clock.get_elapsed_time() >= backdrive_time) {
					meii.rps_init_par_ref_.start(meii.get_wrist_parallel_positions(), global_timer.get_elapsed_time());
					state = InitRPS;
					LOG(Info) << "Initializing RPS Mechanism.";
					state_clock.restart();
				}
				break;

			case InitRPS:
				// update ref, though not being used
				ref = meii.get_anatomical_joint_positions();

				// calculate commanded torques
				rps_command_torques = meii.set_rps_pos_ctrl_torques(meii.rps_init_par_ref_, global_timer.get_elapsed_time());
				std::copy(rps_command_torques.begin(), rps_command_torques.end(), command_torques.begin() + 2);

				// check for RPS Initialization target reached
				if (meii.check_rps_init()) {
					state = GoToNeutral;
					LOG(Info) << "RPS initialization complete.";
					LOG(Info) << "Going to neutral position.";
					meii.set_rps_control_mode(2); // platform height NON-backdrivable                   
					dmp.set_endpoints(WayPoint(Time::Zero, meii.get_anatomical_joint_positions()), neutral_point.set_time(time_to_start));
					if (!dmp.trajectory().validate()) {
						LOG(Warning) << "DMP trajectory invalid.";
						stop = true;
					}
					ref_clock.restart();
					state_clock.restart();
				}
				break;

			case GoToNeutral:
				// update reference from trajectory
				ref = dmp.trajectory().at_time(ref_clock.get_elapsed_time());

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

				// check for end of trajectory
				if (ref_clock.get_elapsed_time() > dmp.trajectory().back().when()) {
					state = WaitAtNeutral;
					ref = dmp.trajectory().back().get_pos();
					LOG(Info) << "Waiting at neutral position.";
					state_clock.restart();
				}

				break;

			case WaitAtNeutral:
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

				if (state_clock.get_elapsed_time() > wait_at_neutral_time) {
					command_torques.assign(MahiExoII::N_aj_, 0.0);
					state = DirectControl;
					LOG(Info) << "EMG Direct Control.";
					state_clock.restart();
					ref_clock.restart();
					active_ref_integrator.set_init(meii.get_anatomical_joint_position(active_dof));
				}

				break;

			case DirectControl:

				// calculate velocity controls
				if (pred[4] > pred[0]) {
					ref_vel[active_dof] = pred[0];
				}
				else {
					ref_vel[active_dof] = -pred[4];
				}
				//ref_vel[active_dof] = pred[4] - pred[0];

				// calculate new reference position
				ref[active_dof] = active_ref_integrator.update(ref_vel[active_dof], ref_clock.get_elapsed_time());

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

				break;

			case Finish:
				break;
			case Last:
				LOG(Error) << "Should not have entered last state.";
				stop = true;
				break;
			}

			// write to MelShares
			ms_emg.write_data(mes.get_envelope());
			ms_pred.write_data(pred);
			ms_ref.write_data(ref);


			// update all DAQ output channels
			q8.update_output();

			// check for exit key
			if (Keyboard::is_key_pressed(Key::Escape)) {
				stop = true;
			}

			// kick watchdog and check limits
			if (!q8.watchdog.kick() || meii.any_limit_exceeded()) {
				stop = true;
			}

			// wait for remainder of sample period
			global_timer.wait();

		}
		meii.disable();
		q8.disable();

	} // end direct control

	mel::disable_realtime();
	return 0;
}
