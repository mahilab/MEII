#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEII/MahiExoII/MahiExoII.hpp>
#include <MEL/Utility/System.hpp>
#include <MEL/Communications/Windows/MelShare.hpp>
#include <MEL/Utility/Options.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Logging/DataLogger.hpp>
#include <MEL/Utility/Console.hpp>
#include <MEL/Utility/Windows/Keyboard.hpp>
#include <vector>
#include <MEII/Control/Trajectory.hpp>
#include <MEII/PhriLearning/PhriFeatures.hpp>

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
    Options options("phri_learning.exe", "Physical Human-Robot Interaction Learning Experiment");
    options.add_options()
        ("c,calibrate",  "Calibrates the MAHI Exo-II")
        ("t,trajectory", "Runs the MAHI Exo-II trajectory following demo")
        ("h,help", "Prints this help message");

    auto result = options.parse(argc, argv);

    if (result.count("help") > 0) {
        print(options.help());
        return 0;
    }

    // enable Windows realtime
    enable_realtime();

    // initialize logger
    init_logger();

    // register ctrl-c handler
    register_ctrl_handler(handler);   

    // make Q8 USB and configure
    Q8Usb q8;
    q8.digital_output.set_enable_values(std::vector<Logic>(8, High));
    q8.digital_output.set_disable_values(std::vector<Logic>(8, High));
    q8.digital_output.set_expire_values(std::vector<Logic>(8, High));
    if (!q8.identify(7)) {
        LOG(Error) << "Incorrect DAQ";
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

    // calibrate - manually zero the encoders (right arm supinated)
    if (result.count("calibrate") > 0) {       
        meii.calibrate(stop);
        LOG(Info) << "MAHI Exo-II encoders calibrated.";
        return 0;
    }

    // create robot data log
    DataLogger robot_log(WriterType::Buffered, false);
    std::vector<double> robot_log_row(16);
    std::vector<std::string> log_header = { "Time [s]", "MEII EFE Position [rad]", "MEII EFE Velocity [rad/s]", "MEII EFE Commanded Torque [Nm]",
        "MEII FPS Position [rad]", "MEII FPS Velocity [rad/s]", "MEII FPS Commanded Torque [Nm]",
        "MEII RPS L1 Position [m]", "MEII RPS L1 Velocity [m/s]", "MEII RPS L1 Commanded Force [N]",
        "MEII RPS L2 Position [m]", "MEII RPS L2 Velocity [m/s]", "MEII RPS L2 Commanded Force [N]",
        "MEII RPS L3 Position [m]", "MEII RPS L3 Velocity [m/s]", "MEII RPS L3 Commanded Force [N]" };
    robot_log.set_header(log_header);
    robot_log.set_record_format(DataFormat::Default, 12);
    bool save_data = false;

    // make MelShares
    MelShare ms_pos("ms_pos");
    MelShare ms_vel("ms_vel");
    MelShare ms_trq("ms_trq");
    MelShare ms_ref("ms_ref");
    MelShare ms_phi("ms_phi");

    // trajectory following
    if (result.count("trajectory") > 0) {
        LOG(Info) << "MAHI Exo-II Trajectory Following.";

        // create ranges
        std::vector<std::vector<double>> setpoint_rad_ranges = { { -90 * DEG2RAD, 0 * DEG2RAD },
        { -90 * DEG2RAD, 90 * DEG2RAD },
        { -15 * DEG2RAD, 15 * DEG2RAD },
        { -15 * DEG2RAD, 15 * DEG2RAD },
        { 0.08, 0.115 } };

        // create discrete trajectory at with certain max velocities
        std::vector<double> traj_max_diff = { 10 * mel::DEG2RAD, 10 * mel::DEG2RAD, 5 * mel::DEG2RAD, 5 * mel::DEG2RAD, 0.01 };
        WayPoint initial_waypoint;
        WayPoint final_waypoint(seconds(5), { -35 * DEG2RAD, 20 * DEG2RAD, 15 * DEG2RAD, 0, 0.09 });
        std::vector<WayPoint> waypoints(2);
        waypoints[1] = final_waypoint;
        Trajectory ref_traj;

        // construct for trajectory
        Clock ref_traj_clock;

        // set up state machine
        std::size_t state = 0;
        Time backdrive_time = seconds(3);

        // create data containers
        std::vector<double> rj_positions(meii.N_rj_);
        std::vector<double> rj_velocities(meii.N_rj_);
        std::vector<double> aj_positions(meii.N_aj_);
        std::vector<double> aj_velocities(meii.N_aj_);
        std::vector<double> command_torques(meii.N_aj_, 0.0);
        std::vector<double> rps_command_torques(meii.N_qs_, 0.0);
        std::vector<double> ref(meii.N_aj_, 0.0);
        std::vector<double> phi(1, 0.0);
        std::vector<double> theta(1, 0.0);

        // enable DAQ and exo
        q8.enable();
        meii.enable();

        // initialize controller
        meii.set_rps_control_mode(0);

        // construct timer in hybrid mode to avoid using 100% CPU
        Timer timer(milliseconds(1), Timer::Hybrid);

        // start loop
        LOG(Info) << "Robot Backdrivable.";
        q8.watchdog.start();
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

            switch (state) {
            case 0: // backdrive

                // update ref, though not being used
                ref = meii.get_anatomical_joint_positions();

                // command zero torque
                meii.set_joint_torques(command_torques);

                // check for wait period to end
                if (timer.get_elapsed_time() >= backdrive_time) {
                    meii.rps_init_par_ref_.start(meii.get_wrist_parallel_positions(), timer.get_elapsed_time());
                    state = 1;
                    LOG(Info) << "Initializing RPS Mechanism.";
                }
                break;

            case 1: // initialize rps                

                // update ref, though not being used
                ref = meii.get_anatomical_joint_positions();

                // calculate commanded torques
                rps_command_torques = meii.set_rps_pos_ctrl_torques(meii.rps_init_par_ref_, timer.get_elapsed_time());
                std::copy(rps_command_torques.begin(), rps_command_torques.end(), command_torques.begin() + 2);

                // check for RPS Initialization target reached
                if (meii.check_rps_init()) {
                    LOG(Info) << "RPS initialization complete.";
                    meii.set_rps_control_mode(2); // platform height backdrivable
                    state = 2;
                    initial_waypoint = WayPoint(seconds(0.0), aj_positions);
                    waypoints[0] = initial_waypoint;
                    ref_traj.set_waypoints(meii.N_aj_, waypoints, Trajectory::Interp::Linear, traj_max_diff);
                    ref_traj_clock.restart();
                }
                break;

            case 2: // trajectory following


                // learned desired trajectory


                // update reference from trajectory
                ref = ref_traj.at_time(ref_traj_clock.get_elapsed_time());

                // constraint trajectory to be within range
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
            }

            // write to MelShares
            ms_pos.write_data(aj_positions);
            ms_vel.write_data(aj_velocities);
            ms_trq.write_data(command_torques);
            ms_ref.write_data(ref);
            ms_phi.write_data(phi);

            // update all DAQ output channels
            q8.update_output();

            // write to robot data log
            robot_log_row[0] = timer.get_elapsed_time().as_seconds();
            for (std::size_t i = 0; i < meii.N_rj_; ++i) {
                robot_log_row[3 * i + 1] = meii[i].get_position();
                robot_log_row[3 * i + 2] = meii[i].get_velocity();
                robot_log_row[3 * i + 3] = meii[i].get_torque();
            }
            robot_log.buffer(robot_log_row);

            // check for save key
            if (Keyboard::is_key_pressed(Key::Enter)) {
                stop = true;
                save_data = true;
            }

            // check for exit key
            if (Keyboard::is_key_pressed(Key::Escape)) {
                stop = true;
            }

            // kick watchdog
            if (!q8.watchdog.kick() || meii.any_limit_exceeded())
                stop = true;

            // wait for remainder of sample period
            timer.wait();

        }
        meii.disable();
        q8.disable();
    }

    

    if (save_data) {
        print("Do you want to save the robot data log? (Y/N)");
        Key key = Keyboard::wait_for_any_keys({ Key::Y, Key::N });
        if (key == Key::Y) {
            robot_log.save_data("example_meii_robot_data_log.csv", ".", false);
            robot_log.wait_for_save();
        }
    }

    disable_realtime();
    return 0;
}


