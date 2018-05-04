#include "MEII/EmgRealTimeControl/EmgRealTimeControl.hpp"
#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEII/MahiExoII/MahiExoII.hpp>
#include <MEL/Communications/Windows/MelShare.hpp>
#include <MEL/Utility/Windows/Keyboard.hpp>
#include <MEL/Utility/Options.hpp>
#include <MEL/Utility/System.hpp>
#include <MEL/Logging/Log.hpp>
#include "MEL/Utility/Console.hpp"
#include "MEII/EMG/MesArray.hpp"
#include <iostream>



using namespace mel;
using namespace meii;

// create global stop variable CTRL-C handler function
ctrl_bool stop(false);
bool handler(CtrlEvent event) {
    stop = true;
    return true;
}

int main(int argc, char * argv[]) {

    // make options
    Options options("emg_real_time_control.exe", "EMG Real-Time Control of MAHI Exo-II");
    options.add_options()
        ("c,calibrate", "Calibrates the MAHI Exo-II")
        ("e,emg","EMG signal check")
        ("n,neutral","Find the neutral wrist position")
        ("r,run", "Runs the EMG Real-Time Control Experiment for MAHI Exo-II")
        ("h,help", "Prints this help message");

    auto result = options.parse(argc, argv);

    if (result.count("help") > 0) {
        print(options.help());
        return 0;
    }

    //==========================================================================
    // SET THE EMG CHANNEL NUMBERS HERE
    //==========================================================================
    std::vector<uint32> emg_channel_numbers = { 0 };
    //==========================================================================


    // enable Windows realtime
    enable_realtime();

    // initialize default MEL logger
    init_logger();

    // register ctrl-c handler
    register_ctrl_handler(handler);    

    // make Q8 USB and configure    
    Q8Usb q8(QOptions(), true, true, emg_channel_numbers); // specify all EMG channels
    q8.digital_output.set_enable_values(std::vector<Logic>(8, High));
    q8.digital_output.set_disable_values(std::vector<Logic>(8, High));
    q8.digital_output.set_expire_values(std::vector<Logic>(8, High));
    if (!q8.identify(7)) {
        LOG(Error) << "Incorrect DAQ";
        return 0;
    }
    emg_channel_numbers = q8.analog_input.get_channel_numbers();
    std::size_t emg_channel_count = q8.analog_input.get_channel_count();

    // construct array of Myoelectric Signals    
    MesArray mes(q8.analog_input.get_channels(emg_channel_numbers));

    // create MahiExoII and bind Q8 channels to it
    std::vector<Amplifier> amplifiers;
    std::vector<double> amp_gains;
    for (uint32 i = 0; i < 2; ++i) {
        amplifiers.push_back(
            Amplifier("meii_amp_" + std::to_string(i),
                Logic::Low,
                q8.digital_output[i + 1],
                1.8,
                q8.analog_output[i + 1])
        );
    }
    for (uint32 i = 2; i < 5; ++i) {
        amplifiers.push_back(
            Amplifier("meii_amp_" + std::to_string(i),
                Logic::Low,
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
    }

    // check EMG signals
    if (result.count("emg") > 0) {
        LOG(Info) << "Showing EMG on MelScope";
        

        // make MelShares
        MelShare ms_mes_raw("ms_mes_raw");
        MelShare ms_mes_demean("ms_mes_demean");
        MelShare ms_mes_env("ms_mes_env");
        MelShare ms_mes_tkeo_env("ms_mes_tkeo_env");

        // create local variables
        std::vector<double> mes_raw(emg_channel_count);
        std::vector<double> mes_demean(emg_channel_count);
        std::vector<double> mes_env(emg_channel_count);
        std::vector<double> mes_tkeo_env(emg_channel_count);
        bool mes_buffers_full = false;

        // construct timer in hybrid mode to avoid using 100% CPU
        Timer timer(milliseconds(1), Timer::Hybrid);

        // prompt user for input
        print("Press 'Escape' to exit.");

        while (!stop) {

            // update all DAQ input channels
            q8.update_input();

            // emg signal processing
            mes.update();

            // write to MelShares
            ms_mes_raw.write_data(mes.get_raw());
            ms_mes_demean.write_data(mes.get_demean());
            ms_mes_env.write_data(mes.get_envelope());
            ms_mes_tkeo_env.write_data(mes.get_tkeo_envelope());

            // check for user input
            if (Keyboard::is_key_pressed(Key::Escape)) {
                stop = true;
            }

            // wait for remainder of sample period
            timer.wait();
        }

    }

    // Find neutral wrist position
    if (result.count("neutral") > 0) {

        // initialize local state variables
        bool neutral_position_selected = false;
        Key key;

        // construct timer in hybrid mode to avoid using 100% CPU
        Timer timer(milliseconds(1), Timer::Hybrid);

        // prompt user for input
        print("Press 'Enter' to capture a new wrist neutral position. Press 'Escape' to exit.");

        // enter the control loop
        while (!stop) {

            // update all DAQ input channels
            q8.update_input();

            // update robot kinematics
            meii.update_kinematics();

            // check joint limits
            if (meii.any_limit_exceeded()) {
                stop = true;
                break;
            }

            // check for user input
            if (Keyboard::is_key_pressed(Key::Escape)) {
                stop = true;
            }
            if (!neutral_position_selected) {
                if (Keyboard::is_key_pressed(Key::Enter)) {
                    neutral_position_selected = true;
                    print("Do you want to use this wrist position (Y/N)?");
                }
            }
            else {
                key = Keyboard::are_any_keys_pressed({ Key::Y, Key::N });
                switch (key) {
                case Key::Y:
                    print("Elbow F/E angle = " + std::to_string(meii.get_anatomical_joint_position(0) * RAD2DEG) + " DEG");
                    print("Forearm P/S angle = " + std::to_string(meii.get_anatomical_joint_position(1) * RAD2DEG) + " DEG");
                    print("Wrist F/E angle = " + std::to_string(meii.get_anatomical_joint_position(2) * RAD2DEG) + " DEG");
                    print("Wrist R/U angle = " + std::to_string(meii.get_anatomical_joint_position(3) * RAD2DEG) + " DEG");
                    print("Wrist platform height = " + std::to_string(meii.get_anatomical_joint_position(4)) + " M");
                    stop = true;
                    break;
                case Key::N:
                    neutral_position_selected = false;
                    print("Press 'Enter' to capture a new neutral wrist position. Press 'Escape' to exit.");
                    break;
                }
            }

            // wait for remainder of sample period
            timer.wait();
        }
        
    }

    // check if running in virtual mode
    bool virtual_hardware = false;
    if (result.count("virtual") > 0) {
        virtual_hardware = true;
    }

    // run the experiment
    if (result.count("run") > 0) {

        // enable DAQ
        //q8.enable();

        //EmgRealTimeControl emg_real_time_control(meii, q8, q8.watchdog, stop, virtual_hardware);
        //emg_real_time_control.execute();
        
    }

    disable_realtime();
    return 0;

}