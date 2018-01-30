#include <iostream>
#include <csignal>
#include "MEL/Daq/Quanser/Q8Usb.hpp"
#include "MEL/Utility/Timer.hpp"
#include "MEL/Exoskeletons/MahiExoII/MahiExoIIEmg.hpp"
#include "EmgRTControl/EmgRTControl.hpp"
#include "MEL/Communications/Windows/MelShare.hpp"
#include "MEL/Utility/Windows/Keyboard.hpp"
#include <MEL/Utility/Options.hpp>

using namespace mel;

static bool stop = false;
static void handler(int var) {
    stop = true;
}

int main(int argc, char * argv[]) {

    // make options
    Options options("ex_mahiexoii_q8usb.exe", "MahiExoII Q8 USB Demo");
    options.add_options()
        ("c,calibrate", "Calibrates the MAHI Exo-II")
        ("r,run", "Runs the EMG Real-Time Control Experiment for MAHI Exo-II")
        ("h,help", "Prints this help message");

    auto result = options.parse(argc, argv);

    if (result.count("help") > 0) {
        print(options.help());
        return 0;
    }


    // register ctrl-c handler
    register_ctrl_c_handler(handler);

    // enable Windows realtime
    enable_realtime();

    // make Q8 USB and configure
    Q8Usb q8;
    q8.digital_output.set_enable_values(std::vector<logic>(8, HIGH));
    q8.digital_output.set_disable_values(std::vector<logic>(8, HIGH));
    q8.digital_output.set_expire_values(std::vector<logic>(8, HIGH));
    if (!q8.identify(7)) {
        print("Incorrect DAQ");
        return 0;
    }


    // create MahiExoII and bind Q8 channels to it
    std::vector<Amplifier> amplifiers;
    std::vector<double> amp_gains;
    for (uint32 i = 0; i < 2; ++i) {
        amplifiers.push_back(
            Amplifier("meii_amp_" + std::to_string(i),
                Amplifier::TtlLevel::Low,
                q8.digital_output[i + 1],
                1.8,
                q8.analog_output[i + 1])
        );
    }
    for (uint32 i = 2; i < 5; ++i) {
        amplifiers.push_back(
            Amplifier("meii_amp_" + std::to_string(i),
                Amplifier::TtlLevel::Low,
                q8.digital_output[i + 1],
                0.184,
                q8.analog_output[i + 1])
        );
    }
    MeiiConfiguration config(q8, q8.watchdog, q8.encoder[{1, 2, 3, 4, 5}], q8.velocity[{1, 2, 3, 4, 5}], amplifiers, q8.analog_input[{0, 1, 2, 3, 4, 5, 6, 7}]);
    MahiExoIIEmg meii(config);

    // calibrate - manually zero the encoders (right arm supinated)
    if (result.count("calibrate") > 0) {
        meii.calibrate(stop);
        return 0;
    }


    // run the experiment
    if (result.count("run") > 0) {



        // enable DAQ
        q8.enable();

        EmgRTControl emg_rt_control(meii, q8, q8.watchdog);
        emg_rt_control.execute();
        
    }

    disable_realtime();
    return 0;

}