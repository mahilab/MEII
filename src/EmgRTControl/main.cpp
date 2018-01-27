#include <iostream>
#include <csignal>
#include "MEL/Daq/Quanser/Q8Usb.hpp"
#include "MEL/Utility/Clock.hpp"
#include "MEL/Exoskeletons/MahiExoII/MahiExoIIEmg.hpp"
#include "EmgRTControl/EmgRTControl.hpp"
#include "MEL/Communications/Windows/MelShare.hpp"
#include "MEL/Utility/Windows/Keyboard.hpp"

using namespace mel;

static bool stop = false;
static void handler(int var) {
    stop = true;


int main(int argc, char * argv[]) {

    // make options
    Options options("ex_mahiexoii_q8usb.exe", "MahiExoII Q8 USB Demo");
    options.add_options()
        ("c,calibrate", "Calibrates the MAHI Exo-II")
        ("e,EmgRTControl", "Runs the EMG Real-Time Control Experiment for MAHI Exo-II")
        ("h,help", "Prints this help message");

    auto result = options.parse(argc, argv);

    if (result.count("help") > 0) {
        print(options.help());
        return 0;
    }


    // register ctrl-c handler
    register_ctrl_c_handler(handler);


    // make Q8 USB and configure
    Q8Usb q8;
    q8.digital_output.set_enable_values(std::vector<logic>(8, HIGH));
    q8.digital_output.set_disable_values(std::vector<logic>(8, HIGH));
    q8.digital_output.set_expire_values(std::vector<logic>(8, HIGH));


    // create and configure a MahiExoII object
    exo::MahiExoIIEmg::Config config;
    for (int i = 0; i < 5; ++i) {
        config.enable_[i] = q8_emg->do_(i + 1);
        config.command_[i] = q8_emg->ao_(i + 1);
        config.encoder_[i] = q8_emg->encoder_(i + 1);
        config.encrate_[i] = q8_emg->encrate_(i + 1);
    }
    for (int i = 0; i < 8; ++i) {
        config.emg_[i] = q8_emg->ai_(i);
    }
    exo::MahiExoIIEmg meii(config);

    // manual zero joint positions
    if (var_map.count("zero")) {
        meii.zero_encoders(q8_emg);
        return 0;
    }


    // run the experiment
    Clock clock(1000);
    enable_realtime();
    EmgRTControl emg_rt_control(clock, q8_emg, meii);
    emg_rt_control.execute();
    delete q8_emg;
    disable_realtime();
    return 0;

}