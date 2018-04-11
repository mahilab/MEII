#include <iostream>
#include <csignal>
#include "MEL/Daq/Quanser/Q8Usb.hpp"
#include "MEL/Utility/Clock.hpp"
#include "MEL/Exoskeletons/MahiExoII/MahiExoII.hpp"
#include "util.h"
#include "mahiexoii_util.h"
#include <boost/program_options.hpp>
#include "EmgRTControl/FesExperiment.hpp"
#include "MEL/Utility/Windows/Keyboard.hpp"

int main(int argc, char * argv[]) {

    // ignore CTRL-C signal (we can do this with Input)
    signal(SIGINT, SIG_IGN);


    // set up program options 
    boost::program_options::options_description desc("Available Options");
    desc.add_options()
        ("help", "produces help message")
        ("zero", "zeros encoder counts on startup")
        ("subject", boost::program_options::value<int>(), "the subject number, 1-10")
        ("trial", boost::program_options::value<int>(), "the trial to start at, e.g. 1, 2, 3, etc");
            

    boost::program_options::variables_map var_map;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), var_map);
    boost::program_options::notify(var_map);

    if (var_map.count("help")) {
        print(desc);
        return 0;
    }

    //  create a Q8Usb object
    uint32 id_emg = 0;
    channel_vec  ai_channels = { 0, 1, 2, 3, 4, 5, 6, 7 };
    channel_vec  ao_channels = { 1, 2, 3, 4, 5 };
    channel_vec  di_channels = { 0, 1, 2, 3, 4, 5, 6, 7 };
    channel_vec  do_channels = { 0, 1, 2, 3, 4, 5, 6, 7 };
    channel_vec enc_channels = { 1, 2, 3, 4, 5 };
    Q8Usb::Options options;
    for (int i = 0; i < 8; ++i) {
        options.do_initial_signals_[i] = 1;
        options.do_final_signals_[i] = 1;
        options.do_expire_signals_[i] = 1;
    }
    Daq* q8_emg = new Q8Usb(id_emg, ai_channels, ao_channels, di_channels, do_channels, enc_channels, options);


    // create and configure a MahiExoII object
    MahiExoII::Config config;
    for (int i = 0; i < 5; ++i) {
        config.enable_[i] = q8_emg->do_(i + 1);
        config.command_[i] = q8_emg->ao_(i + 1);
        config.encoder_[i] = q8_emg->encoder_(i + 1);
        config.encrate_[i] = q8_emg->encrate_(i + 1);
    }
    MahiExoII meii(config);

    // manual zero joint positions
    if (var_map.count("zero")) {
        q8_emg->enable();
        q8_emg->offset_encoders({ 0, -33259, 29125, 29125, 29125 });
        q8_emg->disable();
        return 0;
    }

    // read in command line input arguments
    int subject, condition, trial;
    if (var_map.count("subject") && var_map.count("trial")) {
        subject = var_map["subject"].as<int>();
        trial = var_map["trial"].as<int>();
    }
    else {
        print("Not enough input parameters were provided to run the experiment.");
        delete q8_emg;
        return -1;
    }

    // run the experiment
    Clock clock(1000);
    FesExperiment fes_experiment( clock, q8_emg, meii, subject, trial);
    fes_experiment.execute();
    delete q8_emg;
    return 0;

}