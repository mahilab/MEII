#include <iostream>
#include <csignal>
#include "MEL/Daq/Quanser/Q8Usb.hpp"
#include "MEL/Utility/Clock.hpp"
#include "MEL/Exoskeletons/MahiExoII/MahiExoIIEmg.hpp"
#include "mel_util.h"
#include "EmgRTControl/mahiexoii_util.hpp"
#include <boost/program_options.hpp>
#include "EmgRTControl/EmgTraining.hpp"
#include "MEL/Communications/Windows/MelShare.hpp"
#include "GuiFlag.h"


namespace po = boost::program_options;


int main(int argc, char * argv[]) {

    // ignore CTRL-C signal (we can do this with Input)
    signal(SIGINT, SIG_IGN);

    // set up program options 
    po::options_description desc("Available Options");
    desc.add_options()
        ("help", "produces help message")
        ("zero", "zeros encoder counts on startup");

    po::variables_map var_map;
    po::store(po::parse_command_line(argc, argv, desc), var_map);
    po::notify(var_map);

    if (var_map.count("help")) {
        print(desc);
        return 0;
    }

    // identify Q8Usb's
    uint32 id_emg = 0;
    uint32 id_ati = 1;
    if (!check_digital_loopback(0, 7)) {
        print("Warning: Digital loopback not connected to Q8Usb 0");
        if (check_digital_loopback(1, 7)) {
            id_emg = 1;
            id_ati = 0;
        }
        else {
            print("Error: Digital loopback not connected to Q8Usb 1. EMG DAQ not identified.");
            return -1;
        }
    }

    //  create a Q8Usb object
    channel_vec  ai_channels = { 0, 1, 2, 3, 4, 5, 6, 7 };
    channel_vec  ao_channels = { 0, 1, 2, 3, 4 };
    channel_vec  di_channels = { 0, 1, 2, 3, 4, 5, 6, 7 };
    channel_vec  do_channels = { 0, 1, 2, 3, 4, 5, 6, 7 };
    channel_vec enc_channels = { 0, 1, 2, 3, 4 };

    // zero the encoders if requested by user
    Q8Usb::Options options;
    for (int i = 0; i < 8; ++i) {
        options.do_initial_signals_[i] = 1;
        options.do_final_signals_[i] = 1;
        options.do_expire_signals_[i] = 1;
    }

    Daq* q8_emg = new Q8Usb(id_emg, ai_channels, ao_channels, di_channels, do_channels, enc_channels, options);


    /*//  create a second Q8Usb object
    ai_channels = { 0, 1, 2, 3, 4, 5 };
    ao_channels = {};
    di_channels = {};
    do_channels = {};
    enc_channels = {};

    Daq* q8_ati = new Q8Usb(id_ati, ai_channels, ao_channels, di_channels, do_channels, enc_channels);*/


    // create and configure a MahiExoII object
    MahiExoIIEmgFrc::Config config;
    for (int i = 0; i < 5; ++i) {
        config.enable_[i] = q8_emg->do_(i);
        config.command_[i] = q8_emg->ao_(i);
        config.encoder_[i] = q8_emg->encoder_(i);
        config.encrate_[i] = q8_emg->encrate_(i);
    }
    for (int i = 0; i < 8; ++i) {
        config.emg_[i] = q8_emg->ai_(i);
    }
    /*for (int i = 0; i < 6; ++i) {
        config.wrist_force_sensor_[i] = q8_ati->ai_(i);
    }*/
    //MahiExoIIEmgFrc meii(config);
    MahiExoIIEmg meii(config);

    // manual zero joint positions
    if (var_map.count("zero")) {
        q8_emg->enable();
        q8_emg->offset_encoders({ 0, -33259, 29125, 29125, 29125 });
        q8_emg->disable();
        return 0;
    }
    
    // create GUI flag
    GuiFlag gui_flag("gui_flag", 0);

    // run the experiment
    int input_mode = 0;
    Clock clock(1000);
    //EmgTraining emg_training(clock, q8_emg, q8_ati, meii, gui_flag, input_mode);
    EmgTraining emg_training(clock, q8_emg, meii, gui_flag, input_mode);
    emg_training.execute();
    delete q8_emg;// , q8_ati;
    return 0;

}