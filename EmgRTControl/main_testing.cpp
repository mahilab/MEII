#include <iostream>
#include <csignal>
#include "Q8Usb.h"
#include "Clock.h"
#include "MahiExoIIEmg.h"
#include "mel_util.h"
#include "mahiexoii_util.h"
#include <boost/program_options.hpp>
#include "BackdriveMode.h"
#include "SmoothPositionControl.h"
#include "IsometricContractions.h"
#include "EmgRTControl.h"
#include "MelShare.h"
#include "Input.h"
#include <iostream>
#include <fstream>
#include <string>
#include <random>
#include <numeric>
#include <Eigen\Dense>
#include <Eigen\StdVector>
#include "Filter.h"

using namespace mel;

int main(int argc, char * argv[]) {

    // ignore CTRL-C signal (we can do this with Input)
    signal(SIGINT, SIG_IGN);

    // set up program options 
    boost::program_options::options_description desc("Available Options");
    desc.add_options()
        ("help", "produces help message")
        ("zero", "zeros encoder counts on startup");

    boost::program_options::variables_map var_map;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), var_map);
    boost::program_options::notify(var_map);

    if (var_map.count("help")) {
        util::print(desc);
        return 0;
    }


    //  create a Q8Usb object for the board connected to robot motors
    uint32 id_mot = 0;
    channel_vec  ai_channels = { 0, 1, 2, 3, 4, 5, 6, 7 };
    channel_vec  ao_channels = { 1, 2, 3, 4, 5 };
    channel_vec  di_channels = { 0, 1, 2, 3, 4, 5, 6, 7 };
    channel_vec  do_channels = { 0, 1, 2, 3, 4, 5, 6, 7 };
    channel_vec enc_channels = { 1, 2, 3, 4, 5 };
    dev::Q8Usb::Options options;
    for (int i = 0; i < 8; ++i) {
        options.do_initial_signals_[i] = 1;
        options.do_final_signals_[i] = 1;
        options.do_expire_signals_[i] = 1;
    }
    core::Daq* q8_mot = new dev::Q8Usb(id_mot, ai_channels, ao_channels, di_channels, do_channels, enc_channels, options);


    /*// create and configure a MahiExoII object
    exo::MahiExoII::Config config;
    for (int i = 0; i < exo::MahiExoII::N_rj_; ++i) {
        config.enable_[i] = q8_mot->do_(i+1);
        config.command_[i] = q8_mot->ao_(i+1);
        config.encoder_[i] = q8_mot->encoder_(i+1);
        config.encrate_[i] = q8_mot->encrate_(i+1);
    }
    exo::MahiExoII meii(config);*/

    // create and configure a MahiExoIIEmg object
    exo::MahiExoIIEmg::Config config;
    for (int i = 0; i < 5; ++i) {
        config.enable_[i] = q8_mot->do_(i + 1);
        config.command_[i] = q8_mot->ao_(i + 1);
        config.encoder_[i] = q8_mot->encoder_(i + 1);
        config.encrate_[i] = q8_mot->encrate_(i + 1);
    }
    for (int i = 0; i < 8; ++i) {
        config.emg_[i] = q8_mot->ai_(i);
    }
    exo::MahiExoIIEmg meii(config);

    // manual zero joint positions
    if (var_map.count("zero")) {
        meii.zero_encoders(q8_mot);
        return 0;
    }


    // run state machine
    util::Clock clock(1000);
    util::enable_realtime();
    BackdriveMode bd_mode(clock, q8_mot, meii);
    bd_mode.execute();
    delete q8_mot;
    util::disable_realtime();
    return 0;

}