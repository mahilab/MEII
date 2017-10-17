#include "testing.h"
#include <iostream>
#include <csignal>
#include "Q8Usb.h"
#include "Clock.h"
#include "MahiExoIIEmg.h"
#include "mel_util.h"
#include "mahiexoii_util.h"
#include <boost/program_options.hpp>
#include "TransparentMode.h"
#include "SmoothPositionControl.h"
#include "IsometricContractions.h"
#include "EmgRTControl.h"
#include "MelShare.h"
#include "GuiFlag.h"
#include "Input.h"
#include <iostream>
#include <fstream>
#include <string>
#include <random>
#include <numeric>
#include <Eigen\Dense>
#include <Eigen\StdVector>

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

   
    Eigen::VectorXd y(4);
    y <<  -2.08e16, -2.04e16, -2.07e16, -1.9e16;
    double_vec p(4);
    for (int i = 0; i < 4; ++i) {
        p[i] = math::softmax(y, i);
    }
    std::cout << y << std::endl;
    util::print(p);

    /*//  create a Q8Usb object for the board connected to robot motors
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
    core::Daq* q8_mot = new dev::Q8Usb(id_mot, ai_channels, ao_channels, di_channels, do_channels, enc_channels, options);*/


    /*// create and configure a MahiExoII object
    exo::MahiExoII::Config config;
    for (int i = 0; i < exo::MahiExoII::N_rj_; ++i) {
        config.enable_[i] = q8_mot->do_(i+1);
        config.command_[i] = q8_mot->ao_(i+1);
        config.encoder_[i] = q8_mot->encoder_(i+1);
        config.encrate_[i] = q8_mot->encrate_(i+1);
    }
    exo::MahiExoII meii(config);*/

    /*// create and configure a MahiExoIIEmg object
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
    exo::MahiExoIIEmg meii(config);*/

    /*// manual zero joint positions
    if (var_map.count("zero")) {
        q8_mot->enable();
        q8_mot->offset_encoders({ 0, -33259, 29125, 29125, 29125 });
        q8_mot->disable();
        delete q8_mot;
        return 0;
    }*/   

    /*// run state machine
    util::Clock clock(1000);
    util::enable_realtime();
    
    delete q8_mot;
    util::disable_realtime();*/
    return 0;

}