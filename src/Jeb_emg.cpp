#include <MEL/Logging/Log.hpp>
#include <MEL/Core/Console.hpp>
#include <MEL/Communications/MelShare.hpp>
#include <MEII/EMG/MesArray.hpp>
#include "MEL/Daq/Quanser/Q8Usb.hpp"
#include "MEL/Utility/System.hpp"
#include <MEII/EMG/MyoelectricSignal.hpp>
#include "MEL/Devices/Windows/Keyboard.hpp"
#include "MEII/Classification/EmgDirClassifier.hpp"
#include <MEL/Core/Clock.hpp>
#include <MEL/Logging/DataLogger.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEII/EMG/EmgDataCapture.hpp>
#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEL/Devices/VoltPaqX4.hpp>
#include <MEL/Devices/Myo/MyoBand.hpp>
#include <MEII/EMG/EmgDataCapture.hpp>
#include <MEII/EmgRealTimeControl/EmgRealTimeControl.hpp>

using namespace mel;

using namespace mel;
using namespace meii;

ctrl_bool ctrlc(false);
bool handler(CtrlEvent event) {
    ctrlc = true;
    return true;
}

int main(int argc, char *argv[]) {

    // handle inputs
    std::vector<uint32> emg_channel_numbers = { 0,1,2,3,4,5,6,7 };

    mel::MyoBand myo("my_myo");

    // initialize logger
    //mel::init_logger(mel::Verbose);

    // register ctrl-c handler
    register_ctrl_handler(handler);

    // construct array of Myoelectric Signals
    MesArray mes(myo.get_channels(emg_channel_numbers));

    // make MelShares
    MelShare ms_mes_env("mes_env");
    MelShare ms_mes_dm("mes_dm");
    MelShare ms_pred_label("pred_label");

    // initialize testing conditions
    Time Ts = milliseconds(1); // sample period
    std::size_t num_classes = 7; // number of active classes
    std::vector<Key> active_keys = { Key::Num1, Key::Num2, Key::Num3, Key::Num4, Key::Num5, Key::Num6, Key::Num7 };
    Time mes_active_capture_period = seconds(0.2);
    std::size_t mes_active_capture_window_size = (std::size_t)((unsigned)(mes_active_capture_period.as_seconds() / Ts.as_seconds()));
    mes.resize_buffer(mes_active_capture_window_size);
    std::size_t pred_label = 0;
    std::size_t prev_pred_label = 0;

    //// make Q8 USB that's configured for current control with VoltPAQ-X4
    //QOptions qoptions;
    //qoptions.set_update_rate(QOptions::UpdateRate::Fast);
    //qoptions.set_analog_output_mode(0, QOptions::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
    //qoptions.set_analog_output_mode(1, QOptions::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
    //qoptions.set_analog_output_mode(2, QOptions::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
    //Q8Usb q8(qoptions);

    //VoltPaqX4 vpx4(q8.digital_output[{ 0, 1, 2 }], q8.analog_output[{ 0, 1, 2 }], q8.digital_input[{0, 1, 2}], q8.analog_input[{ 0, 1, 2 }]);

    // create OpenWrist and bind Q8 channels to it
    //OwConfiguration config(
    //    q8,
    //    q8.watchdog,
    //    q8.encoder[{ 0, 1, 2 }],
    //    q8.velocity[{ 0, 1, 2 }],
    //    vpx4.amplifiers
    //);
    //OpenWrist ow(config);

    //mel::PdController pd0(25, 1.15); // OpenWrist Joint 0 (PS)
    //mel::PdController pd1(20, 1.00); // OpenWrist Joint 1 (FE)
    //mel::PdController pd2(20, 0.25); // OpenWrist Joint 2 (RU)

    //double ps_goal = 0.0;
    //double fe_goal = 0.0;
    //double ru_goal = 0.0;

    //double move_speed = 30 * DEG2RAD; // 30 deg/s

    // initialize classifier
    bool RMS = true;
    bool MAV = false;
    bool WL = false;
    bool ZC = false;
    bool SSC = false;
    bool AR1 = false;
    bool AR2 = false;
    bool AR3 = false;
    bool AR4 = false;
    EmgDirClassifier dir_classifier(num_classes, emg_channel_numbers.size(), Ts, RMS, MAV, WL, ZC, SSC, AR1, AR2, AR3, AR4, seconds(1.0), seconds(0.2), seconds(0.9));

    bool run = false;
    Clock cooldown_clock;

    // construct clock to regulate interaction
    Clock keypress_refract_clock;
    Time keypress_refract_time = seconds((double)((signed)mes.get_buffer_capacity()) * Ts.as_seconds());

    // construct timer in hybrid mode to avoid using 100% CPU
    Timer timer(Ts, Timer::Hybrid);

    // prompt the user for input
    print("Press 'A + target #' to add training data for one target.");
    print("Press 'C + target #' to clear training data for one target.");
    print("Number of targets/classes is:");
    print(num_classes);
    print("Press 'T' to train classifier and begin real-time classification.");
    print("Press 'R' to run the OpenWrist");
    print("Press 'S' to stop the OpenWrist");
    print("Press 'Escape' to exit.");

    // enable hardware
    //q8.enable();
    //ow.enable();
    //q8.watchdog.start();
    myo.enable();

    while (!ctrlc) {

        // update hardware
        //q8.watchdog.kick();
        //q8.update_input();

        // update all DAQ input channels
        myo.update();

        // emg signal processing
        mes.update_and_buffer();

        // predict state
        if (dir_classifier.update(mes.get_demean())) {
            pred_label = dir_classifier.get_class();
        }

        // clear active data
        for (std::size_t k = 0; k < num_classes; ++k) {
            if (Keyboard::are_all_keys_pressed({ Key::C, active_keys[k] })) {
                if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                    if (dir_classifier.clear_training_data(k)) {
                        LOG(Info) << "Cleared active data for target " + stringify(k + 1) + ".";
                    }
                    keypress_refract_clock.restart();
                }
            }
        }

        // capture active data
        for (std::size_t k = 0; k < num_classes; ++k) {
            if (Keyboard::are_all_keys_pressed({ Key::A, active_keys[k] })) {
                if (mes.is_buffer_full()) {
                    if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                        if (dir_classifier.add_training_data(k, mes.get_dm_buffer_data(mes_active_capture_window_size))) {
                            LOG(Info) << "Added active data for target " + stringify(k + 1) + ".";
                        }
                        keypress_refract_clock.restart();
                    }
                }
            }
        }

        // train the active/rest classifiers
        if (Keyboard::is_key_pressed(Key::T)) {
            if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                if (dir_classifier.train()) {
                    LOG(Info) << "Trained new active/rest classifier based on given data.";
                }
                keypress_refract_clock.restart();
            }
        }

        // set OpenWrist run state
        if (Keyboard::is_key_pressed(Key::R) && keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
            LOG(Info) << "OpenWrist Running";
            run = true;
            keypress_refract_clock.restart();
        }
        else if (Keyboard::is_key_pressed(Key::S) && keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
            LOG(Info) << "OpenWrist Stopped";
            run = false;
            keypress_refract_clock.restart();
        }

         //fake prediction
        //if (Keyboard::is_key_pressed(Key::Z))
        //    pred_label = 0;
        //else if (Keyboard::is_key_pressed(Key::X))
        //    pred_label = 1;
        //else if (Keyboard::is_key_pressed(Key::C))
        //    pred_label = 2;
        //else if (Keyboard::is_key_pressed(Key::V))
        //    pred_label = 3;
        //else if (Keyboard::is_key_pressed(Key::B))
        //    pred_label = 4;
        //else if (Keyboard::is_key_pressed(Key::N))
        //    pred_label = 5;
        //else if (Keyboard::is_key_pressed(Key::M))
        //    pred_label = 6;

        // set OpenWrist goal
        if (run) {
        //    if (cooldown_clock.get_elapsed_time() > seconds(0.5)) {
        //        if (pred_label == 0) {
        //            // Rest
        //            ps_goal = ow[0].get_position();
        //            fe_goal = ow[1].get_position();
        //            ru_goal = ow[2].get_position();
        //        }
        //        if (pred_label == 1) {
        //            // Flexion
        //            ps_goal = ow[0].get_position();
        //            fe_goal = 60 * mel::DEG2RAD;
        //            ru_goal = ow[2].get_position();
        //        }
        //        else if (pred_label == 2) {
        //            // Extension
        //            ps_goal = ow[0].get_position();
        //            fe_goal = -60 * mel::DEG2RAD;
        //            ru_goal = ow[2].get_position();
        //        }
        //        else if (pred_label == 3) {
        //            // Radial Deviation
        //            ps_goal = ow[0].get_position();
        //            fe_goal = ow[1].get_position();
        //            ru_goal = 30 * mel::DEG2RAD;
        //        }
        //        else if (pred_label == 4) {
        //            // Ulnar Deviation
        //            ps_goal = ow[0].get_position();
        //            fe_goal = ow[1].get_position();
        //            ru_goal = -30 * mel::DEG2RAD;
        //        }
        //        else if (pred_label == 5) {
        //            // Pronation
        //            ps_goal = 80 * mel::DEG2RAD;
        //            fe_goal = ow[1].get_position();
        //            ru_goal = ow[2].get_position();
        //        }
        //        else if (pred_label == 6) {
        //            // Supination
        //            ps_goal = -80 * mel::DEG2RAD;
        //            fe_goal = ow[1].get_position();
        //            ru_goal = ow[2].get_position();
        //        }
        //        cooldown_clock.restart(); // restart so OpenWrist goal is only change once per half second
        //    }
        }
        //else {
        //    ps_goal = 0.0;
        //    fe_goal = 0.0;
        //    ru_goal = 0.0;
        //}

        //// set previous label
        //prev_pred_label = pred_label;

        //// set OpenWrist torques
        //ow[0].set_torque(pd0.move_to_hold(ps_goal, ow[0].get_position(),
        //    move_speed, ow[0].get_velocity(),
        //    0.001, mel::DEG2RAD, 10 * mel::DEG2RAD));
        //ow[0].add_torque(ow.compute_gravity_compensation(0));

        //ow[1].set_torque(pd1.move_to_hold(fe_goal, ow[1].get_position(),
        //    move_speed, ow[1].get_velocity(),
        //    0.001, mel::DEG2RAD, 10 * mel::DEG2RAD));
        //ow[0].add_torque(ow.compute_gravity_compensation(1));

        //ow[2].set_torque(pd2.move_to_hold(ru_goal, ow[2].get_position(),
        //    move_speed, ow[2].get_velocity(),
        //    0.001, mel::DEG2RAD, 10 * mel::DEG2RAD));
        //ow[0].add_torque(ow.compute_gravity_compensation(2));


        // write to MelShares
        ms_mes_env.write_data(mes.get_envelope());
        ms_mes_dm.write_data(mes.get_demean());
        ms_pred_label.write_data({ (double)((signed)(pred_label + 1)) });

        // check limits
        //if (ow.any_limit_exceeded()) {
        //    ctrlc == true;
        //}

        // check for exit key
        if (Keyboard::is_key_pressed(Key::Escape)) {
            ctrlc = true;
        }

        // update hardware
        //q8.update_output();

        // wait for remainder of sample period
        timer.wait();

    } // end control loop

    return 0;
}

