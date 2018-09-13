#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEL/Communications/MelShare.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Core/Clock.hpp>
#include <MEL/Logging/DataLogger.hpp>
#include <MEL/Core/Console.hpp>
#include <MEL/Utility/System.hpp>
#include <MEL/Devices/Windows/Keyboard.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEII/EMG/MesArray.hpp>
#include <MEII/Classification/EmgActiveEnsClassifier.hpp>
#include <MEII/EMG/EmgDataCapture.hpp>
#include <MEL/Utility/Options.hpp>

using namespace mel;
using namespace meii;

ctrl_bool stop(false);
bool handler(CtrlEvent event) {
    stop = true;
    return true;
}

int main(int argc, char *argv[]) {

    // handle inputs 
    std::vector<uint32> emg_channel_numbers;
    if (argc > 1) {
        uint32 ch;
        for (int i = 1; i < argc; ++i) {
            std::stringstream ss(argv[i]);
            ss >> ch;
            if (ch >= 0 && ch < 8) {
                emg_channel_numbers.push_back(ch);
            }
        }
    }
    else {
        return 0;
    }

    // enable Windows realtime
    enable_realtime();  

    // register ctrl-c handler
    register_ctrl_handler(handler);   

    // construct Q8 USB and configure    
	Q8Usb q8;
	q8.open();

    q8.DO.set_enable_values(std::vector<Logic>(8, High));
    q8.DO.set_disable_values(std::vector<Logic>(8, High));
    q8.DO.set_expire_values(std::vector<Logic>(8, High));
    if (!q8.identify(7)) {
        LOG(Error) << "Incorrect DAQ";
        return 0;
    }
    emg_channel_numbers = q8.AI.get_channel_numbers();
    std::size_t emg_channel_count = q8.AI.get_channel_count();
    
    // construct array of Myoelectric Signals    
    MesArray mes(q8.AI.get_channels(emg_channel_numbers));

    // make MelShares
    MelShare ms_mes_tkeo_env("mes_tkeo_env");
    MelShare ms_active_state("active_state");

    // create data log for EMG data
    DataLogger emg_log(WriterType::Buffered, false);
    std::vector<std::string> emg_log_header;
    emg_log_header.push_back("Time [s]");
    for (std::size_t i = 0; i < emg_channel_count; ++i) {
        emg_log_header.push_back("MES TKEO ENV " + stringify(emg_channel_numbers[i]));
    }
    emg_log.set_header(emg_log_header);
    std::vector<double> emg_log_row(emg_log_header.size());   

    // initialize testing conditions
    Time Ts = milliseconds(1); // sample period
    std::size_t num_classes = 2; // number of active classes    
    Time mes_rest_capture_period = seconds(1);
    Time mes_active_capture_period = seconds(3);
    Time mes_active_period = milliseconds(200);
    std::size_t mes_rest_capture_window_size = (std::size_t)((unsigned)(mes_rest_capture_period.as_seconds() / Ts.as_seconds()));
    std::size_t mes_active_capture_window_size = (std::size_t)((unsigned)(mes_active_capture_period.as_seconds() / Ts.as_seconds()));
    mes.resize_buffer(std::max(mes_rest_capture_window_size, mes_active_capture_window_size));
    std::size_t mes_active_window_size = (std::size_t)((unsigned)(mes_active_period.as_seconds() / Ts.as_seconds()));
    bool active_detector_computed = false;
    std::size_t active_state = 0;
    std::vector<Key> active_keys = { Key::Num1, Key::Num2, Key::Num3, Key::Num4 };

    // initialize classifier
    EmgActiveEnsClassifier active_detector(emg_channel_count, Ts);
    active_detector.resize(num_classes);
    
    // enable DAQ
    q8.enable();   

    // construct clock to regulate interaction
    Clock keypress_refract_clock;
    
    Time keypress_refract_time = seconds((double)((signed)mes.get_buffer_capacity()) * Ts.as_seconds());

    // construct timer in hybrid mode to avoid using 100% CPU
    Timer timer(Ts, Timer::Hybrid);

    // start while loop
    q8.watchdog.start();

    // promt the user for input
    print("Press 'A + 0' to add 'rest' state training data to all classifiers.");
    print("Press 'C + 0' to clear 'rest' state training data from all classifiers.");
    print("Press 'A + target #' to add 'active' state training data for one classifier.");
    print("Press 'C + target #' to clear 'active' state training data for one classifier.");
    print("Number of 'active' state classifiers is:");
    print(num_classes);
    print("Press 'T' to train classifier and begin real-time classification.");
    print("Press 'Escape' to exit.");
    
    while (!stop) {

        // update all DAQ input channels
        q8.update_input();
           
        // emg signal processing
        mes.update_and_buffer();

        // write to emg data log
        emg_log_row = mes.get_tkeo_envelope();
        emg_log_row.insert(emg_log_row.begin(), timer.get_elapsed_time().as_seconds());
        emg_log.buffer(emg_log_row);
        
        // predict state
        if (active_detector.update(mes.get_tkeo_envelope()))
            active_state = active_detector.get_class();

        // clear rest data
        if (Keyboard::are_all_keys_pressed({ Key::C, Key::Num0 })) {
            if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                bool cleared_successfully = true;
                for (std::size_t k = 0; k < num_classes; ++k) {
                    if (!active_detector.clear_training_data(k, 0))
                        cleared_successfully = false;
                }
                if (cleared_successfully)
                    LOG(Info) << "Cleared rest data.";
                keypress_refract_clock.restart();
            }
        }

        // capture rest data
        if (Keyboard::are_all_keys_pressed({ Key::A, Key::Num0 })) {
            if (mes.is_buffer_full()) {
                if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                    bool added_successfully = true;
                    for (std::size_t k = 0; k < num_classes; ++k) {
                        if (!active_detector.add_training_data(k, 0, mes.get_tkeo_env_buffer_data(mes_rest_capture_window_size)))
                            added_successfully = false;
                    }
                    if (added_successfully)
                        LOG(Info) << "Added rest data.";
                    keypress_refract_clock.restart();
                }
            }
        }

        // clear active data
        for (std::size_t k = 0; k < num_classes; ++k) {
            if (Keyboard::are_all_keys_pressed({ Key::C, active_keys[k] })) {
                if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                    if (active_detector.clear_training_data(k, 1))
                        LOG(Info) << "Cleared active data for target " + stringify(k + 1) + ".";
                    keypress_refract_clock.restart();
                }
            }
        }

        // capture active data
        for (std::size_t k = 0; k < num_classes; ++k) {
            if (Keyboard::are_all_keys_pressed({ Key::A, active_keys[k] })) {
                if (mes.is_buffer_full()) {
                    if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                        if (active_detector.add_training_data(k, 1, find_sum_max_window(mes.get_tkeo_env_buffer_data(mes_active_capture_window_size), mes_active_window_size)))
                            LOG(Info) << "Added active data for target " + stringify(k + 1) + ".";
                        keypress_refract_clock.restart();
                    }
                }
            }
        }

        // train the active/rest classifiers
        if (Keyboard::is_key_pressed(Key::T)) {
            if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                if (active_detector.train())
                    LOG(Info) << "Trained new active/rest classifier based on given data.";
                keypress_refract_clock.restart();
            }
        }

        // exit calibration and save the computed classifier
        if (Keyboard::is_key_pressed(Key::Enter)) {
            active_detector_computed = true;
        }        

        // write to MelShares
        ms_mes_tkeo_env.write_data(mes.get_tkeo_envelope()); 
        ms_active_state.write_data({ (double)active_state });

        // check for exit key
        if (Keyboard::is_key_pressed(Key::Escape)) {
            stop = true;
        }

        // kick watchdog
        if (!q8.watchdog.kick())
            stop = true;

        // wait for remainder of sample period
        timer.wait();

    } // end while loop       

    if (active_detector_computed) {
        // save to file

        print("Do you want to save the EMG data log? (Y/N)");
        Key key = Keyboard::wait_for_any_keys({ Key::Y, Key::N });
        if (key == Key::Y) {
            emg_log.save_data("test_active_classifier_emg_data_log.csv", ".", false);
            emg_log.wait_for_save();
            emg_log.clear_data();
            sleep(seconds(0.5));
        }
    }

    

    disable_realtime();
    return 0;
}

