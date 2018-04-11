#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEL/Communications/Windows/MelShare.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Core/Clock.hpp>
#include <MEL/Logging/DataLogger.hpp>
#include <MEL/Utility/Console.hpp>
#include <MEL/Utility/System.hpp>
#include <MEL/Utility/Windows/Keyboard.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEII/EMG/MyoelectricSignal.hpp>
#include <MEII/EMG/EmgDataCapture.hpp>

using namespace mel;
using namespace meii;

ctrl_bool stop(false);
bool handler(CtrlEvent event) {
    stop = true;
    return true;
}

int main(int argc, char *argv[]) {

    // handle inputs 
    std::vector<uint32> emg_channel_numbers(1);
    if (argc > 1) {
        uint32 ch;
        std::stringstream ss(argv[1]);
        ss >> ch;
        if (ch >= 0 && ch < 8) {
            emg_channel_numbers[0] = ch;
        }
    }
    else {
        return 0;
    }

    // enable Windows realtime
    enable_realtime();  

    // initialize logger
    init_logger();

    // register ctrl-c handler
    register_ctrl_handler(handler);   

    // construct Q8 USB and configure    
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
    if (emg_channel_count != 1) {
        LOG(Error) << "Must only be one EMG channel selected.";
        return 0;
    }
    
    // construct single Myoelectric Signal
    MyoelectricSignal mes(q8.analog_input.get_channels(emg_channel_numbers)[0]);

    // make MelShares
    MelShare ms_mes_tkeo_env("mes_tkeo_env");

    // create data log for EMG data
    DataLogger emg_log(WriterType::Buffered, false);
    std::vector<std::string> emg_log_header;
    emg_log_header.push_back("MES TKEO ENV " + stringify(emg_channel_numbers[0]));
    emg_log.set_header(emg_log_header);
    std::vector<double> emg_log_row(emg_log_header.size());   

    // initialize testing conditions
    Time Ts = milliseconds(1); // sample period
    Time mes_capture_period = seconds(3);
    Time mes_select_period = milliseconds(200);
    std::size_t mes_capture_window_size = (std::size_t)((unsigned)(mes_capture_period.as_seconds() / Ts.as_seconds()));
    std::size_t mes_select_window_size = (std::size_t)((unsigned)(mes_select_period.as_seconds() / Ts.as_seconds()));
    mes.resize_buffer(mes_capture_window_size);
    std::vector<double> captured_data(mes_capture_window_size);
    std::vector<double> selected_data(mes_select_window_size);
    
    // enable DAQ
    q8.enable();   

    // construct clock to regulate interaction
    Clock keypress_refract_clock;
    Time keypress_refract_time = seconds(1);

    // construct timer in hybrid mode to avoid using 100% CPU
    Timer timer(Ts, Timer::Hybrid);

    // start while loop
    q8.watchdog.start();
   
    while (!stop) {

        // update all DAQ input channels
        q8.update_input();
           
        // emg signal processing
        mes.update_and_buffer(); 

        // capture data and select the window surrounding the max value
        if (Keyboard::are_all_keys_pressed({ Key::C })) {
            if (mes.is_buffer_full()) {
                if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                    captured_data = mes.get_tkeo_env_buffer_data(mes_capture_window_size);
                    selected_data = find_max_window(captured_data, mes_select_window_size);                   
                    LOG(Info) << "Added data.";
                    keypress_refract_clock.restart();
                }
            }
        }
 
        // write to MelShares
        ms_mes_tkeo_env.write_data({ mes.get_tkeo_envelope() });

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

    disable_realtime();
    return 0;
}

