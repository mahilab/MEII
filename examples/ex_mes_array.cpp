#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEL/Communications/Windows/MelShare.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Core/Clock.hpp>
#include <MEL/Logging/DataLogger.hpp>
#include <MEL/Utility/Console.hpp>
#include <MEL/Utility/System.hpp>
#include <MEL/Utility/Windows/Keyboard.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEII/EMG/MesArray.hpp>
#include <MEII/EMG/EmgDataCapture.hpp>

using namespace mel;
using namespace meii;

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

    // initialize logger
    init_logger();

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
    
    // construct Myoelectric Signal (MES) Array
    MesArray mes(q8.analog_input.get_channels(emg_channel_numbers), 300);

    // make MelShares
    MelShare ms_mes_tkeo_env("mes_tkeo_env");

    // create data log for EMG data
    //DataLogger emg_log(WriterType::Buffered, false);
    //std::vector<std::string> emg_log_header;
    //for (std::size_t i = 0; i < emg_channel_count; ++i) {
    //    emg_log_header.push_back("MES TKEO ENV " + stringify(emg_channel_numbers[i]));
    //}
    //emg_log.set_header(emg_log_header);
    //std::vector<double> emg_log_row(emg_log_header.size());   

    // initialize testing conditions
    bool stop = false;
    Time Ts = milliseconds(1); // sample period
    Time mes_capture_period = seconds(3);
    Time mes_select_period = milliseconds(200);
    std::size_t mes_capture_window_size = (std::size_t)((unsigned)(mes_capture_period.as_seconds() / Ts.as_seconds()));
    std::size_t mes_select_window_size = (std::size_t)((unsigned)(mes_select_period.as_seconds() / Ts.as_seconds()));
    mes.resize_buffer(mes_capture_window_size);
    std::vector<std::vector<double>> captured_tkeo_data(mes_capture_window_size);
    std::vector<std::vector<double>> captured_dm_data(mes_capture_window_size);
    std::vector<std::vector<double>> selected_dm_data(mes_select_window_size);
    
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
                    captured_tkeo_data = mes.get_tkeo_env_buffer_data(mes_capture_window_size);
                    captured_dm_data = mes.get_dm_buffer_data(mes_capture_window_size);
                    selected_dm_data = find_sum_max_window(captured_tkeo_data, mes_select_window_size, captured_dm_data);
                    LOG(Info) << "Captured data.";
                    keypress_refract_clock.restart();
                }
            }
        }
 
        // write to MelShares
        ms_mes_tkeo_env.write_data(mes.get_tkeo_envelope());

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

    DataLogger captured_tkeo_datalog(WriterType::Buffered, false);
    for (std::size_t i = 0; i < captured_tkeo_data.size(); ++i) {
        captured_tkeo_datalog.buffer(captured_tkeo_data[i]);
    }
    captured_tkeo_datalog.save_data("ex_mes_array_captured_tkeo_datalog.csv", ".", false);
    captured_tkeo_datalog.wait_for_save();

    DataLogger captured_dm_datalog(WriterType::Buffered, false);
    for (std::size_t i = 0; i < captured_dm_data.size(); ++i) {
        captured_dm_datalog.buffer(captured_dm_data[i]);
    }
    captured_dm_datalog.save_data("ex_mes_array_captured_dm_datalog.csv", ".", false);
    captured_dm_datalog.wait_for_save();

    DataLogger selected_dm_datalog(WriterType::Buffered, false);
    for (std::size_t i = 0; i < selected_dm_data.size(); ++i) {
        selected_dm_datalog.buffer(selected_dm_data[i]);
    }
    selected_dm_datalog.save_data("ex_mes_array_selected_dm_datalog.csv", ".", false);
    selected_dm_datalog.wait_for_save();

    disable_realtime();   
    return 0;
}

