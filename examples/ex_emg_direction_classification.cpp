#include "MEL/Daq/Quanser/Q8Usb.hpp"
#include "MEL/Utility/System.hpp"
#include "MEL/Logging/Log.hpp"
#include "MEL/Utility/Console.hpp"
#include "MEII/EMG/MesArray.hpp"
#include "MEL/Communications/MelShare.hpp"
#include "MEL/Utility/Windows/Keyboard.hpp"
#include "MEII/Classification/EmgDirClassifier.hpp"
#include <MEL/Core/Clock.hpp>
#include <MEL/Logging/DataLogger.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEII/EMG/EmgDataCapture.hpp>
#include <iostream>
#include <fstream>

using namespace mel;
using namespace meii;

bool read_csv(std::string filename, std::string directory, std::vector<std::vector<double>>& output) {
	output.clear();
	std::string full_filename = directory + "\\" + filename + ".csv";
	std::ifstream input(full_filename);
	input.precision(12);
	if (input.is_open()) {
		std::string csv_line;
		while (std::getline(input, csv_line)) {
			std::istringstream csv_stream(csv_line);
			std::vector<double> row;
			std::string number;
			double data;
			while (std::getline(csv_stream, number, ',')) {
				std::istringstream number_stream(number);
				number_stream >> data;
				row.push_back(data);
			}
			output.push_back(row);
		}
		return true;
	}
	else {
		LOG(Warning) << "File not found for read_csv().";
		return false;
	}
}

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

    // construct array of Myoelectric Signals    
    MesArray mes(q8.analog_input.get_channels(emg_channel_numbers));

    // make MelShares
    MelShare ms_mes_tkeo_env("mes_tkeo_env");
    MelShare ms_mes_dm("mes_dm");
    MelShare ms_pred_label("pred_label");
	MelShare ms_directory("file_path");
	MelShare ms_filename("file_name");
	MelShare ms_lda_training_flag("lda_training_flag");
	//MelShare ms_numfeats("num_feats");
	MelShare ms_numelec("num_elec");
   

    // create data log for EMG data
    DataLogger training_log(WriterType::Buffered, false);
    std::vector<double> training_log_row;


    // initialize testing conditions
    Time Ts = milliseconds(1); // sample period
    std::size_t num_classes = 2; // number of active classes
    std::vector<Key> active_keys = { Key::Num1, Key::Num2, Key::Num3, Key::Num4 };
    Time mes_active_capture_period = seconds(1);
    Time mes_active_period = milliseconds(200);
    std::size_t mes_active_capture_window_size = (std::size_t)((unsigned)(mes_active_capture_period.as_seconds() / Ts.as_seconds()));
    mes.resize_buffer(mes_active_capture_window_size);
    std::size_t mes_active_window_size = (std::size_t)((unsigned)(mes_active_period.as_seconds() / Ts.as_seconds()));
    std::size_t pred_label = 0;
	std::string directory_ = "C:\\Git\\MEII\\bin\\Release";
	std::string filename_ = "S00_EFE_training_data";
	std::string program_directory_ = "C:\\Git\\MEII\\python";
	std::vector<double> lda_training_complete(1, 0.0);
	std::vector<double> emg_channel_vec(1, emg_channel_count);
	std::vector<std::vector<double>> training_data;
	bool python_return = false;
	std::vector<std::vector<double>> weights;
	std::vector<std::vector<double>> lda_coeffs;
	std::vector<double> lda_intercept;

    // initialize classifier
    bool RMS = true;
    bool MAV = true;
    bool WL = true;
    bool ZC = true;
    bool SSC = true;
    bool AR1 = true;
    bool AR2 = true;
    bool AR3 = true;
    bool AR4 = true;
    EmgDirClassifier dir_classifier(num_classes, emg_channel_count, Ts, RMS, MAV, WL, ZC, SSC, AR1, AR2, AR3, AR4);
    
    // enable DAQ
    q8.enable();

    // construct clock to regulate interaction
    Clock keypress_refract_clock;
    Time keypress_refract_time = seconds((double)((signed)mes.get_buffer_capacity()) * Ts.as_seconds());

    // construct timer in hybrid mode to avoid using 100% CPU
    Timer timer(Ts, Timer::Hybrid);

    // start while loop
    q8.watchdog.start();

	// prompt the user for input
	print("Press 'A + target #' to add training data for one target.");
	print("Press 'C + target #' to clear training data for one target.");
	print("Number of targets/classes is:");
	print(num_classes);
	print("Press 'T' to train classifier and begin real-time classification.");
	print("Press 'Escape' to exit.");

	while (!stop) {

		// update all DAQ input channels
		q8.update_input();

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
						if (dir_classifier.add_training_data(k, find_sum_max_window(mes.get_tkeo_env_buffer_data(mes_active_capture_window_size), mes_active_window_size, mes.get_dm_buffer_data(mes_active_capture_window_size)))) {
							LOG(Info) << "Added active data for target " + stringify(k + 1) + ".";
						}
						keypress_refract_clock.restart();
					}
				}
			}
		}

		// train the direction classifier
		if (Keyboard::is_key_pressed(Key::T)) {
			if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
				//if (dir_classifier.train()) {
				//	LOG(Info) << "Trained new active/rest classifier based on given data.";
				//}
				dir_classifier.compute_training_features();

				//print(dir_classifier.get_class_feature_data(0));

				for (std::size_t i = 0; i < num_classes; ++i) {
					training_data = dir_classifier.get_class_feature_data(i);
					for (std::size_t j = 0; j < training_data.size(); ++j) {
						training_log.buffer(training_data[j]);
					}
				}
				training_log.save_data(filename_, directory_, false);



				// send file location to python over melshare
				ms_numelec.write_data(emg_channel_vec);
				ms_directory.write_message(directory_);
				ms_filename.write_message(filename_);

				// open LDA script in Python
				std::string system_command;
				system_command = "start " + program_directory_ + "\\" + "EMG_FS_LDA_ex.py &";
				system(system_command.c_str());

				// wait for python to return results
				print("Waiting for Python to return training results...");
				
			}
		}

		if (!lda_training_complete.empty() && lda_training_complete[0] == 1.0) {
			if (!python_return) {
				python_return = true;

				read_csv("S00_EFE_emg_dir_classifier", directory_, weights);
				print(lda_coeffs.size());
				for (std::size_t m = 0; m < weights.size(); ++m){
					for (std::size_t n = 0; n < weights[0].size()-2; ++n) {
						print("meow");
						lda_coeffs[m][n] = weights[m][n];
					}
					lda_intercept[m] = weights[m][weights[0].size()];
				}
				print(lda_coeffs);
				print(lda_intercept);

					dir_classifier.set_model(lda_coeffs,lda_intercept);
			}
			print("Python completed");
		}

        // write to MelShares
        ms_mes_tkeo_env.write_data(mes.get_tkeo_envelope());
        ms_mes_dm.write_data(mes.get_demean());
        ms_pred_label.write_data({ (double)((signed)(pred_label + 1)) });
		lda_training_complete = ms_lda_training_flag.read_data();

        // check for exit key
        if (Keyboard::is_key_pressed(Key::Escape)) {
            stop = true;
        }

        // kick watchdog
        if (!q8.watchdog.kick()) {
            stop = true;
        }

        // wait for remainder of sample period
        timer.wait();

    } // end control loop

    // disable Windows realtime and exit the program
    disable_realtime();
    return 0;
}
