#include <MEL/Core/Timer.hpp>
#include <MEL/Core/Clock.hpp>
#include <MEL/Utility/Console.hpp>
#include <MEL/Utility/System.hpp>
#include <MEL/Utility/Options.hpp>
#include <MEL/Utility/Windows/Keyboard.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Communications/Windows/MelShare.hpp>
#include <MEL/Math/Butterworth.hpp>
#include <MEII/Classification/RealTimeClassifier.hpp>
#include <MEII/Classification/RealTimeMultiClassifier.hpp>
#include <MEII/Classification/EnsembleRTClassifier.hpp>
#include <chrono>
#include <random>

using namespace mel;
using namespace meii;

ctrl_bool stop(false);
bool handler(CtrlEvent event) {
    stop = true;
    return true;
}

int main(int argc, char *argv[]) {

    // make options
    Options options("classification.exe", "Testing Real-Time Classification");
    options.add_options()
        ("b,binary", "Tests binary real-time classifier")
        ("m,multi", "Tests multi-class real-time classifier")
        ("e,ensemble", "Tests ensemble binary real-time classifier");

    auto result = options.parse(argc, argv);

    // enable Windows realtime
    enable_realtime();

    // initialize logger
    init_logger();

    // register ctrl-c handler
    register_ctrl_handler(handler); 
   
    // initialize testing conditions
    bool auto_train = true;
    std::size_t training_data_size = 50;
    Time Ts = milliseconds(1); // sample period
    std::size_t sample_dim = 3; // size of each sample
    std::vector<double> a = { 1.5, 0.5, 3.0 }; // signal amplitudes
    std::vector<double> f = { 1.0, 0.3, 2.2 }; // signal frequencies  
    double n = 0.5; // noise amplitude   
    Butterworth lpf(2, 0.001);

    // initialize data containers
    double white_noise;
    double filt_noise;
    std::vector<double> signal(sample_dim);
    RingBuffer<std::vector<double>> signal_buffer(training_data_size);
    std::size_t true_label = 0;
    std::size_t pred_label = 0;
    RingBuffer<std::size_t> label_buffer(training_data_size);
    std::vector<bool> class_trained;
    std::vector<Key> num_keys = { Key::Num0, Key::Num1, Key::Num2, Key::Num3, Key::Num4, Key::Num5, Key::Num6, Key::Num7, Key::Num8, Key::Num9 };
    
    // construct a random generator engine from a time-based seed
    uint64 seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::normal_distribution<double> distribution(0.0, 1.0);
    
    // construct MelShare for plotting
    MelShare ms_input_signal("input_signal");
    MelShare ms_true_label("true_label");
    MelShare ms_pred_label("pred_label");

    // construct clock to regulate interaction
    Clock keypress_refract_clock;
    Time keypress_refract_time = seconds(1);

    // initialize timer
    Timer timer(Ts, Timer::Hybrid);
    
    print("Open MEL Scope ex_classification.scope.");
    if (result.count("binary") > 0) {

        // initialize classifier
        std::size_t num_classes = 2; // default classifier is binary
        std::vector<std::vector<double>> b = { { 0.2, -0.7, 1.2 },{ -3.2, -5.2, 4.7 } }; // signal offsets depending on true label
        RealTimeClassifier rt_classifier(sample_dim, Ts);
        class_trained = std::vector<bool>(num_classes, false);
       
        if (auto_train) {           

            // add training data
            double t = 0.0;
            for (std::size_t k = 0; k < num_classes; ++k) {
                for (std::size_t j = 0; j < training_data_size; ++j) {
                    white_noise = distribution(generator);
                    for (std::size_t i = 0; i < sample_dim; ++i) {
                        signal[i] = a[i] * std::sin(t * f[i] * 2 * 3.14159) + b[k][i] + n * white_noise;
                        t += Ts.as_seconds();
                    }
                    signal_buffer.push_back(signal);
                }
                if (rt_classifier.add_training_data(k, signal_buffer.get_vector())) {
                    LOG(Info) << "Added training data for class " + stringify(k) + ".";
                }
            }

            // train classifier
            if (rt_classifier.train()) {
                LOG(Info) << "Trained classifier based on given data.";
            }

            print("Press 'Escape' to exit.");
        }
        else {
            // promt the user for input
            print("Press 'A + class label #' to add training data.");
            print("Press 'C + class label #' to clear training data.");
            print("Class labels are from 0 to 1.");
            print("Press 'T' to train classifier and begin real-time classification.");
            print("Press 'Escape' to exit.");
        }

        while (!stop) {

            // generate input signal
            white_noise = distribution(generator);
            if (lpf.update(white_noise) > 0)
                true_label = 0;
            else
                true_label = 1;
            for (std::size_t i = 0; i < sample_dim; ++i) {
                signal[i] = a[i] * std::sin(timer.get_elapsed_time().as_seconds() * f[i] * 2 * 3.14159) + b[true_label][i] + n * white_noise;
            }
            signal_buffer.push_back(signal);

            // predict state
            if (rt_classifier.update(signal))
                pred_label = rt_classifier.get_class();

            if (!auto_train) {

                // clear training data
                for (std::size_t k = 0; k < num_classes; ++k) {
                    if (Keyboard::are_all_keys_pressed({ Key::C, num_keys[k] })) {
                        if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                            if (rt_classifier.clear_training_data(k)) {
                                LOG(Info) << "Cleared training data for class " + stringify(k) + ".";
                            }
                            keypress_refract_clock.restart();
                        }
                    }
                }

                // add training data
                for (std::size_t k = 0; k < num_classes; ++k) {
                    if (Keyboard::are_all_keys_pressed({ Key::A, num_keys[k] })) {
                        if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                            if (rt_classifier.add_training_data(k, signal_buffer.get_vector())) {
                                LOG(Info) << "Added training data for class " + stringify(k) + ".";
                            }
                            keypress_refract_clock.restart();
                        }
                    }
                }

                // train classifier
                if (Keyboard::is_key_pressed(Key::T)) {
                    if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                        if (rt_classifier.train()) {
                            LOG(Info) << "Trained classifier based on given data.";
                        }
                        keypress_refract_clock.restart();
                    }
                }
            }

            // write to melshare
            ms_input_signal.write_data(signal);
            ms_true_label.write_data({ (double)((signed)true_label) });
            ms_pred_label.write_data({ (double)((signed)pred_label) });

            // check for exit key
            if (Keyboard::is_key_pressed(Key::Escape)) {
                stop = true;
            }

            // wait for remainder of sample period
            timer.wait();

        } // end while loop       
    }

    if (result.count("multi") > 0) {

        // initialize classifier
        std::size_t num_classes = 4;
        std::vector<std::vector<double>> b = { { 0.2, -0.7, 1.2 }, { -3.2, -5.2, 4.7 }, { 1.4, -0.0, -3.9 },{ -1.4, 2.0, -0.8 } }; // signal offsets depending on true label
        RealTimeMultiClassifier rt_classifier(num_classes, sample_dim, Ts);       

        if (auto_train) {

            // add training data
            double t = 0.0;
            for (std::size_t k = 0; k < num_classes; ++k) {
                for (std::size_t j = 0; j < training_data_size; ++j) {
                    white_noise = distribution(generator);
                    for (std::size_t i = 0; i < sample_dim; ++i) {
                        signal[i] = a[i] * std::sin(t * f[i] * 2 * 3.14159) + b[k][i] + n * white_noise;
                        t += Ts.as_seconds();
                    }
                    signal_buffer.push_back(signal);
                }
                if (rt_classifier.add_training_data(k, signal_buffer.get_vector())) {
                    LOG(Info) << "Added training data for class " + stringify(k) + ".";
                }
            }

            // train classifier
            if (rt_classifier.train()) {
                LOG(Info) << "Trained classifier based on given data.";
            }

            print("Press 'Escape' to exit.");
        }
        else {
            // promt the user for input
            print("Press 'A + class label #' to add training data.");
            print("Press 'C + class label #' to clear training data.");
            print("Class labels are from 0 to 3.");
            print("Press 'T' to train classifier and begin real-time classification.");
            print("Press 'Escape' to exit.");
        }

        while (!stop) {

            // generate input signal
            white_noise = distribution(generator);
            filt_noise = lpf.update(white_noise);
            if (filt_noise > 0) {
                if (filt_noise > 0.025)
                    true_label = 0;
                else
                    true_label = 1;
            }
            else {
                if (filt_noise > -0.025)
                    true_label = 2;
                else
                    true_label = 3;
            }
            for (std::size_t i = 0; i < sample_dim; ++i) {
                signal[i] = a[i] * std::sin(timer.get_elapsed_time().as_seconds() * f[i] * 2 * 3.14159) + b[true_label][i] + n * white_noise;
            }
            signal_buffer.push_back(signal);

            // predict state
            if (rt_classifier.update(signal))
                pred_label = rt_classifier.get_class();

            if (!auto_train) {

                // clear training data
                for (std::size_t k = 0; k < num_classes; ++k) {
                    if (Keyboard::are_all_keys_pressed({ Key::C, num_keys[k] })) {
                        if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                            if (rt_classifier.clear_training_data(k)) {
                                LOG(Info) << "Cleared training data for class " + stringify(k) + ".";
                            }
                            keypress_refract_clock.restart();
                        }
                    }
                }

                // capture training data
                for (std::size_t k = 0; k < num_classes; ++k) {
                    if (Keyboard::are_all_keys_pressed({ Key::A, num_keys[k] })) {
                        if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                            if (rt_classifier.add_training_data(k, signal_buffer.get_vector())) {
                                LOG(Info) << "Added training data for class " + stringify(k) + ".";
                            }
                            keypress_refract_clock.restart();
                        }
                    }
                }

                // train classifier
                if (Keyboard::is_key_pressed(Key::T)) {
                    if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                        if (rt_classifier.train()) {
                            LOG(Info) << "Trained classifier based on given data.";
                        }
                        keypress_refract_clock.restart();
                    }
                }
            }

            // write to melshare
            ms_input_signal.write_data(signal);
            ms_true_label.write_data({ (double)((signed)true_label) });
            ms_pred_label.write_data({ (double)((signed)pred_label) });

            // check for exit key
            if (Keyboard::is_key_pressed(Key::Escape)) {
                stop = true;
            }

            // wait for remainder of sample period
            timer.wait();

        } // end while loop       
    }

    if (result.count("ensemble") > 0) {

        // initialize classifier
        std::size_t num_classes = 2; // default classifier is binary
        std::vector<std::vector<double>> b = { { 0.2, -0.7, 1.2 },{ -3.2, -5.2, 4.7 } }; // signal offsets depending on true label
        std::size_t ens_size = 5;
        EnsembleRTClassifier rt_classifier(sample_dim, Ts, ens_size);

        // initialize data container
        int number_keypress = 0;
        std::size_t selected_classifier = 0;

        if (auto_train) {

            // add training data
            double t = 0.0;
            for (std::size_t l = 0; l < ens_size; ++l) {
                for (std::size_t k = 0; k < num_classes; ++k) {
                    for (std::size_t j = 0; j < training_data_size; ++j) {
                        white_noise = distribution(generator);
                        for (std::size_t i = 0; i < sample_dim; ++i) {
                            signal[i] = a[i] * std::sin(t * f[i] * 2 * 3.14159) + b[k][i] + n * white_noise;
                            t += Ts.as_seconds();
                        }
                        signal_buffer.push_back(signal);
                    }
                    if (rt_classifier.add_training_data(l, k, signal_buffer.get_vector())) {
                        LOG(Info) << "Added training data for classifier " + stringify(selected_classifier) + " for class " + stringify(k) + ".";
                    }
                }
            }

            // train classifier
            if (rt_classifier.train()) {
                LOG(Info) << "Trained classifier based on given data.";
            }

            print("Press 'Escape' to exit.");
        }       
        else {
            // promt the user for input
            print("Press number keys to select a classifier.");
            print("Number of classifiers used is:");
            print(ens_size);
            print("Press 'A + class label #' to add training data.");
            print("Press 'C + class label #' to clear training data.");
            print("Class labels are from 0 to 1.");
            print("Press 'T' to train classifier and begin real-time classification.");
            print("Press 'Escape' to exit.");
        }

        while (!stop) {

            // generate input signal
            white_noise = distribution(generator);
            if (lpf.update(white_noise) > 0)
                true_label = 0;
            else
                true_label = 1;
            for (std::size_t i = 0; i < sample_dim; ++i) {
                signal[i] = a[i] * std::sin(timer.get_elapsed_time().as_seconds() * f[i] * 2 * 3.14159) + b[true_label][i] + n * white_noise;
            }
            signal_buffer.push_back(signal);

            // predict state
            if (rt_classifier.update(signal))
                pred_label = rt_classifier.get_class();

            if (!auto_train) {

                // clear training data
                for (std::size_t k = 0; k < num_classes; ++k) {
                    if (Keyboard::are_all_keys_pressed({ Key::C, num_keys[k] })) {
                        if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                            if (rt_classifier.clear_training_data(selected_classifier, k)) {
                                LOG(Info) << "Cleared training data for classifier " + stringify(selected_classifier) + " for class " + stringify(k) + ".";
                            }
                            keypress_refract_clock.restart();
                        }
                    }
                }

                // capture training data
                for (std::size_t k = 0; k < num_classes; ++k) {
                    if (Keyboard::are_all_keys_pressed({ Key::A, num_keys[k] })) {
                        if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                            if (rt_classifier.add_training_data(selected_classifier, k, signal_buffer.get_vector())) {
                                LOG(Info) << "Added training data for classifier " + stringify(selected_classifier) + " for class " + stringify(k) + ".";
                            }
                            keypress_refract_clock.restart();
                        }
                    }
                }

                // switch selected classifier
                number_keypress = Keyboard::is_any_num_key_pressed();
                if (number_keypress >= 0) {
                    if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                        if (number_keypress < ens_size) {
                            selected_classifier = (size_t)((unsigned)number_keypress);
                        }

                        keypress_refract_clock.restart();
                    }
                }

                // train classifier
                if (Keyboard::is_key_pressed(Key::T)) {
                    if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                        if (rt_classifier.train()) {
                            LOG(Info) << "Trained classifier based on given data.";
                        }
                        keypress_refract_clock.restart();
                    }
                }
            }

            // write to melshare
            ms_input_signal.write_data(signal);
            ms_true_label.write_data({ (double)((signed)true_label) });
            ms_pred_label.write_data({ (double)((signed)pred_label) });

            // check for exit key
            if (Keyboard::is_key_pressed(Key::Escape)) {
                stop = true;
            }

            // wait for remainder of sample period
            timer.wait();

        } // end while loop       
    }

    disable_realtime();
    return 0;
}

