#include <MEII/EMG/EmgDataCapture.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Math/Functions.hpp>

using namespace mel;

namespace meii {

    std::vector<double> find_max_window(const std::vector<double>& ref_signal, std::size_t window_size, const std::vector<double>& output_signal) {
        if (ref_signal.empty()) {
            LOG(Warning) << "Input ref_signal to find_max_window() was empty. Returning empty vector.";
            return std::vector<double>();
        }
        if (!output_signal.empty() && output_signal.size() != ref_signal.size()) {
            LOG(Warning) << "Inputs to find_max_window() ref_signal and output_signal must be of the same size. Returning empty vector.";
            return std::vector<double>();
        }
        std::size_t max_idx = std::distance(ref_signal.begin(), std::max_element(ref_signal.begin(), ref_signal.end()));
        if (max_idx < (window_size / 2)) {
            max_idx = window_size / 2;
        }
        if (ref_signal.size() - max_idx < ((window_size + 1) / 2)) {
            max_idx = ref_signal.size() - ((window_size + 1) / 2);
        }
        std::vector<double> signal_window;
        if (output_signal.empty()) {
            signal_window = ref_signal;
        }
        else {
            signal_window = output_signal;
        }
        signal_window.erase(signal_window.begin(), signal_window.begin() + max_idx - (window_size / 2));
        signal_window.erase(signal_window.begin() + window_size, signal_window.end());
        return signal_window;
    }


    std::vector<std::vector<double>> find_sum_max_window(const std::vector<std::vector<double>> &ref_signal, std::size_t window_size, const std::vector<std::vector<double>> &output_signal) {
        if (ref_signal.empty()) {
            LOG(Warning) << "Input to find_sum_max_window() was empty. Returning empty vector.";
            return std::vector<std::vector<double>>();
        }
        if (!output_signal.empty() && output_signal.size() != ref_signal.size()) {
            LOG(Warning) << "Inputs to find_max_sum_window() ref_signal and output_signal must be of the same size. Returning empty vector.";
            return std::vector<std::vector<double>>();
        }
        std::size_t sample_size = ref_signal[0].size();
        for (std::size_t i = 0; i < ref_signal.size(); ++i) {
            if (ref_signal[i].size() != sample_size) {
                LOG(Warning) << "Input to find_sum_max_window() ref_signal contains sample vectors of different sizes. Returning empty vector.";
                return std::vector<std::vector<double>>();
            }
        }
        if (!output_signal.empty()) {
            sample_size = output_signal[0].size();
            for (std::size_t i = 0; i < output_signal.size(); ++i) {
                if (output_signal[i].size() != sample_size) {
                    LOG(Warning) << "Input to find_sum_max_window() output_signal contains sample vectors of different sizes. Returning empty vector.";
                    return std::vector<std::vector<double>>();
                }
            }
        }
        
        std::vector<double> signal_sum(ref_signal.size());
        for (std::size_t i = 0; i < ref_signal.size(); ++i) {
            signal_sum[i] = sum(ref_signal[i]);
        }
        std::size_t max_idx = std::distance(signal_sum.begin(), std::max_element(signal_sum.begin(), signal_sum.end()));
        if (max_idx < (window_size / 2)) {
            max_idx = window_size / 2;
        }
        if (ref_signal.size() - max_idx < ((window_size + 1) / 2)) {
            max_idx = ref_signal.size() - ((window_size + 1) / 2);
        }
        
        std::vector<std::vector<double>> signal_window(window_size);     
        if (output_signal.empty()) {
            for (std::size_t i = 0; i < window_size; ++i) {
                signal_window[i] = ref_signal[i + max_idx - (window_size / 2)];
            }
        }
        else {
            for (std::size_t i = 0; i < window_size; ++i) {
                signal_window[i] = output_signal[i + max_idx - (window_size / 2)];
            }
        }


        return signal_window;
    }

	std::vector<double> max_sample_per_channel(const std::vector<std::vector<double>> signal) {
		if (signal.empty()) {
			LOG(Warning) << "Input to max_sample() was empty. Returning empty vector.";
			return std::vector<double>();
		}
		std::size_t sample_size = signal[0].size();
		for (std::size_t i = 0; i < signal.size(); ++i) {
			if (signal[i].size() != sample_size) {
				LOG(Warning) << "Input to max_sample() signal contains sample vectors of different sizes. Returning empty vector.";
				return std::vector<double>();
			}
		}


		std::vector<double> max(sample_size);
		std::vector<double> channel(signal.size());
		for (std::size_t i = 0; i < sample_size; ++i) {
			for (std::size_t j = 0; j < signal.size(); ++j) {
				channel[j] = signal[j][i];
			}
			max[i] = *std::max_element(channel.begin(), channel.end());
		}
		return max;
	}

} // namespace meii