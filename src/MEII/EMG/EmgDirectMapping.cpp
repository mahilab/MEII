#include <MEII/EMG/EmgDirectMapping.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEII/EMG/EmgDataCapture.hpp>
#include <MEL/Math/Functions.hpp>

using namespace mel;

namespace meii {

	EmgDirectMapping::EmgDirectMapping(std::size_t sample_dimension, mel::Time sample_period) :
		sample_dim_(sample_dimension),
		Ts_(sample_period),
		regressor_(sample_dimension, sample_period),
		baseline_(sample_dimension),
		max_(sample_dimension)
	{}

	bool EmgDirectMapping::update(std::vector<double> sample) {
		return regressor_.update(sample);
	}

	bool EmgDirectMapping::add_baseline_data(const std::vector<std::vector<double>>& new_baseline_data) {
		if (new_baseline_data.empty()) {
			LOG(Warning) << "Input provided to EmgDirectMapping::add_baseline_data() was empty.  Baseline data was not added.";
			return false;
		}
		for (std::size_t i = 0; i < new_baseline_data.size(); ++i) {
			if (new_baseline_data[i].size() != sample_dim_) {
				LOG(Warning) << "Baseline data provided to EmgDirectMapping::add_baseline_data() has incorrect size for the sample dimension. Baseline data was not added.";
				return false;
			}
		}
		baseline_data_.insert(baseline_data_.end(), new_baseline_data.begin(), new_baseline_data.end());
		return true;
	}

	void EmgDirectMapping::clear_baseline_data() {
		baseline_data_.clear();
	}

	bool EmgDirectMapping::add_active_data(const std::vector<std::vector<double>>& new_active_data) {
		if (new_active_data.empty()) {
			LOG(Warning) << "Input provided to EmgDirectMapping::add_active_data() was empty.  Active data was not added.";
			return false;
		}
		for (std::size_t i = 0; i < new_active_data.size(); ++i) {
			if (new_active_data[i].size() != sample_dim_) {
				LOG(Warning) << "Active data provided to EmgDirectMapping::add_active_data() has incorrect size for the sample dimension. Active data was not added.";
				return false;
			}
		}
		active_data_.insert(active_data_.end(), new_active_data.begin(), new_active_data.end());
		return true;
	}

	void EmgDirectMapping::clear_active_data() {
		active_data_.clear();
	}

	bool EmgDirectMapping::fit() {
		if (baseline_data_.empty()) {
			LOG(Warning) << "Cannot fit EmgDirectMapping because there is no baseline data.";
			return false;
		}
		if (active_data_.empty()) {
			LOG(Warning) << "Cannot fit EmgDirectMapping because there is no active data.";
			return false;
		}
		baseline_.assign(sample_dim_, 0.0);
		double n = (double)baseline_data_.size();
		for (std::size_t i = 0; i < sample_dim_; ++i) {
			for (std::size_t j = 0; j < baseline_data_.size(); ++j) {
				baseline_[i] += baseline_data_[j][i];
			}
			baseline_[i] = baseline_[i] / n;
		}
		max_ = max_sample_per_channel(active_data_);
		//regressor_.set_model();
		return true;
	}

} // namespace meii