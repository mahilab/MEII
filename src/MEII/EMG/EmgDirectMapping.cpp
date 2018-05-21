#include <MEII/EMG/EmgDirectMapping.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEII/EMG/EmgDataCapture.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEL/Logging/DataLogger.hpp>

using namespace mel;

namespace meii {

	EmgDirectMapping::EmgDirectMapping(std::size_t sample_dimension, mel::Time sample_period) :
		sample_dim_(sample_dimension),
		Ts_(sample_period),
		regressor_(sample_dimension, sample_dimension, sample_period),
		baseline_(sample_dimension),
		max_(sample_dimension),
		reg_pred_(sample_dimension),
		scaling_(sample_dimension, 1.0),
		bias_(sample_dimension, 0.0),
		pred_(sample_dimension)
	{
		for (std::size_t i = 0; i < sample_dimension; ++i) {
			channel_map_[i] = i;
		}
	}

	bool EmgDirectMapping::update(std::vector<double> sample) {
		if (!regressor_.update(sample)) {
			return false;
		}
		reg_pred_ = regressor_.get_pred();
		for (std::size_t i = 0; i < sample_dim_; ++i) {
			pred_[channel_map_[i]] = scaling_[i] * saturate(reg_pred_[i], 0.0, 1.0) + bias_[i];
		}
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

		// extract baseline and max from collected data
		baseline_.assign(sample_dim_, 0.0);
		double n = (double)baseline_data_.size();
		for (std::size_t i = 0; i < sample_dim_; ++i) {
			for (std::size_t j = 0; j < baseline_data_.size(); ++j) {
				baseline_[i] += baseline_data_[j][i];
			}
			baseline_[i] = baseline_[i] / n;
		}
		max_ = max_sample_per_channel(active_data_);
		//std::cout << "Baseline values are: " << baseline_ << "\r\n";
		//std::cout << "Max values are: " << max_ << "\r\n";

		// compute linear regression model from baseline and max values
		std::vector<std::vector<double>> w(sample_dim_, std::vector<double>(sample_dim_, 0.0));
		std::vector<double> w_0(sample_dim_, 0.0);
		for (std::size_t i = 0; i < sample_dim_; ++i) {
			if (max_[i] > baseline_[i]) {
				w_0[i] = -baseline_[i] / (max_[i] - baseline_[i]);
				w[i][i] = 1.0 / (max_[i] - baseline_[i]);
			}
		}
		//std::cout << "Linear regression matrix is: " << w << "\r\n";
		//std::cout << "Linear regression intercept is: " << w_0 << "\r\n";
		
		return regressor_.set_model(w, w_0);
	}

	std::vector<double> EmgDirectMapping::get_pred() const {
		return pred_;
	}

	bool EmgDirectMapping::is_fit() const {
		return regressor_.is_trained();
	}

	bool EmgDirectMapping::set_scaling(const std::vector<double> &new_scaling) {
		if (new_scaling.size() != sample_dim_) {
			LOG(Warning) << "Incorrect number of new scaling values provided to EmgDirectMapping. Scaling not set.";
			return false;
		}
		scaling_ = new_scaling;
	}

	bool EmgDirectMapping::set_bias(const std::vector<double> &new_bias) {
		if (new_bias.size() != sample_dim_) {
			LOG(Warning) << "Incorrect number of new bias values provided to EmgDirectMapping. Bias not set.";
			return false;
		}
		bias_ = new_bias;
	}

	bool EmgDirectMapping::set_channel_map(const std::vector<std::size_t> &new_map) {
		if (new_map.size() != sample_dim_) {
			LOG(Warning) << "Incorrect size of new channel map provided to EmgDirectMapping. Mapping not set.";
			return false;
		}
		for (std::size_t i = 0; i < sample_dim_; ++i) {
			if (new_map[i] >= sample_dim_) {
				LOG(Warning) << "Invalid element of new channel map provided to EmgDirectMapping. Mapping not set.";
				return false;
			}
		}
		for (std::size_t i = 0; i < sample_dim_; ++i) {
			channel_map_[i] = new_map[i];
		}
	}

	bool EmgDirectMapping::save(const std::string &filename, const std::string& directory, bool timestamp) {
		return DataLogger::write_to_csv(make_datalog(), filename, directory, timestamp);
	}

	bool EmgDirectMapping::load(const std::string &filename, const std::string& directory) {
		std::vector<Table> tables;
		if (DataLogger::read_from_csv(tables, filename, directory)) {
			return read_datalog(tables);
		}
		else {
			return false;
		}
	}

	std::vector<mel::Table> EmgDirectMapping::make_datalog() const {
		std::vector<Table> tables;

		Table params("Parameters", { "sample_dim", "Ts" });
		std::vector<double> params_values;
		params_values.push_back((double)sample_dim_);
		params_values.push_back(Ts_.as_seconds());
		params.set_values({ params_values });
		tables.push_back(params);

		Table baseline_table("BaselineData");
		for (std::size_t i = 0; i < sample_dim_; ++i) {
			baseline_table.push_back_col("x_" + stringify(i));
		}
		if (!baseline_data_.empty()) {
			baseline_table.set_values(baseline_data_);
		}
		tables.push_back(baseline_table);

		Table active_table("ActiveData");
		for (std::size_t i = 0; i < sample_dim_; ++i) {
			active_table.push_back_col("x_" + stringify(i));
		}
		if (!active_data_.empty()) {
			active_table.set_values(active_data_);
		}
		tables.push_back(active_table);

		Table map_table("ChannelMap");
		std::size_t x = channel_map_.at(0);
		for (std::size_t i = 0; i < sample_dim_; ++i) {
			active_table.push_back_col("ch_" + stringify(i), { (double)channel_map_.at(i) });
		}
		tables.push_back(map_table);

		Table states("States");
		states.push_back_col("baseline", baseline_);
		states.push_back_col("max", max_);
		states.push_back_col("scaling", baseline_);
		states.push_back_col("bias", max_);
		tables.push_back(states);

		std::vector<Table> regressor_datalog = regressor_.make_datalog();
		tables.insert(tables.end(), regressor_datalog.begin(), regressor_datalog.end());

		return tables;
	}

	bool EmgDirectMapping::read_datalog(const std::vector<mel::Table> &tables) {
		if (tables.empty()) {
			LOG(Warning) << "Contents of file given to EmgDirectMapping::read_datalog() are invalid. Incorrect number of Tables.";
			return false;
		}

		if (tables[0].name().compare("Parameters") == 0) {
			sample_dim_ = (std::size_t)tables[0](0, 0);
			Ts_ = seconds(tables[0](0, 1));
		}
		else {
			LOG(Warning) << "Contents of file given to EmgDirectMapping::read_datalog() are invalid. No Parameters Table.";
			return false;
		}

		if (tables.size() != 7) {
			LOG(Warning) << "Contents of file given to EmgDirectMapping::read_datalog() are invalid. Incorrect number of Tables.";
			return false;
		}

		baseline_data_ = tables[1].values();
		active_data_ = tables[2].values();
		std::vector<size_t> new_map(sample_dim_);
		for (std::size_t i = 0; i < sample_dim_; ++i) {
			new_map[i] = (std::size_t)tables[3](0, i);
		}
		
		set_channel_map(new_map);
		
		baseline_ = tables[4].get_col(0);
		max_ = tables[4].get_col(1);
		scaling_ = tables[4].get_col(2);
		bias_ = tables[4].get_col(3);

		std::vector<Table> regressor_datalog(2);
		std::copy(tables.begin() + 5, tables.begin() + 6, regressor_datalog.begin());
		regressor_.read_datalog(regressor_datalog);
	}

	bool EmgDirectMapping::export_training_data(const std::string &filename, const std::string& directory, bool timestamp) const {
		Table active_data_log("EmgDirectMappingActiveData");
		for (std::size_t i = 0; i < sample_dim_; ++i) {
			active_data_log.push_back_col("x_" + stringify(i));
		}
		active_data_log.set_values(active_data_);
		if (!DataLogger::write_to_csv(active_data_log, "active_" + filename, directory, timestamp)) {
			return false;
		}
		Table baseline_data_log("EmgDirectMappingBaselineData");
		for (std::size_t i = 0; i < sample_dim_; ++i) {
			baseline_data_log.push_back_col("x_" + stringify(i));
		}
		baseline_data_log.set_values(baseline_data_);
		if (!DataLogger::write_to_csv(baseline_data_log, "baseline_" + filename, directory, timestamp)) {
			return false;
		}
		return true;
	}

} // namespace meii