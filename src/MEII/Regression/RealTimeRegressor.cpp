#include <MEII/Regression/RealTimeRegressor.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEII/SignalProcessing/SignalProcessingFunctions.hpp>
#include <MEL/Logging/Log.hpp>

using namespace mel;

namespace meii {

	RealTimeRegressor::RealTimeRegressor(std::size_t sample_dimension, std::size_t prediction_dimension, Time sample_period) :
		sample_dim_(sample_dimension),
		pred_dim_(prediction_dimension),
		Ts_(sample_period),
		w_(prediction_dimension),
		w_0_(prediction_dimension),
		y_(prediction_dimension),
		trained_(false)
	{ }

	bool RealTimeRegressor::update(const std::vector<double> &sample) {
		if (!trained_) {
			return false;
		}
		if (sample.size() != sample_dim_) {
			LOG(Error) << "Incorrect size for sample given to RealTimeRegressor::update().";
			return false;
		}

		y_ = w_0_;
		for (std::size_t i = 0; i < pred_dim_; ++i) {
			for (std::size_t j = 0; j < sample_dim_; ++j) {
				y_[i] += w_[i][j] * sample[j];
			}
		}

		return true;
	}

	bool RealTimeRegressor::set_model(const std::vector<std::vector<double>> &w, const std::vector<double> &w_0) {
		if (w.size() != pred_dim_) {
			LOG(Warning) << "Regression model size must match the prediction dimension. Model not set.";
			return trained_ = false;
		}
		if (w_0.size() != pred_dim_) {
			LOG(Warning) << "Regression model intercept size must match the prediction dimension. Model not set.";
			return trained_ = false;
		}
		for (std::size_t i = 0; i < pred_dim_; ++i) {
			if (w[i].size() != sample_dim_) {
				LOG(Warning) << "Regression model inner dimension size must match the sample dimension. Model not set.";
				return trained_ = false;
			}
		}
		w_ = w;
		w_0_ = w_0;
		return trained_ = true;
	}

	const std::vector<double>& RealTimeRegressor::get_pred() const {
		return y_;
	}

	void RealTimeRegressor::get_model(std::vector<std::vector<double>> &w, std::vector<double> &w_0) {
		w = w_;
		w_0 = w_0_;
	}

	bool RealTimeRegressor::is_trained() const {
		return trained_;
	}

	std::size_t RealTimeRegressor::get_sample_dim() const {
		return sample_dim_;
	}

	std::size_t RealTimeRegressor::get_pred_dim() const {
		return pred_dim_;
	}
	
	bool RealTimeRegressor::save(const std::string &filename, const std::string& directory, bool timestamp) {
		return DataLogger::write_to_csv(make_datalog(), filename, directory, timestamp);
	}

	bool RealTimeRegressor::load(const std::string &filename, const std::string& directory) {
		std::vector<Table> tables;
		if (DataLogger::read_from_csv(tables, filename, directory)) {
			return read_datalog(tables);
		}
		else {
			return false;
		}
	}

	std::vector<mel::Table> RealTimeRegressor::make_datalog() const {
		std::vector<Table> tables;

		Table params("Parameters", { "sample_dim", "pred_dim", "Ts", "trained" });
		std::vector<double> params_values;
		params_values.push_back((double)sample_dim_);
		params_values.push_back((double)pred_dim_);
		params_values.push_back(Ts_.as_seconds());
		params_values.push_back((double)trained_);
		params.set_values({ params_values });
		tables.push_back(params);

		Table model("Model");
		for (std::size_t i = 0; i < sample_dim_; ++i) {
			model.push_back_col("w_" + stringify(i));
		}
		if (trained_) {
			for (std::size_t i = 0; i < pred_dim_; ++i) {
				model.push_back_row(w_[i]);
			}
			model.push_back_col("intercept", w_0_);
		}
		else {
			model.push_back_col("intercept");
		}
		tables.push_back(model);

		return tables;
	}

	bool RealTimeRegressor::read_datalog(const std::vector<mel::Table> &tables) {
		if (tables.empty()) {
			LOG(Warning) << "Contents of file given to RealTimeRegressor::read_datalog() are invalid. Incorrect number of Tables.";
			return false;
		}

		if (tables[0].name().compare("Parameters") == 0) {
			sample_dim_ = (std::size_t)tables[0](0, 0);
			pred_dim_ = (std::size_t)tables[0](0, 1);
			Ts_ = seconds(tables[0](0, 2));
			trained_ = (bool)tables[0](0, 3);
		}
		else {
			LOG(Warning) << "Contents of file given to RealTimeRegressor::read_datalog() are invalid. No Parameters Table.";
			return false;
		}

		if (tables.size() != 2) {
			LOG(Warning) << "Contents of file given to RealTimeRegressor::read_datalog() are invalid. Incorrect number of Tables.";
			return false;
		}

		w_ = tables[1].values();
		for (std::size_t i = 0; i < w_.size(); ++i) {
			w_0_[i] = w_[i].back();
			w_[i].pop_back();
		}

		return true;
	}

} // namespace meii