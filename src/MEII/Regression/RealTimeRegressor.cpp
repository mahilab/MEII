#include <MEII/Regression/RealTimeRegressor.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEII/SignalProcessing/SignalProcessingFunctions.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Logging/DataLogger.hpp>

using namespace mel;

namespace meii {

	RealTimeRegressor::RealTimeRegressor(std::size_t sample_dimension, Time sample_period) :
		sample_dim_(sample_dimension),
		Ts_(sample_period),
		trained_(false)
	{ }

	bool RealTimeRegressor::update(std::vector<double> sample) {
		if (!trained_)
			return false;
		if (sample.size() != sample_dim_) {
			LOG(Error) << "Incorrect size for sample given to RealTimeRegressor::update().";
			return false;
		}

		y_ = w_0_;
		for (std::size_t i = 0; i < sample_dim_; ++i) {
			y_ += w_[i] * sample[i];
		}
		return true;
	}

	bool RealTimeRegressor::set_model(const std::vector<double>& w, double w_0) {
		if (w.size() != sample_dim_) {
			LOG(Warning) << "Size of model provided to RealTimeRegressor::set_model() must match the size of the sample dimension. Model not set.";
			return trained_ = false;
		}
		w_ = w;
		w_0_ = w_0;
		return trained_ = true;
	}

	bool RealTimeRegressor::set_model(std::vector<double> w_full) {
		if (w_full.size() != sample_dim_ + 1) {
			LOG(Warning) << "Size of model provided to RealTimeRegressor::set_model() must match the size of the sample dimension. Model not set.";
			return trained_ = false;
		}
		w_0_ = w_full.back();
		w_full.pop_back();
		w_ = w_full;
		return trained_ = true;
	}

	std::size_t RealTimeRegressor::get_pred() const {
		return y_;
	}

	std::vector<double> RealTimeRegressor::get_model() const {
		std::vector<double> w_full = w_;
		w_full.push_back(w_0_);
		return w_full;
	}

	bool RealTimeRegressor::is_trained() {
		return trained_;
	}

	std::size_t RealTimeRegressor::get_sample_dim() const {
		return sample_dim_;
	}
	

} // namespace meii