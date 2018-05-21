// MIT License
//
// MEII - MAHI Exo-II Extension of MEL, the MAHI Exoskeleton Library
// Copyright (c) 2018 Mechatronics and Haptic Interfaces Lab - Rice University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// Author(s): Craig McDonald (craig.g.mcdonald@gmail.com)

#ifndef MEII_REAL_TIME_REGRESSOR_HPP
#define MEII_REAL_TIME_REGRESSOR_HPP

#include <MEL/Core/Time.hpp>
#include <MEL/Logging/Table.hpp>
#include <vector>

namespace meii {

	//==============================================================================
	// CLASS DECLARATION
	//==============================================================================

	class RealTimeRegressor {

	public:

		/// Constructor
		RealTimeRegressor(std::size_t sample_dimension, std::size_t prediction_dimension, mel::Time sample_period);

		/// Update called every sample period, taking in new input sample. Must have already done training.
		bool update(const std::vector<double> &sample);

		/// Provide the linear regression model without training data.
		bool set_model(const std::vector<std::vector<double>> &w, const std::vector<double> &w_0);

		/// Get latest prediction since calling update.
		const std::vector<double>& get_pred() const;

		/// Get the vector of w with w_0 on the end used for computing predictions.
		void get_model(std::vector<std::vector<double>> &w, std::vector<double> &w_0);

		/// Return whether or not the classifier has been trained.
		bool is_trained() const;

		/// Return the dimension of the sample space
		std::size_t get_sample_dim() const;

		/// Return the dimension of the prediction space
		std::size_t get_pred_dim() const;

		bool save(const std::string &filename = "real_time_regressor", const std::string& directory = ".", bool timestamp = true);

		bool load(const std::string &filename = "real_time_regressor", const std::string& directory = ".");

		std::vector<mel::Table> make_datalog() const;

		bool read_datalog(const std::vector<mel::Table> &tables);

	protected:


	private:

		std::size_t sample_dim_; ///< size of sample vector
		std::size_t pred_dim_; ///< size of prediction vector

		mel::Time Ts_; ///< sample period    

		std::vector<std::vector<double>> w_; ///< regressor weights
		std::vector<double> w_0_; ///< regressor intercept
		std::vector<double> y_; ///< regressor output

		bool trained_; ///< whether or not a regressor has been trained

	};

} // namespace meii

#endif // MEII_REAL_TIME_REGRESSOR_HPP