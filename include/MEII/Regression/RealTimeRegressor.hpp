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
#include <vector>

namespace meii {

	//==============================================================================
	// CLASS DECLARATION
	//==============================================================================

	class RealTimeRegressor {

	public:

		/// Constructor
		RealTimeRegressor(std::size_t sample_dimension, mel::Time sample_period);

		/// Update called every sample period, taking in new input sample. Must have already done training.
		bool update(std::vector<double> sample);

		/// Provide the linear classification model without training data.
		bool set_model(const std::vector<double>& w, double w_0);
		bool set_model(std::vector<double> w_full);

		/// Get latest prediction of the class label since calling update.
		std::size_t get_pred() const;

		/// Get the vector of w with w_0 on the end used for computing predictions.
		std::vector<double> get_model() const;

		/// Return whether or not the classifier has been trained.
		bool is_trained();

		/// Return the size of the sample space
		std::size_t get_sample_dim() const;


	protected:


	private:

		std::size_t sample_dim_; ///< size of sample vector

		mel::Time Ts_; ///< sample period    

		std::vector<double> w_; ///< regressor weights
		double w_0_; ///< regressor intercept
		double y_; ///< regressor output

		bool trained_; ///< whether or not a regressor has been trained

	};

} // namespace meii

#endif // MEII_REAL_TIME_REGRESSOR_HPP