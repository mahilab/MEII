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

#ifndef MEII_EMG_DIRECT_MAPPING_HPP
#define MEII_EMG_DIRECT_MAPPING_HPP

#include <MEII/Regression/RealTimeRegressor.hpp>
#include <MEL/Core/Time.hpp>
#include <vector>

namespace meii {

	class EmgDirectMapping {
	public:

		/// Constructor
		EmgDirectMapping(std::size_t sample_dimension, mel::Time sample_period);

		bool update(std::vector<double> sample);

		/// Provide more baseline data for determining baseline activity for this direct mapping, adding it to existing data. Must be done before predictions can begin. 
		bool add_baseline_data(const std::vector<std::vector<double>>& new_baseline_data);

		/// Clear the baseline data stored for this direct mapping. 
		void clear_baseline_data();

		/// Provide more active data for determining maximal activity for this direct mapping, adding it to existing data. Must be done before predictions can begin. 
		bool add_active_data(const std::vector<std::vector<double>>& new_active_data);

		/// Clear the active data stored for this direct mapping. 
		void clear_active_data();

		/// Fit the regressor to the stored data
		bool fit();

		std::size_t get_output() const;

	private:

		std::size_t sample_dim_; ///< size of sample vector

		mel::Time Ts_; ///< sample period 

		std::vector<std::vector<double>> baseline_data_;

		std::vector<std::vector<double>> active_data_;

		std::vector<double> baseline_;

		std::vector<double> max_;

		double output_;

		RealTimeRegressor regressor_;

	};

} // namespace meii

#endif // MEII_EMG_DIRECT_MAPPING_HPP