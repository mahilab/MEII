// MIT License
//
// MEII - MAHI Exo-II Library
// Copyright (c) 2020 Mechatronics and Haptic Interfaces Lab - Rice University
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

#pragma once

#include <Mahi/Util/Timing/Time.hpp>
#include <Mahi/Util/Math/Butterworth.hpp>

namespace meii {

	class DisturbanceObserver {

	public:

		/// Constructor
		DisturbanceObserver(mahi::util::Time Ts_);

        /// Constructor
		DisturbanceObserver(mahi::util::Time Ts_, double z_0);

		/// Updates the parameters of the Disturbance observer based on the current positions and velocities
		void update(const double x, const double x_dot, const double T_prev, const double delta_t,const mahi::util::Time &t);

        /// Returns the value of the estimated disturbance
        double get_d_hat() const;

	private:

        double c; ///< parameter dictating convergence rate
		double C; ///< viscous friciton parameter
		double J; ///< inertia of joint
		double L; ///< Variable for the DO
        double z; ///< auxilary variable
        double d_hat; ///< estimated disturbance
        mahi::util::Time Ts; ///< sampling rate for update equation
		mahi::util::Butterworth butt;
	};

} // namespace meii
