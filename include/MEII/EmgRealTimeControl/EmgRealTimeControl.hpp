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

#ifndef EMG_REAL_TIME_CONTROL_HPP
#define EMG_REAL_TIME_CONTROL_HPP

namespace meii {

	enum Arm {
		Left, // Left = 0 by default
		Right, // Right = 1
		LastArm
	};

	enum DoF {
		ElbowFE, // ElbowFE = 0 by default
		WristPS, // WristPS = 1
		WristFE, // WristFE = 2
		WristRU, // WristRU = 3
		ElbowFE_and_WristPS, // ElbowFE_and_WristPS = 4
		WristFE_and_WristRU, // WristFE_and_WristRU = 5
		LastDoF
	};

	enum Phase {
		Calibration, // Calibration = 0 by default
		Training, // Training = 1
		BlindTesting, // BlindTesting = 2
		FullTesting, // FullTesting = 3
		LastCondition
	};

} // namespace meii

#endif // EMG_REAL_TIME_CONTROL_HPP