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

#ifndef MEII_LOGGING_UTIL_HPP
#define MEII_LOGGING_UTIL_HPP

#include <MEL/Logging/Table.hpp>
#include <MEL/Core/Types.hpp>
#include <vector>

namespace meii {

	class MeiiTable : public mel::Table {
	public:
		/// Constructor
		MeiiTable(const std::string &name = "MeiiTable");

	private:

		std::vector<std::string> joint_coordinates_list_;
		std::vector<std::string> joint_velocities_list_;

		std::vector<double> locked_coordinate_defaults_;
		std::vector<double> locked_coordinates_;

		std::vector<std::string> robot_joint_str = { "ElbowFE", "WristPS", "WristRail1T", "WristRail2T", "WristRail3T"};
		std::vector<std::string> anatomical_joint_str = { "ElbowFE", "WristPS", "WristFE", "WristRU", "ForearmT" };
		std::vector<std::string> passive_coordinates_str = { "WristSlider1R", "WristSlider2R", "WristSlider3R", "WristRingX", "WristRingY", "WristRingZ", "WristRingAlpha", "WristRingBeta", "WristRingGamma" };
		std::vector<std::string> locked_coordinates_str = { "BaseBlockT", "ShoulderAA", "HandleT" };

	};

	class EmgTable : public mel::Table {
	public:
		/// Constructor
		EmgTable(const std::string &name = "EmgTable", std::vector<mel::uint32> emg_channel_numbers = std::vector<mel::uint32>(), bool raw = true, bool dm = false, bool env = false, bool tkeo_env = false);

	private:

		std::vector<std::string> joint_coordinates_list_;
		std::vector<std::string> joint_velocities_list_;

		std::vector<double> locked_coordinate_defaults_;
		std::vector<double> locked_coordinates_;

		std::vector<std::string> robot_joint_str = { "ElbowFE", "WristPS", "WristRail1T", "WristRail2T", "WristRail3T" };
		std::vector<std::string> anatomical_joint_str = { "ElbowFE", "WristPS", "WristFE", "WristRU", "ForearmT" };
		std::vector<std::string> passive_coordinates_str = { "WristSlider1R", "WristSlider2R", "WristSlider3R", "WristRingX", "WristRingY", "WristRingZ", "WristRingAlpha", "WristRingBeta", "WristRingGamma" };
		std::vector<std::string> locked_coordinates_str = { "BaseBlockT", "ShoulderAA", "HandleT" };

	};
    

}

#endif // MEII_LOGGING_UTIL_HPP