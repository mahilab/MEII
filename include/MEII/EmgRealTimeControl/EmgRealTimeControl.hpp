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

#include <Eigen/Dense>
#include <vector>

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
		ElbowFE_WristPS, // ElbowFE_WristPS = 4
		WristFE_WristRU, // WristFE_WristRU = 5
		LastDoF
	};

	enum Phase {
		Calibration, // Calibration = 0 by default
		Training, // Training = 1
		BlindTesting, // BlindTesting = 2
		FullTesting, // FullTesting = 3
		LastCondition
	};

	enum IndexBase {
		Zero,
		One
	};

	// return true if given DoF is single-DoF, as opposed to multi-DoF
	bool is_single_dof(DoF dof);

	// generates list of num_labels randomly distributed unsigned integers from zero to num_classes - 1
	std::vector<std::size_t> gen_rand_class_labels(std::size_t num_labels, std::size_t num_classes, IndexBase index_base = One);

	// generates list of num_lables_per_class * num_classes randomly shuffled unsigned integers from zero to num_classes - 1
	std::vector<std::size_t> rand_shuffle_class_labels(std::size_t num_labels_per_class, std::size_t num_classes, IndexBase index_base = One);

	// generates a confusion matrix from labels to evaluate classification performance
	Eigen::MatrixXd gen_confusion_mat(const std::vector<double> &actual_labels, const std::vector<double> &predicted_labels, IndexBase index_base = One);
	Eigen::MatrixXd gen_confusion_mat(const std::vector<std::size_t> &actual_labels, const std::vector<std::size_t> &predicted_labels, IndexBase index_base = One);

} // namespace meii

#endif // EMG_REAL_TIME_CONTROL_HPP