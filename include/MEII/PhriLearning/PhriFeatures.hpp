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

#ifndef MEII_PHRI_FEATURES_HPP
#define MEII_PHRI_FEATURES_HPP

#include <MEII/Utility/Matrix.hpp>
#include <vector>

namespace meii {

    /// Extract vector of features phi encoding the potentially important aspects of the task from the robot's learned desired trajectory
    Matrix feature_extraction(const Matrix &q_d);

    Matrix feature_jacobian(const Matrix &q_d);

    Matrix feature_gradient(const Matrix &q_d, const Matrix &theta);

} // namespace meii


#endif // MEII_PHRI_FEATURES_HPP
