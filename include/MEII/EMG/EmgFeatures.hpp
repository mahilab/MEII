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

#ifndef MEII_EMG_FEATURES_HPP
#define MEII_EMG_FEATURES_HPP

#include <vector>

namespace meii {

    double mean_rms(const std::vector<double>& mes_window);

    double mean_absolute_value(const std::vector<double>& mes_window);

    double wavelength(const std::vector<double>& mes_window);

    double zero_crossings(const std::vector<double>& mes_window);

    double slope_sign_changes(const std::vector<double>& mes_window);

    void auto_regressive_coefficients(std::vector<double>& coeffs, const std::vector<double>& mes_window);

} // namespace meii

#endif // MEII_EMG_FEATURES_HPP