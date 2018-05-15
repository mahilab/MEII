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

#ifndef MEII_EMG_DATA_CAPTURE_HPP
#define MEII_EMG_DATA_CAPTURE_HPP

#include <vector>

namespace meii {

    /// Return a window of output_signal (or ref_signal if output_signal is empty) of size window_size that is centered on the maximum value of ref_signal
    extern std::vector<double> find_max_window(const std::vector<double>& ref_signal, std::size_t window_size = 1, const std::vector<double>& output_signal = std::vector<double>());

    /// Return a window of output_signal (or ref_signal if output_signal is empty) of size window_size that is centered on the maximum value of the sum of the vectors in ref_signal
    extern std::vector<std::vector<double>> find_sum_max_window(const std::vector<std::vector<double>> &ref_signal, std::size_t window_size = 1, const std::vector<std::vector<double>> &output_signal = std::vector<std::vector<double>>());

	extern std::vector<double> max_sample_per_channel(const std::vector<std::vector<double>> signal);

} // namespace meii

#endif // MEII_EMG_DATA_CAPTURE_HPP