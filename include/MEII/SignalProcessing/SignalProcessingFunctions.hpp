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

#ifndef MEII_SIGNAL_PROCESSING_FUNCTIONS_HPP
#define MEII_SIGNAL_PROCESSING_FUNCTIONS_HPP

#include <MEL/Logging/Log.hpp>
#include <MEL/Core/Types.hpp>
#include <vector>

using namespace mel;

namespace meii {

    template <typename T>
    extern std::vector<T> downsample(const std::vector<T>& signal, std::size_t ds_factor) {
        if (signal.empty() || ds_factor == 0 || ds_factor == 1) {
            return signal;
        }

        std::vector<T> ds_signal(((signal.size() - 1) / ds_factor) + 1);
        for (std::size_t i = 0; i < ds_signal.size(); ++i) {
            ds_signal[i] = signal[i * ds_factor];
        }

        return ds_signal;
    }

    template <typename T>
    extern std::vector<std::vector<T>> bin_signal(const std::vector<T>& signal, std::size_t bin_size) {
        std::vector<std::vector<T>> signal_bins = { signal };
        if (signal.size() < bin_size) {
            LOG(Warning) << "Signal was not long enough to split into requested bin size.";
            return signal_bins;
        }
        if (bin_size == 0) {
            return signal_bins;
        }
        std::size_t bin_count = signal.size() / bin_size;
        signal_bins.resize(bin_count);
        std::size_t sample_count = (signal.size() / bin_size) * bin_size;
        for (std::size_t i = 0; i < sample_count; ++i) {
            if (i % bin_size == 0) {
                signal_bins[i / bin_size].resize(bin_size);
            }
            signal_bins[i / bin_size][i % bin_size] = signal[i];
        }
        return signal_bins;
    }

} // namespace meii

#endif // MEII_SIGNAL_PROCESSING_FUNCTIONS_HPP