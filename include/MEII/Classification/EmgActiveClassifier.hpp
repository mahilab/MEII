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

#ifndef MEII_EMG_ACTIVE_CLASSIFIER_HPP
#define MEII_EMG_ACTIVE_CLASSIFIER_HPP

#include <MEII/Classification/RealTimeClassifier.hpp>
#include <MEL/Core/Time.hpp>
#include <vector>
#include <limits>

namespace meii {

    //==============================================================================
    // CLASS DECLARATION
    //==============================================================================

    class EmgActiveClassifier : public RealTimeClassifier {

    public:

        /// Constructor
        EmgActiveClassifier(std::size_t sample_dimension, mel::Time sample_period, mel::Time classification_period = mel::milliseconds(200), mel::Time feature_period = mel::milliseconds(10), mel::Time classification_overlap = mel::milliseconds(199));

        /// Return the size of the feature space
        virtual std::size_t get_feature_dim() const override;

    protected:

        /// Takes the mean of each bin, returning a vector of the same size as individual observations.
        virtual std::vector<double> feature_extraction(const std::vector<std::vector<double>>& signal) const override;

    };

} // namespace meii

#endif // MEII_EMG_ACTIVE_CLASSIFIER_HPP