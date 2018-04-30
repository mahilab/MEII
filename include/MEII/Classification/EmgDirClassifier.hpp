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

#ifndef MEII_EMG_DIR_CLASSIFIER_HPP
#define MEII_EMG_DIR_CLASSIFIER_HPP

#include <MEII/Classification/RealTimeMultiClassifier.hpp>
#include <MEL/Core/Time.hpp>
#include <vector>
#include <limits>

namespace meii {
    //==============================================================================
    // CLASS DECLARATION
    //==============================================================================

    class EmgDirClassifier : public RealTimeMultiClassifier {

    public:

        /// Preferred constructor
        EmgDirClassifier(std::size_t class_count, std::size_t sample_dimension, mel::Time sample_period,
            bool RMS = true, bool MAV = true, bool WL = true, bool ZC = true, bool SSC = true, bool AR1 = true, bool AR2 = true, bool AR3 = true, bool AR4 = true,
            mel::Time classification_period = mel::milliseconds(1), mel::Time feature_period = mel::milliseconds(200), mel::Time classification_overlap = mel::Time::Zero);

        /// Return the size of the feature space
        virtual std::size_t get_feature_dim() const override;

    protected:

        /// Convert window of observations into feature vector for classification.
        virtual std::vector<double> feature_extraction(const std::vector<std::vector<double>>& signal) const override;

        std::vector<double> normalize_by_mean(const std::vector<double>& input) const;

    private:

        bool RMS_;
        bool MAV_;
        bool WL_;
        bool ZC_;
        bool SSC_;
        bool AR1_;
        bool AR2_;
        bool AR3_;
        bool AR4_;
        std::size_t feature_count_; ///< number of features calculated per MES

    };

} // namespace meii

#endif // MEII_EMG_DIR_CLASSIFIER_HPP