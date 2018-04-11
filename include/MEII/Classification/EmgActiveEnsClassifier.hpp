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

#ifndef MEII_EMG_ACTIVE_ENS_CLASSIFIER_HPP
#define MEII_EMG_ACTIVE_ENS_CLASSIFIER_HPP

#include <MEII/Classification/EmgActiveClassifier.hpp>
#include <MEII/Classification/EnsembleRTClassifier.hpp>
#include <MEL/Core/Time.hpp>
#include <vector>
#include <limits>

namespace meii {

    //==============================================================================
    // CLASS DECLARATION
    //==============================================================================

    class EmgActiveEnsClassifier : public EnsembleRTClassifier {

    public:

        /// Constructor
        EmgActiveEnsClassifier(std::size_t sample_dimension, mel::Time sample_period, std::size_t classifier_count = 0, mel::Time classification_period = mel::milliseconds(200), mel::Time feature_period = mel::milliseconds(10), mel::Time classification_overlap = mel::milliseconds(199));


    protected:

        /// Constructor for classifier_ptrs_ and classifiers_
        virtual void construct_classifiers() override;

        /// Method of converting the predictions of the individual classifiers into a single prediction.
        virtual std::size_t ensemble_classification_heuristic(const std::vector<std::size_t>& pred_classes) const override;

    private:

        std::vector<EmgActiveClassifier> emg_active_classifiers_; ///< vector of binary classifiers feeding into ensemble classifier
    };

} // namespace meii

#endif // MEII_EMG_ACTIVE_ENS_CLASSIFIER_HPP