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

#ifndef MEII_ENSEMBLE_RT_CLASSIFIER_HPP
#define MEII_ENSEMBLE_RT_CLASSIFIER_HPP

#include <MEII/Classification/RealTimeClassifier.hpp>
#include <MEL/Core/Time.hpp>
#include <MEL/Utility/RingBuffer.hpp>
#include <vector>
#include <limits>

namespace meii {

    //==============================================================================
    // CLASS DECLARATION
    //==============================================================================

    class EnsembleRTClassifier {

    public:

        /// Constructor
        EnsembleRTClassifier(std::size_t sample_dimension, mel::Time sample_period, std::size_t classifier_count = 0, mel::Time classification_period = mel::Time::Zero, mel::Time feature_period = mel::Time::Zero, mel::Time classification_overlap = mel::microseconds(std::numeric_limits<mel::int64>::max()));

        /// Update called every sample period, taking in new input sample. Must have already done training.
        bool update(std::vector<double> sample);

        /// Provide more training data for binary classification, adding it to existing data. Must be done before predictions can begin. 
        bool add_training_data(std::size_t classifier_index, std::size_t class_label, const std::vector<std::vector<double>>& class_data);

        /// Clear the training data for a specific class. 
        bool clear_training_data(std::size_t classifier_index, std::size_t class_label);

        /// Use training data to compute linear classification model. Must have set training data for every class. Must be done before predictions can begin.
        bool train();

        /// Provide the linear classification model without training data.
        bool set_model(std::size_t classifier_index, const std::vector<double>& w, double w_0);
        bool set_model(std::size_t classifier_index, std::vector<double> w_full);

        /// Get latest prediction of the class label since calling update.
        std::size_t get_class() const;

        /// Get latest vector of predictions of the class label by each classifier since calling update.
        const std::vector<std::size_t>& get_classes() const;

        /// Get the current training data that has been added for a specific class.
        const std::vector<std::vector<double>>& get_class_training_data(std::size_t classifier_index, std::size_t class_label) const;

        /// Get the vector of w with w_0 on the end used for computing predictions
        std::vector<double> get_model(std::size_t classifier_index);

        /// Return whether or not the classifier has been trained.
        bool is_trained();

        /// Change the number of EmgActiveClassifiers that compose the ensemble classifier, reinitializing the entire vector
        void resize(std::size_t classifier_count);

		bool save(const std::string &filename = "ensemble_real_time_classifier", const std::string& directory = ".", bool timestamp = true);

		bool load(const std::string &filename = "", const std::string& directory = ".");

		std::vector<mel::Table> make_datalog() const;

		bool read_datalog(const std::vector<mel::Table> &tables);

    protected:

        /// Constructor for classifier_ptrs_ and classifiers_
        virtual void construct_classifiers();

        /// Method of converting the predictions of the individual classifiers into a single prediction.
        virtual std::size_t ensemble_classification_heuristic(const std::vector<std::size_t>& pred_classes) const;


		

    protected:

        std::size_t classifier_count_; ///< number of binary classifiers feeding into ensemble classifier
        std::size_t sample_dim_; ///< size of each sample vector
        mel::Time Ts_; ///< sample period
        mel::Time classification_period_; ///< classification period
        mel::Time feature_period_; ///< feature period
        mel::Time classification_overlap_; ///< classification overlap

        std::vector<RealTimeClassifier*> classifier_ptrs_; ///< vector of pointers to binary classifiers feeding into ensemble classifier
        std::vector<RealTimeClassifier> classifiers_; ///< vector of binary classifiers feeding into ensemble classifier

        std::vector<std::size_t> pred_classes_; ///< vector of classification outputs of individual classifiers

        std::size_t pred_class_; ///< ensemble class prediction

        bool trained_; ///< whether or not a classifier has been trained

    };

} // namespace meii

#endif // MEII_ENSEMBLE_RT_CLASSIFIER_HPP