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

#ifndef MEII_REAL_TIME_MULTI_CLASSIFIER_HPP
#define MEII_REAL_TIME_MULTI_CLASSIFIER_HPP

#include <MEL/Core/Time.hpp>
#include <MEL/Utility/RingBuffer.hpp>
#include <MEL/Logging/Table.hpp>
#include <vector>
#include <limits>

namespace meii {

    //==============================================================================
    // CLASS DECLARATION
    //==============================================================================

    class RealTimeMultiClassifier {

    public:

		/// Preferred constructor
        RealTimeMultiClassifier(std::size_t class_count, std::size_t sample_dimension, mel::Time sample_period, mel::Time classification_period = mel::Time::Zero, mel::Time feature_period = mel::Time::Zero, mel::Time classification_overlap = mel::microseconds(std::numeric_limits<mel::int64>::max()));

        /// Update called every sample period, taking in new input sample. Must have already done training.
        bool update(std::vector<double> sample);

        /// Provide more training data for classification, adding it to existing data. Must be done before predictions can begin. 
        bool add_training_data(std::size_t class_label, const std::vector<std::vector<double>>& class_data);

        /// Clear the training data for a specific class. 
        bool clear_training_data(std::size_t class_label);

        /// Use training data to compute linear classification model. Must have set training data for every class. Must be done before predictions can begin.
        bool train();		

        /// Provide the linear classification model without training data.
        bool set_model(const std::vector<std::vector<double>>& w, const std::vector<double>& w_0);

        /// Get latest prediction of the class label since calling update. If classifier not trained, return invalid label = class_count.
        std::size_t get_class() const;

		/// Get latest features computed from the sample buffer since calling update.
		const std::vector<double> &get_features() const;

		/// Get latest model output computed from the sample buffer since calling update.
		const std::vector<double> &get_model_output() const;

		/// Get latest class posterior probabilities computed from the sample buffer since calling update.
		const std::vector<double> &get_class_posteriors() const;

        /// Get the vector of w with w_0 on the end used for computing predictions.
        void get_model(std::vector<std::vector<double>>& w, std::vector<double>& w_0);

		/// Compute features from the existing training data and store them.
		bool compute_training_features();

        /// Get the current training data that has been added for a specific class.
        std::vector<std::vector<double>> get_class_training_data(std::size_t class_label) const;

        /// Get the current feature data that has been added for a specific class, with class labels at the end of each observation vector.
        std::vector<std::vector<double>> get_class_feature_data(std::size_t class_label) const;

        /// Return whether or not the classifier has been trained.
        bool is_trained();

        /// Return the size of the sample space
        std::size_t get_sample_dim() const;

        /// Return the size of the feature space
        virtual std::size_t get_feature_dim() const;

		/// Return the number of classes
		std::size_t get_class_count() const;

		/// Sets the number of classes. Clears stored training data, feature data, and classification model if size does not match.
		void set_class_count(std::size_t class_count);

		/// Clears data stored in the classification buffer and sample buffer.
		void clear_buffers();

		bool save(const std::string &filename = "real_time_multi_classifier", const std::string& directory = ".", bool timestamp = true);

		bool load(const std::string &filename = "", const std::string& directory = ".");

		std::vector<mel::Table> make_datalog() const;

		bool read_datalog(const std::vector<mel::Table> &tables);

		/// uses 1-based indexing of class labels
		bool export_training_features(const std::string &filename = "real_time_multi_classifier_training_features", const std::string& directory = ".", bool timestamp = true);

    protected:

        /// Convert window of observations into feature vector for classification.
        virtual std::vector<double> feature_extraction(const std::vector<std::vector<double>>& signal) const;

        /// Method of converting all predictions in a classification window into a single prediction.
        virtual std::size_t classification_heuristic(const std::vector<std::vector<double>>& probabilities) const;

        /// Convert probability vectors into class labels
        std::vector<std::size_t> activation_function(const std::vector<std::vector<double>>& probabilities) const;

        /// Apply the majority vote rule to a vector of class labels
        std::size_t majority_vote(const std::vector<std::size_t>& labels) const;

    private:

        std::size_t class_count_; ///< number of classes to choose from, must be at least 2
        std::size_t sample_dim_; ///< size of sample vector

        mel::Time Ts_; ///< sample period    
        std::size_t classification_window_size_; ///< number of samples on which to perform classification
        std::size_t feature_window_size_; ///< number of samples on which to extract features

        std::size_t pred_counter_; ///< keeps track of relative spacing of predictions
        std::size_t pred_spacing_; ///< controls relative spacing of predictions

        mel::RingBuffer<std::vector<double>> classification_buffer_; ///< contains most resent posterior probabilities 
        mel::RingBuffer<std::vector<double>> sample_buffer_; ///< contains most recent samples to be converted to feature vector for classification   

        std::vector<std::vector<std::vector<double>>> training_data_; ///< training data for all classes
        std::vector<std::vector<std::vector<double>>> feature_data_; ///< feature data computed from training data for all classes

        std::vector<std::vector<double>> w_; ///< classification weighting coefficients
        std::vector<double> w_0_; ///< classification intercept
        std::vector<double> phi_; ///< classification input
        std::vector<double> y_; ///< classification output
        std::vector<double> p_; ///< class posterior probabilities
        std::size_t pred_class_; ///< class prediction

        bool trained_; ///< whether or not a classifier has been trained

    };

} // namespace meii

#endif // MEII_REAL_TIME_MULTI_CLASSIFIER_HPP