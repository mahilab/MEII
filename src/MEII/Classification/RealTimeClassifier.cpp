#include <MEII/Classification/RealTimeClassifier.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEII/SignalProcessing/SignalProcessingFunctions.hpp>
#include <MEII/Classification/LinearDiscriminantAnalysis.hpp>
#include <MEL/Logging/Log.hpp>

using namespace mel;

namespace meii {

    RealTimeClassifier::RealTimeClassifier(std::size_t sample_dimension, Time sample_period, Time classification_period, Time feature_period, Time classification_overlap) :
        sample_dim_(sample_dimension),
        Ts_(sample_period),
        classification_window_size_(sample_period < classification_period ? (std::size_t)((unsigned)std::round(classification_period.as_seconds() / sample_period.as_seconds())) : 1),
        feature_window_size_(sample_period < feature_period ? (std::size_t)((unsigned)std::round(feature_period.as_seconds() / sample_period.as_seconds())) : 1),
        pred_counter_(0),
        pred_spacing_(classification_overlap > classification_period ? 1 : (std::size_t)((unsigned)std::round((classification_period.as_seconds() - classification_overlap.as_seconds()) / sample_period.as_seconds()))),
        classification_buffer_(classification_window_size_),
        sample_buffer_(feature_window_size_),
        feature_dim_(get_feature_dim()),
        trained_(false)
    { }

    bool RealTimeClassifier::update(std::vector<double> sample) {
        if (!trained_)
            return false;
        if (sample.size() != sample_dim_) {
            LOG(Error) << "Incorrect size for sample given to RealTimeClassifier::update().";
            return false;
        }

        sample_buffer_.push_back(sample);
        if (!sample_buffer_.full())
            return false;

        x_ = feature_extraction(sample_buffer_.get_vector());
        y_1_ = w_0_;
        for (std::size_t i = 0; i < feature_dim_; ++i) {
            y_1_ += w_[i] * x_[i];
        }
        p_1_ = sigmoid(y_1_);

        classification_buffer_.push_back(p_1_);
        if (++pred_counter_ == classification_window_size_)
            pred_counter_ = 0;
        if (classification_buffer_.full() && (pred_counter_ % pred_spacing_ == 0)) {
            pred_class_ = classification_heuristic(downsample(classification_buffer_.get_vector(), feature_window_size_));
        }
        return true;
    }

    bool RealTimeClassifier::add_training_data(std::size_t class_label, const std::vector<std::vector<double>>& class_data) {
        if (class_label != 0 && class_label != 1) {
            LOG(Warning) << "Invalid class label provided to RealTimeClassifier::add_training_data(). Training data was not added.";
            return false;
        }
        if (class_data.empty()) {
            LOG(Warning) << "Input provided to RealTimeClassifier::add_training_data() was empty. Training data was not added.";
            return false;
        }
        for (std::size_t i = 0; i < class_data.size(); ++i) {
            if (class_data[i].size() != sample_dim_) {
                LOG(Warning) << "Training data provided to RealTimeClassifier::add_training_data() has incorrect size for the sample dimension. Training data was not added.";
                return false;
            }
        }
        switch (class_label) {
        case 0:
            class_0_training_data_.insert(class_0_training_data_.end(), class_data.begin(), class_data.end());
            break;
        case 1:
            class_1_training_data_.insert(class_1_training_data_.end(), class_data.begin(), class_data.end());
            break;
        }
        return true;
    }

    bool RealTimeClassifier::clear_training_data(std::size_t class_label) {
        if (class_label != 0 && class_label != 1) {
            LOG(Warning) << "Invalid class label provided to RealTimeClassifier::add_training_data(). Training data was not cleared.";
            return false;
        }
        switch (class_label) {
        case 0:
            class_0_training_data_.clear();
            break;
        case 1:
            class_1_training_data_.clear();
            break;
        }
        return true;
    }

    bool RealTimeClassifier::train() {
        if (class_0_training_data_.empty() || class_1_training_data_.empty()) {
            LOG(Warning) << "Must collect training data for all classes before calling RealTimeClassifier::train(). Training was aborted.";
            return trained_ = false;
        }

        std::vector<std::vector<std::vector<double>>> binned_data = bin_signal(class_0_training_data_, feature_window_size_);
        class_0_feature_data_.resize(binned_data.size());
        for (std::size_t i = 0; i < binned_data.size(); ++i) {
            class_0_feature_data_[i] = feature_extraction(binned_data[i]);
        }
        binned_data = bin_signal(class_1_training_data_, feature_window_size_);
        class_1_feature_data_.resize(binned_data.size());
        for (std::size_t i = 0; i < binned_data.size(); ++i) {
            class_1_feature_data_[i] = feature_extraction(binned_data[i]);
        }

        if (!bin_linear_discriminant_model(class_0_feature_data_, class_1_feature_data_, w_, w_0_))
            return trained_ = false;

        return trained_ = true;
    }

    bool RealTimeClassifier::set_model(const std::vector<double>& w, double w_0) {
        if (w.size() != feature_dim_) {
            LOG(Warning) << "Size of classification model provided to RealTimeClassifier::set_model() must match the size of the feature dimension. Model not set.";
            return trained_ = false;
        }
        w_ = w;
        w_0_ = w_0;
        return trained_ = true;
    }

    bool RealTimeClassifier::set_model(std::vector<double> w_full) {
        if (w_full.size() != feature_dim_ + 1) {
            LOG(Warning) << "Size of classification model provided to RealTimeClassifier::set_model() must match the size of the feature dimension. Model not set.";
            return trained_ = false;
        }
        w_0_ = w_full.back();
        w_full.pop_back();
        w_ = w_full;
        return trained_ = true;
    }

    std::size_t RealTimeClassifier::get_class() const {
        return pred_class_;
    }

    const std::vector<std::vector<double>>& RealTimeClassifier::get_class_training_data(std::size_t class_label) const {
        if (class_label != 0 && class_label != 1) {
            LOG(Warning) << "Incorrect class label provided to RealTimeClassifier::get_class_training_data(). Returning class 0 training data.";
            return class_0_training_data_;
        }
        if (class_label == 0)
            return class_0_training_data_;
        else
            return class_1_training_data_;
    }

    std::vector<std::vector<double>> RealTimeClassifier::get_all_feature_data() const {
        std::vector<std::vector<double>> all_feature_data;
        std::vector<double> feature_row;
        for (std::size_t i = 0; i < class_0_feature_data_.size(); ++i) {
            feature_row = class_0_feature_data_[i];
            feature_row.push_back(0);
            all_feature_data.push_back(feature_row);
        }
        for (std::size_t i = 0; i < class_1_feature_data_.size(); ++i) {
            feature_row = class_1_feature_data_[i];
            feature_row.push_back(1);
            all_feature_data.push_back(feature_row);
        }
        return all_feature_data;
    }

    std::vector<double> RealTimeClassifier::get_model() const {
        std::vector<double> w_full = w_;
        w_full.push_back(w_0_);
        return w_full;
    }

    bool RealTimeClassifier::is_trained() {
        return trained_;
    }

    std::size_t RealTimeClassifier::get_sample_dim() const {
        return sample_dim_;
    }

    std::size_t RealTimeClassifier::get_feature_dim() const {
        return sample_dim_;
    }

    std::vector<double> RealTimeClassifier::feature_extraction(const std::vector<std::vector<double>>& signal) const {
        if (signal.empty()) {
            LOG(Warning) << "Input given to RealTimeClassifier::feature_extraction() is empty. Returning empty vector.";
            return std::vector<double>();
        }
        return signal[0];
    }

    std::size_t RealTimeClassifier::classification_heuristic(const std::vector<double>& probabilities) const {
        return majority_vote(activation_function(probabilities));
    }

    std::vector<std::size_t> RealTimeClassifier::activation_function(const std::vector<double>& probabilities) const {
        std::vector<std::size_t> labels(probabilities.size(), 0);
        for (std::size_t i = 0; i < probabilities.size(); ++i) {
            if (probabilities[i] > 0.5) {
                labels[i] = 1;
            }
        }
        return labels;
    }

    std::size_t RealTimeClassifier::majority_vote(const std::vector<std::size_t>& labels) const {
        std::vector<std::size_t> votes(2, 0);
        for (std::size_t i = 0; i < labels.size(); ++i) {
            votes[labels[i]]++;
        }
        return std::distance(votes.begin(), std::max_element(votes.begin(), votes.end()));
    }

} // namespace meii