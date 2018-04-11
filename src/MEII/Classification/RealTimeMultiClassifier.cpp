#include <MEII/Classification/RealTimeMultiClassifier.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEII/SignalProcessing/SignalProcessingFunctions.hpp>
#include <MEII/Classification/LinearDiscriminantAnalysis.hpp>
#include <MEL/Logging/Log.hpp>

#include <MEL/Logging/DataLogger.hpp>

using namespace mel;

namespace meii {

    RealTimeMultiClassifier::RealTimeMultiClassifier(std::size_t class_count, std::size_t sample_dimension, Time sample_period, Time classification_period, Time feature_period, Time classification_overlap) :
        class_count_(class_count < 2 ? 2 : class_count),
        sample_dim_(sample_dimension),
        Ts_(sample_period),
        classification_window_size_(sample_period < classification_period ? (std::size_t)((unsigned)std::round(classification_period.as_seconds() / sample_period.as_seconds())) : 1),
        feature_window_size_(sample_period < feature_period ? (std::size_t)((unsigned)std::round(feature_period.as_seconds() / sample_period.as_seconds())) : 1),
        pred_counter_(0),
        pred_spacing_(classification_overlap > classification_period ? 1 : (std::size_t)((unsigned)std::round((classification_period.as_seconds() - classification_overlap.as_seconds()) / sample_period.as_seconds()))),
        classification_buffer_(classification_window_size_),
        sample_buffer_(feature_window_size_),
        training_data_(class_count),
        feature_data_(class_count),
        feature_dim_(get_feature_dim()),
        w_(class_count),
        w_0_(class_count),
        y_(class_count),
        p_(class_count),
        trained_(false)
    {}

    bool RealTimeMultiClassifier::update(std::vector<double> sample) {
        if (!trained_)
            return false;
        if (sample.size() != sample_dim_) {
            LOG(Error) << "Incorrect size for sample given to update() in RealTimeMultiClassifier";
            return false;
        }

        sample_buffer_.push_back(sample);
        if (!sample_buffer_.full())
            return false;

        x_ = feature_extraction(sample_buffer_.get_vector());

        y_ = w_0_;
        for (std::size_t i = 0; i < class_count_; ++i) {
            for (std::size_t j = 0; j < feature_dim_; ++j) {
                y_[i] += w_[i][j] * x_[j];
            }
        }

        for (std::size_t i = 0; i < class_count_; ++i) {
            p_[i] = softmax(y_, i);
        }
        classification_buffer_.push_back(p_);
        if (++pred_counter_ == classification_window_size_)
            pred_counter_ = 0;
        if (classification_buffer_.full() && (pred_counter_ % pred_spacing_ == 0)) {
            pred_class_ = classification_heuristic(downsample(classification_buffer_.get_vector(), feature_window_size_));
        }
        return true;
    }

    bool RealTimeMultiClassifier::add_training_data(std::size_t class_label, const std::vector<std::vector<double>>& class_data) {
        if (class_label >= class_count_) {
            LOG(Warning) << "Invalid class label provided. Training data was not added.";
            return false;
        }
        training_data_[class_label].insert(training_data_[class_label].end(), class_data.begin(), class_data.end());
        return true;
    }

    bool RealTimeMultiClassifier::clear_training_data(std::size_t class_label) {
        if (class_label >= class_count_) {
            LOG(Warning) << "Invalid class label provided. Training data was not cleared.";
            return false;
        }
        training_data_[class_label].clear();
        return true;
    }

    bool RealTimeMultiClassifier::train() {
        for (std::size_t i = 0; i < class_count_; ++i) {
            if (training_data_[i].empty()) {
                LOG(Warning) << "Must collect training data for all classes before calling train(). Training was aborted.";
                return trained_ = false;
            }
        }

        std::vector<std::vector<std::vector<double>>> binned_data;
        for (std::size_t i = 0; i < class_count_; ++i) {
            binned_data = bin_signal(training_data_[i], feature_window_size_);
            feature_data_[i].resize(binned_data.size());
            for (std::size_t j = 0; j < binned_data.size(); ++j) {
                feature_data_[i][j] = feature_extraction(binned_data[j]);
            }
        }
        if (!multi_linear_discriminant_model(feature_data_, w_, w_0_, 1.0))
            return trained_ = false;

        return trained_ = true;
    }

    bool RealTimeMultiClassifier::set_model(const std::vector<std::vector<double>>& w, const std::vector<double>& w_0) {
        if (w.size() != class_count_) {
            LOG(Warning) << "Classification model size must match the number of classes. Model not set.";
            return trained_ = false;
        }
        if (w_0.size() != class_count_) {
            LOG(Warning) << "Classification model intercept size must match the number of classes. Model not set.";
            return trained_ = false;
        }
        for (std::size_t i = 0; i < class_count_; ++i) {
            if (w[i].size() != feature_dim_) {
                LOG(Warning) << "Classification model inner dimension size must match the size of the sample dimension. Model not set.";
                return trained_ = false;
            }
        }
        w_ = w;
        w_0_ = w_0;
        return trained_ = true;
    }

    std::size_t RealTimeMultiClassifier::get_class() const {
        if (!trained_)
            return class_count_;
        return pred_class_;
    }

    void RealTimeMultiClassifier::get_model(std::vector<std::vector<double>>& w, std::vector<double>& w_0) {
        w = w_;
        w_0 = w_0_;
    }

    const std::vector<std::vector<double>>& RealTimeMultiClassifier::get_class_training_data(std::size_t class_label) const {
        if (class_label >= class_count_) {
            LOG(Warning) << "Invalid class label provided to RealTimeMultiClassifier::get_class_training_data(). Class label set to 0.";
            class_label = 0;
        }
        return training_data_[class_label];
    }

    const std::vector<std::vector<double>>& RealTimeMultiClassifier::get_class_feature_data(std::size_t class_label) const {
        if (class_label >= class_count_) {
            LOG(Warning) << "Invalid class label provided to RealTimeMultiClassifier::get_class_feature_data(). Class label set to 0.";
            class_label = 0;
        }
        return feature_data_[class_label];
    }

    bool RealTimeMultiClassifier::is_trained() {
        return trained_;
    }

    std::size_t RealTimeMultiClassifier::get_sample_dim() const {
        return sample_dim_;
    }

    std::size_t RealTimeMultiClassifier::get_feature_dim() const {
        return sample_dim_;
    }

    std::vector<double> RealTimeMultiClassifier::feature_extraction(const std::vector<std::vector<double>>& signal) const {
        if (signal.empty()) {
            LOG(Warning) << "Input given to RealTimeMultiClassifier::feature_extraction() is empty. Returning empty vector.";
            return std::vector<double>();
        }
        return signal[0];
    }

    std::size_t RealTimeMultiClassifier::classification_heuristic(const std::vector<std::vector<double>>& probabilities) const {
        return majority_vote(activation_function(probabilities));
    }

    std::vector<std::size_t> RealTimeMultiClassifier::activation_function(const std::vector<std::vector<double>>& probabilities) const {
        std::vector<std::size_t> labels(probabilities.size());
        for (std::size_t i = 0; i < probabilities.size(); ++i) {
            labels[i] = std::distance(probabilities[i].begin(), std::max_element(probabilities[i].begin(), probabilities[i].end()));
        }
        return labels;
    }

    std::size_t RealTimeMultiClassifier::majority_vote(const std::vector<std::size_t>& labels) const {
        std::vector<std::size_t> votes(class_count_, 0);
        for (std::size_t i = 0; i < labels.size(); ++i) {
            votes[labels[i]]++;
        }
        return std::distance(votes.begin(), std::max_element(votes.begin(), votes.end()));
    }

} // namespace meii