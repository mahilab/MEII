#include <MEII/Classification/EnsembleRTClassifier.hpp>
#include <MEL/Logging/Log.hpp>
#include <numeric>

using namespace mel;

namespace meii {

    EnsembleRTClassifier::EnsembleRTClassifier(std::size_t sample_dimension, Time sample_period, std::size_t classifier_count, Time classification_period, Time feature_period, Time classification_overlap) :
        classifier_count_(classifier_count),
        sample_dim_(sample_dimension),
        Ts_(sample_period),
        classification_period_(classification_period),
        feature_period_(feature_period),
        classification_overlap_(classification_overlap),
        pred_classes_(classifier_count),
        trained_(false)
    {
        construct_classifiers();
    }

    bool EnsembleRTClassifier::update(std::vector<double> sample) {
        if (!trained_)
            return false;
        bool updated = true;
        for (std::size_t i = 0; i < classifier_ptrs_.size(); ++i) {
            if (classifier_ptrs_[i]->update(sample))
                pred_classes_[i] = classifier_ptrs_[i]->get_class();
            else
                updated = false;
        }
        pred_class_ = ensemble_classification_heuristic(pred_classes_);
        return updated;
    }

    bool EnsembleRTClassifier::add_training_data(std::size_t classifier_index, std::size_t class_label, const std::vector<std::vector<double>>& class_data) {
        if (classifier_index >= classifier_count_) {
            LOG(Warning) << "Invalid classifier index provided for EnsembleRTClassifier. Training data not added.";
            return false;
        }
        return classifier_ptrs_[classifier_index]->add_training_data(class_label, class_data);
    }

    bool EnsembleRTClassifier::clear_training_data(std::size_t classifier_index, std::size_t class_label) {
        if (classifier_index >= classifier_count_) {
            LOG(Warning) << "Invalid classifier index provided for EnsembleRTClassifier. Training data not cleared.";
            return false;
        }
        return classifier_ptrs_[classifier_index]->clear_training_data(class_label);
    }

    bool EnsembleRTClassifier::train() {
        trained_ = true;
        for (std::size_t i = 0; i < classifier_count_; ++i) {
            trained_ &= classifier_ptrs_[i]->train();
        }
        return trained_;
    }

    bool EnsembleRTClassifier::set_model(std::size_t classifier_index, const std::vector<double>& w, double w_0) {
        if (classifier_index >= classifier_count_) {
            LOG(Warning) << "Invalid classifier index provided for EnsembleRTClassifier. Model not set.";
            return false;
        }
        return classifier_ptrs_[classifier_index]->set_model(w, w_0);
    }

    bool EnsembleRTClassifier::set_model(std::size_t classifier_index, std::vector<double> w_full) {
        if (classifier_index >= classifier_count_) {
            LOG(Warning) << "Invalid classifier index provided for EnsembleRTClassifier. Model not set.";
            return false;
        }
        return classifier_ptrs_[classifier_index]->set_model(w_full);
    }

    std::size_t EnsembleRTClassifier::get_class() const {
        return pred_class_;
    }

    const std::vector<std::size_t>& EnsembleRTClassifier::get_classes() const {
        return pred_classes_;
    }

    const std::vector<std::vector<double>>& EnsembleRTClassifier::get_class_training_data(std::size_t classifier_index, std::size_t class_label) const {
        if (classifier_index >= classifier_count_) {
            LOG(Warning) << "Invalid classifier index provided for EnsembleRTClassifier. Classifier index set to 0.";
            classifier_index = 0;
        }
        return classifier_ptrs_[classifier_index]->get_class_training_data(class_label);
    }

    std::vector<double> EnsembleRTClassifier::get_model(std::size_t classifier_index) {
        if (classifier_index >= classifier_count_) {
            LOG(Warning) << "Invalid classifier index provided for EnsembleRTClassifier. Classifier index set to 0.";
            classifier_index = 0;
        }
        std::vector<double> w_full = classifier_ptrs_[classifier_index]->get_model();
        return w_full;
    }

    bool EnsembleRTClassifier::is_trained() {
        return trained_;
    }

    void EnsembleRTClassifier::resize(std::size_t classifier_count) {
        classifier_count_ = classifier_count;
        pred_classes_.resize(classifier_count);
        construct_classifiers();
    }

    void EnsembleRTClassifier::construct_classifiers() {
        classifier_ptrs_.resize(classifier_count_);
        classifiers_ = std::vector<RealTimeClassifier>(classifier_count_, RealTimeClassifier(sample_dim_, Ts_, classification_period_, feature_period_, classification_overlap_));
        {
            for (std::size_t i = 0; i < classifier_count_; ++i) {
                classifier_ptrs_[i] = &classifiers_[i];
            }
        }
    }

    std::size_t EnsembleRTClassifier::ensemble_classification_heuristic(const std::vector<std::size_t>& pred_classes) const {

        // majority vote
        if (std::accumulate(pred_classes.begin(), pred_classes.end(), 0) > pred_classes.size() / 2.0) {
            return 1;
        }
        else {
            return 0;
        }
    }

} // namespace meii