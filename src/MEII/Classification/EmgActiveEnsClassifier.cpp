#include <MEII/Classification/EmgActiveEnsClassifier.hpp>
#include <algorithm>

using namespace mel;

namespace meii {

    EmgActiveEnsClassifier::EmgActiveEnsClassifier(std::size_t sample_dimension, Time sample_period, std::size_t classifier_count, Time classification_period, Time feature_period, Time classification_overlap) :
        EnsembleRTClassifier(sample_dimension, sample_period, classifier_count, classification_period, feature_period, classification_overlap)
    {}

    void EmgActiveEnsClassifier::construct_classifiers() {
        classifier_ptrs_.resize(classifier_count_);
        emg_active_classifiers_ = std::vector<EmgActiveClassifier>(classifier_count_, EmgActiveClassifier(sample_dim_, Ts_, classification_period_, feature_period_, classification_overlap_));
        {
            for (std::size_t i = 0; i < classifier_count_; ++i) {
                classifier_ptrs_[i] = &emg_active_classifiers_[i];
            }
        }
    }

    std::size_t EmgActiveEnsClassifier::ensemble_classification_heuristic(const std::vector<std::size_t>& pred_classes) const {
        if (std::any_of(pred_classes.begin(), pred_classes.end(), [](std::size_t c) {return c == 1; }))
            return 1;
        else
            return 0;
    }

}