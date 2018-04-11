#include <MEII/Classification/EmgActiveClassifier.hpp>
#include <MEL/Math/Functions.hpp>

using namespace mel;

namespace meii {

    EmgActiveClassifier::EmgActiveClassifier(std::size_t sample_dimension, Time sample_period, Time classification_period, Time feature_period, Time classification_overlap) :
        RealTimeClassifier(sample_dimension, sample_period, classification_period, feature_period, classification_overlap)
    {}

    std::size_t EmgActiveClassifier::get_feature_dim() const {
        return get_sample_dim();
    }

    std::vector<double> EmgActiveClassifier::feature_extraction(const std::vector<std::vector<double>>& signal) const {
        std::vector<double> feature_vec(signal[0].size(), 0.0);
        for (std::size_t i = 0; i < signal.size(); ++i) {
            for (std::size_t j = 0; j < feature_vec.size(); ++j) {
                feature_vec[j] += signal[i][j];
            }
        }
        double N = (double)signal.size();
        for (std::size_t j = 0; j < feature_vec.size(); ++j) {
            feature_vec[j] /= N;
        }
        return feature_vec;
    }

} // namespace meii