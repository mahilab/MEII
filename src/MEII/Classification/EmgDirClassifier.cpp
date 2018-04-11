#include <MEII/Classification/EmgDirClassifier.hpp>
#include <MEII/EMG/EmgFeatures.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Math/Functions.hpp>

using namespace mel;

namespace meii {

    EmgDirClassifier::EmgDirClassifier(std::size_t class_count, std::size_t sample_dimension, Time sample_period,
        bool RMS, bool MAV, bool WL, bool ZC, bool SSC, bool AR1, bool AR2, bool AR3, bool AR4,
        Time classification_period, Time feature_period, Time classification_overlap) :
        RealTimeMultiClassifier(class_count, sample_dimension, sample_period, classification_period, feature_period, classification_overlap),
        RMS_(RMS),
        MAV_(MAV),
        WL_(WL),
        ZC_(ZC),
        SSC_(SSC),
        AR1_(AR1),
        AR2_(AR2),
        AR3_(AR3),
        AR4_(AR4)
    {
        feature_count_ = 0;
        if (RMS)
            feature_count_++;
        if (MAV)
            feature_count_++;
        if (WL)
            feature_count_++;
        if (ZC)
            feature_count_++;
        if (SSC)
            feature_count_++;
        if (AR1)
            feature_count_++;
        if (AR2)
            feature_count_++;
        if (AR3)
            feature_count_++;
        if (AR4)
            feature_count_++;
    }

    std::size_t EmgDirClassifier::get_feature_dim() const {
        return get_sample_dim() * feature_count_;
    }

    std::vector<double> EmgDirClassifier::feature_extraction(const std::vector<std::vector<double>>& signal) const {
        if (signal.empty()) {
            LOG(Warning) << "Input given to EmgDirClassifier::feature_extraction() was empty. Returning empty vector.";
            return std::vector<double>();
        }
        for (std::size_t i = 0; i < signal.size(); ++i) {
            if (signal[i].empty()) {
                LOG(Warning) << "Input given to EmgDirClassifier::feature_extraction() was empty. Returning empty vector.";
                return std::vector<double>();
            }
        }

        std::size_t mes_size = signal.size();
        std::size_t mes_count = signal[0].size();
        std::vector<double> channel(mes_size);
        std::vector<double> rms(mes_count);
        std::vector<double> mav(mes_count);
        std::vector<double> wl(mes_count);
        std::vector<double> zc(mes_count);
        std::vector<double> ssc(mes_count);
        std::vector<double> ar(4);
        std::vector<double> ar1(mes_count);
        std::vector<double> ar2(mes_count);
        std::vector<double> ar3(mes_count);
        std::vector<double> ar4(mes_count);
        for (std::size_t i = 0; i < mes_count; ++i) {
            for (std::size_t j = 0; j < mes_size; ++j) {
                if (signal[j].size() != mes_count) {
                    LOG(Warning) << "Input signal given to EmgDirClassifier::feature_extraction() contains samples not all of the same size. Returning empty vector.";
                    return std::vector<double>();
                }
            }
            for (std::size_t j = 0; j < mes_size; ++j) {
                channel[j] = signal[j][i];
            }
            if (RMS_)
                rms[i] = mean_rms(channel);
            if (MAV_)
                mav[i] = mean_absolute_value(channel);
            if (WL_)
                wl[i] = wavelength(channel);
            if (ZC_)
                zc[i] = zero_crossings(channel);
            if (SSC_)
                ssc[i] = slope_sign_changes(channel);
            if (AR1_ || AR2_ || AR3_ || AR4_)
                auto_regressive_coefficients(ar, channel);
            if (AR1_)
                ar1[i] = ar[0];
            if (AR2_)
                ar2[i] = ar[1];
            if (AR3_)
                ar3[i] = ar[2];
            if (AR4_)
                ar4[i] = ar[3];
        }

        std::vector<double> features;
        if (RMS_) {       
            if (mes_count > 1) {
                rms = normalize_by_mean(rms);
            }
            features.insert(features.end(), rms.begin(), rms.end());
        }
        if (MAV_) {           
            if (mes_count > 1) {
                mav = normalize_by_mean(mav);
            }
            features.insert(features.end(), mav.begin(), mav.end());
        }
        if (WL_) {
            if (mes_count > 1) {
                wl = normalize_by_mean(wl);
            }
            features.insert(features.end(), wl.begin(), wl.end());
        }
        if (ZC_) {            
            if (mes_count > 1) {
                zc = normalize_by_mean(zc);
            }
            features.insert(features.end(), zc.begin(), zc.end());
        }
        if (SSC_) {
            if (mes_count > 1) {
                ssc = normalize_by_mean(ssc);
            }
            features.insert(features.end(), ssc.begin(), ssc.end());
        }
        if (AR1_) {
            features.insert(features.end(), ar1.begin(), ar1.end());
        }
        if (AR2_) {
            features.insert(features.end(), ar2.begin(), ar2.end());
        }
        if (AR3_) {
            features.insert(features.end(), ar3.begin(), ar3.end());
        }
        if (AR4_) {
            features.insert(features.end(), ar4.begin(), ar4.end());
        }

        return features;
    }

    std::vector<double> EmgDirClassifier::normalize_by_mean(const std::vector<double>& input) const {
        if (input.empty())
            return std::vector<double>();
        double input_mean = mean(input);
        if (input_mean == 0.0)
            input_mean = 1.0;
        std::vector<double> output = input;
        for (std::size_t i = 0; i < output.size(); ++i) {
            output[i] /= input_mean;
        }
        return output;
    }

} // namespace meii