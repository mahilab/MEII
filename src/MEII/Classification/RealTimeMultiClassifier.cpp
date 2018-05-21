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
        training_data_(class_count_),
        feature_data_(class_count_),
        w_(class_count_),
        w_0_(class_count_),
        y_(class_count_),
        p_(class_count_),
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
		if (!sample_buffer_.full()) {
			return false;
		}

        phi_ = feature_extraction(sample_buffer_.get_vector());

        y_ = w_0_;
        for (std::size_t i = 0; i < class_count_; ++i) {
            for (std::size_t j = 0; j < get_feature_dim(); ++j) {
                y_[i] += w_[i][j] * phi_[j];
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
		LOG(Verbose) << "Added training data to RealTimeMultiClassifier for class " << class_label << ". This class now has " << training_data_[class_label].size() << " samples of training data.";
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
		if (!compute_training_features()) {
			return trained_ = false;
		}

		if (!multi_linear_discriminant_model(feature_data_, w_, w_0_, 0.1)) {
			return trained_ = false;
		}

        return trained_ = true;
    }

	bool RealTimeMultiClassifier::compute_training_features() {
		bool all_classes_have_features = true;
		std::vector<std::vector<std::vector<double>>> binned_data;
		for (std::size_t i = 0; i < class_count_; ++i) {
			if (training_data_[i].size() < feature_window_size_) {
				LOG(Warning) << "Not enough training data was provided for class " << i << " when calling RealTimeMultiClassifier::compute_features().";
				all_classes_have_features = false;
			}
			else {
				binned_data = bin_signal(training_data_[i], feature_window_size_);
				feature_data_[i].resize(binned_data.size());
				for (std::size_t j = 0; j < binned_data.size(); ++j) {
					feature_data_[i][j] = feature_extraction(binned_data[j]);
				}
			}
		}
		return all_classes_have_features;
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
            if (w[i].size() != get_feature_dim()) {
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

	const std::vector<double> &RealTimeMultiClassifier::get_features() const {
		if (!trained_) {
			LOG(Warning) << "Attempting to access RealTimeMultiClassifier features computed for prediction before the classifier has been trained.";
		}
		return phi_;
	}

	const std::vector<double> &RealTimeMultiClassifier::get_model_output() const {
		if (!trained_) {
			LOG(Warning) << "Attempting to access RealTimeMultiClassifier model output computed for prediction before the classifier has been trained.";
		}
		return y_;
	}

	const std::vector<double> &RealTimeMultiClassifier::get_class_posteriors() const {
		if (!trained_) {
			LOG(Warning) << "Attempting to access RealTimeMultiClassifier class posteriors computed for prediction before the classifier has been trained.";
		}
		return p_;
	}

    void RealTimeMultiClassifier::get_model(std::vector<std::vector<double>>& w, std::vector<double>& w_0) {
        w = w_;
        w_0 = w_0_;
    }

    std::vector<std::vector<double>> RealTimeMultiClassifier::get_class_training_data(std::size_t class_label) const {
		if (class_label >= class_count_) {
			LOG(Warning) << "Invalid class label provided to RealTimeMultiClassifier::get_class_training_data(). Returning empty vector.";
			return std::vector<std::vector<double>>();
		}
		if (training_data_.empty() || training_data_[class_label].empty()) {
			LOG(Warning) << "No feature data found during call to RealTimeMultiClassifier::get_class_training_data(). Returning empty vector.";
			return std::vector<std::vector<double>>();
		}

		std::vector<std::vector<double>> class_training_data = training_data_[class_label];
		for (std::size_t i = 0; i < class_training_data.size(); ++i) {
			class_training_data[i].push_back(class_label+1);
		}
		return class_training_data;
    }

    std::vector<std::vector<double>> RealTimeMultiClassifier::get_class_feature_data(std::size_t class_label) const {
        if (class_label >= class_count_) {
            LOG(Warning) << "Invalid class label provided to RealTimeMultiClassifier::get_class_feature_data(). Returning empty vector.";
            return std::vector<std::vector<double>>();
        }
		if (feature_data_.empty() || feature_data_[class_label].empty()) {
			LOG(Warning) << "No feature data found during call to RealTimeMultiClassifier::get_class_feature_data(). Returning empty vector.";
			return std::vector<std::vector<double>>();
		}

		std::vector<std::vector<double>> class_feature_data = feature_data_[class_label];
		for (std::size_t i = 0; i < class_feature_data.size(); ++i) {
			class_feature_data[i].push_back(class_label+1);
		}
        return class_feature_data;
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

	std::size_t RealTimeMultiClassifier::get_class_count() const {
		return class_count_;
	}

	void RealTimeMultiClassifier::set_class_count(std::size_t class_count) {
		class_count = class_count < 2 ? 2 : class_count;
		if (class_count_ != class_count) {
			class_count_ = class_count;
			training_data_.resize(class_count);
			feature_data_.resize(class_count);
			w_.resize(class_count);
			w_0_.resize(class_count);
			y_.resize(class_count);
			p_.resize(class_count);
			trained_ = false;
		}
	}

	void RealTimeMultiClassifier::clear_buffers() {
		classification_buffer_.clear();
		sample_buffer_.clear();
	}

	bool RealTimeMultiClassifier::save(const std::string &filename, const std::string& directory, bool timestamp) {
		return DataLogger::write_to_csv(make_datalog(), filename, directory, timestamp);
	}

	bool RealTimeMultiClassifier::load(const std::string &filename, const std::string& directory) {
		std::vector<Table> tables;
		if (DataLogger::read_from_csv(tables, filename, directory)) {
			return read_datalog(tables);
		}
		else {
			return false;
		}
	}

	std::vector<Table> RealTimeMultiClassifier::make_datalog() const {
		std::vector<Table> tables;

		Table params("Parameters", { "class_count", "sample_dim", "Ts", "classification_window_size", "feature_window_size", "pred_spacing", "feature_dim", "trained" });
		std::vector<double> params_values;
		params_values.push_back((double)class_count_);
		params_values.push_back((double)sample_dim_);
		params_values.push_back(Ts_.as_seconds());
		params_values.push_back((double)classification_window_size_);
		params_values.push_back((double)feature_window_size_);
		params_values.push_back((double)pred_spacing_);
		params_values.push_back((double)get_feature_dim());
		params_values.push_back((double)trained_);
		params.set_values({ params_values });
		tables.push_back(params);

		for (std::size_t i = 0; i < class_count_; ++i) {
			Table table("Class" + stringify(i) + "TrainingData");
			for (std::size_t j = 0; j < sample_dim_; ++j) {
				table.push_back_col("x_" + stringify(j));
			}
			if (!training_data_[i].empty()) {
				table.set_values(training_data_[i]);
			}
			tables.push_back(table);
		}

		for (std::size_t i = 0; i < class_count_; ++i) {
			Table table("Class" + stringify(i) + "FeatureData");
			for (std::size_t j = 0; j < get_feature_dim(); ++j) {
				table.push_back_col("phi_" + stringify(j));
			}
			if (!feature_data_.empty()) {
				table.set_values(feature_data_[i]);
			}
			tables.push_back(table);
		}

		Table model("Model");
		for (std::size_t j = 0; j < get_feature_dim(); ++j) {
			model.push_back_col("w_" + stringify(j));
		}
		if (trained_) {
			for (std::size_t i = 0; i < class_count_; ++i) {
				model.push_back_row(w_[i]);
			}
			model.push_back_col("intercept", w_0_);
		}
		else {
			model.push_back_col("intercept");
		}
		tables.push_back(model);

		return tables;
	}

	bool RealTimeMultiClassifier::read_datalog(const std::vector<mel::Table> &tables) {
		if (tables.empty()) {
			LOG(Warning) << "Contents of file given to RealTimeMultiClassifier::read_datalog() are invalid. Incorrect number of Tables.";
			return false;
		}

		if (tables[0].name().compare("Parameters") == 0) {
			class_count_ = (std::size_t)tables[0](0, 0);
			sample_dim_ = (std::size_t)tables[0](0, 1);
			Ts_ = seconds(tables[0](0, 2));
			classification_window_size_ = (std::size_t)tables[0](0, 3);
			feature_window_size_ = (std::size_t)tables[0](0, 4);
			pred_spacing_ = (std::size_t)tables[0](0, 6);
			trained_ = (bool)tables[0](0, 7);
		}
		else {
			LOG(Warning) << "Contents of file given to RealTimeMultiClassifier::read_datalog() are invalid. No Parameters Table.";
			return false;
		}

		if (tables.size() != 2 * class_count_ + 2) {
			LOG(Warning) << "Contents of file given to RealTimeMultiClassifier::read_datalog() are invalid. Incorrect number of Tables.";
			return false;
		}

		set_class_count(class_count_);
		for (std::size_t i = 0; i < class_count_; ++i) {
			training_data_[i] = tables[i + 1].values();
		}
		for (std::size_t i = 0; i < class_count_; ++i) {
			feature_data_[i] = tables[i + class_count_ + 1].values();
		}

		w_ = tables[1 + 2 * class_count_].values();
		for (std::size_t i = 0; i < w_.size(); ++i) {
			w_0_[i] = w_[i].back();
			w_[i].pop_back();
		}

		return true;
	}

	bool RealTimeMultiClassifier::export_training_features(const std::string &filename, const std::string& directory, bool timestamp) {
		Table training_features("RtmcTrainingFeatures");
		for (std::size_t j = 0; j < get_feature_dim(); ++j) {
			training_features.push_back_col("phi_" + stringify(j));
		}
		std::vector<double> true_labels;
		for (std::size_t i = 0; i < class_count_; ++i) {			
			training_features.push_back_rows(feature_data_[i]);
			true_labels.insert(true_labels.end(), feature_data_[i].size(), i + 1);
		}		
		training_features.push_back_col("true_label", true_labels);
		return DataLogger::write_to_csv(training_features, filename, directory, timestamp);
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