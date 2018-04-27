#include <MEII/Classification/RealTimeClassifier.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEII/SignalProcessing/SignalProcessingFunctions.hpp>
#include <MEII/Classification/LinearDiscriminantAnalysis.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Logging/DataLogger.hpp>

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
        //feature_dim_(get_feature_dim()),
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

        phi_ = feature_extraction(sample_buffer_.get_vector());
        y_1_ = w_0_;
        for (std::size_t i = 0; i < get_feature_dim(); ++i) {
            y_1_ += w_[i] * phi_[i];
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
        if (w.size() != get_feature_dim()) {
            LOG(Warning) << "Size of classification model provided to RealTimeClassifier::set_model() must match the size of the feature dimension. Model not set.";
            return trained_ = false;
        }
        w_ = w;
        w_0_ = w_0;
        return trained_ = true;
    }

    bool RealTimeClassifier::set_model(std::vector<double> w_full) {
        if (w_full.size() != get_feature_dim() + 1) {
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

	bool RealTimeClassifier::save(const std::string &filename, const std::string& directory, bool timestamp) {
		return DataLogger::write_to_csv(make_datalog(), filename, directory, timestamp);
	}

	bool RealTimeClassifier::load(const std::string &filename, const std::string& directory) {
		std::vector<Table> tables;
		if (DataLogger::read_from_csv(tables, filename, directory)) {
			return read_datalog(tables);
		}
		else {
			return false;
		}
	}

	std::vector<Table> RealTimeClassifier::make_datalog() const {
		std::vector<Table> tables;

		Table params("Parameters", { "sample_dim", "Ts", "classification_window_size", "pred_spacing", "feature_dim", "trained" });
		std::vector<double> params_values;
		params_values.push_back((double)sample_dim_);
		params_values.push_back(Ts_.as_seconds());
		params_values.push_back((double)classification_window_size_);
		params_values.push_back((double)feature_window_size_);
		params_values.push_back((double)pred_spacing_);
		params_values.push_back((double)trained_);
		params.set_values({ params_values });
		tables.push_back(params);

		Table class_0_training_data("Class0TrainingData");
		for (std::size_t i = 0; i < sample_dim_; ++i) {
			class_0_training_data.push_back_col("x_" + stringify(i));
		}
		class_0_training_data.set_values(class_0_training_data_);
		tables.push_back(class_0_training_data);

		Table class_1_training_data("Class1TrainingData");
		for (std::size_t i = 0; i < sample_dim_; ++i) {
			class_1_training_data.push_back_col("x_" + stringify(i));
		}
		class_1_training_data.set_values(class_1_training_data_);
		tables.push_back(class_1_training_data);

		Table class_0_feature_data("Class0FeatureData");
		for (std::size_t i = 0; i < get_feature_dim(); ++i) {
			class_0_feature_data.push_back_col("phi_" + stringify(i));
		}
		class_0_feature_data.set_values(class_0_feature_data_);
		tables.push_back(class_0_feature_data);

		Table class_1_feature_data("Class1FeatureData");
		for (std::size_t i = 0; i < get_feature_dim(); ++i) {
			class_1_feature_data.push_back_col("phi_" + stringify(i));
		}
		class_1_feature_data.set_values(class_1_feature_data_);
		tables.push_back(class_1_feature_data);

		Table model("Model");
		model.push_back_col("w", get_model());
		tables.push_back(model);

		return tables;
	}

	bool RealTimeClassifier::read_datalog(const std::vector<mel::Table> &tables) {
		if (tables.size() != 6) {
			LOG(Warning) << "Contents of file given to RealTimeClassifier::load() are invalid. Incorrect number of Tables.";
			return false;
		}

		if (tables[0].name().compare("Parameters") == 0) {
			sample_dim_ = (std::size_t)tables[0](0, 0);
			Ts_ = seconds(tables[0](0, 1));
			classification_window_size_ = (std::size_t)tables[0](0, 2);
			feature_window_size_ = (std::size_t)tables[0](0, 3);
			pred_spacing_ = (std::size_t)tables[0](0, 4);
			trained_ = (bool)tables[0](0, 5);
		}
		else {
			LOG(Warning) << "Contents of file given to RealTimeClassifier::load() are invalid. No Parameters Table.";
			return false;
		}

		if (tables[1].name().compare("Class0TrainingData") == 0) {
			class_0_training_data_ = tables[1].values();
		}
		else {
			LOG(Warning) << "Contents of file given to RealTimeClassifier::load() are invalid. No Class0TrainingData Table.";
			return false;
		}

		if (tables[2].name().compare("Class1TrainingData") == 0) {
			class_1_training_data_ = tables[2].values();
		}
		else {
			LOG(Warning) << "Contents of file given to RealTimeClassifier::load() are invalid. No Class1TrainingData Table.";
			return false;
		}

		if (tables[3].name().compare("Class0FeatureData") == 0) {
			class_0_feature_data_ = tables[3].values();
		}
		else {
			LOG(Warning) << "Contents of file given to RealTimeClassifier::load() are invalid. No Class0FeatureData Table.";
			return false;
		}

		if (tables[4].name().compare("Class1FeatureData") == 0) {
			class_1_feature_data_ = tables[4].values();
		}
		else {
			LOG(Warning) << "Contents of file given to RealTimeClassifier::load() are invalid. No Class1FeatureData Table.";
			return false;
		}

		if (tables[5].name().compare("Model") == 0) {
			set_model(tables[5].get_col(0));
		}
		else {
			LOG(Warning) << "Contents of file given to RealTimeClassifier::load() are invalid. No Model Table.";
			return false;
		}

		return true;
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