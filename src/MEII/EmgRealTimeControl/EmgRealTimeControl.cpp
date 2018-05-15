#include <MEII/EmgRealTimeControl/EmgRealTimeControl.hpp>
#include <MEL/Logging/Log.hpp>
#include <random>
#include <algorithm>
#include <ctime>


using namespace mel;

namespace meii {

	bool is_single_dof(DoF dof) {
		return dof < 4;
	}

	std::vector<std::size_t> gen_rand_class_labels(std::size_t num_labels, std::size_t num_classes, IndexBase index_base) {
		std::vector<std::size_t> class_label_list(num_labels, 0);
		if (num_labels > 0) {
			std::random_device rd_seed; // random seed
			std::mt19937 rd_gen(rd_seed()); // random unsigned integer generator
			std::uniform_int_distribution<> class_labels_dist(index_base, index_base + num_classes - 1);
			for (int i = 0; i < num_labels; ++i) {
				class_label_list[i] = (std::size_t)((unsigned)class_labels_dist(rd_gen));
			}
		}
	    return class_label_list;
	}
	
	std::vector<std::size_t> rand_shuffle_class_labels(std::size_t num_labels_per_class, std::size_t num_classes, IndexBase index_base) {
		std::vector<std::size_t> class_label_list(num_labels_per_class * num_classes, 0);
		if (num_labels_per_class > 0 && num_classes > 0) {
			for (int i = 0; i < num_labels_per_class * num_classes; ++i) {
				class_label_list[i] = index_base + (i / num_labels_per_class);
			}
			std::srand(std::time(0));
			std::random_shuffle(class_label_list.begin(), class_label_list.end());
		}
	    return class_label_list;
	}

	Eigen::MatrixXd gen_confusion_mat( const std::vector<double> &actual_labels, const std::vector<double> &predicted_labels, IndexBase index_base) {
		std::vector<std::size_t> actual_labels_u(actual_labels.size());
		for (std::size_t i = 0; i < actual_labels.size(); ++i) {
			actual_labels_u[i] = (std::size_t)actual_labels[i];
		}
		std::vector<std::size_t> predicted_labels_u(predicted_labels.size());
		for (std::size_t i = 0; i < predicted_labels.size(); ++i) {
			predicted_labels_u[i] = (std::size_t)predicted_labels[i];
		}
		return gen_confusion_mat(actual_labels_u, predicted_labels_u, index_base);
	}

	Eigen::MatrixXd gen_confusion_mat(const std::vector<std::size_t> &actual_labels, const std::vector<std::size_t> &predicted_labels, IndexBase index_base) {
		std::size_t N = actual_labels.size();

		std::size_t K = std::max(*std::max_element(actual_labels.begin(), actual_labels.end()), *std::max_element(predicted_labels.begin(), predicted_labels.end())) + 1 - index_base;
		Eigen::MatrixXd conf_mat = Eigen::MatrixXd::Zero(K, K);

		if (predicted_labels.size() != N) {
			LOG(Warning) << "Function gen_confusion_mat() received input arguments with different lengths.";
			return conf_mat;
		}

		//std::size_t min_val = std::min(*std::min_element(actual_labels.begin(), actual_labels.end()), *std::min_element(predicted_labels.begin(), predicted_labels.end()));
		std::size_t min_val = *std::min_element(actual_labels.begin(), actual_labels.end());
		if (min_val < index_base) {
			LOG(Warning) << "Function gen_confusion_mat() received input arguments containing class labels out of valid range.";
			return conf_mat;
		}

		for (std::size_t j = 0; j < K; ++j) {
			for (std::size_t i = 0; i < N; ++i) {
				if (actual_labels[i] == j + index_base) {
					if (predicted_labels[i] >= index_base && predicted_labels[i] <= K) {
						if (predicted_labels[i] == actual_labels[i]) {
							conf_mat(j, j) += 1;
						}
						else {
							conf_mat(j, predicted_labels[i] - index_base) += 1;
						}
					}
				}
			}
		}
		return conf_mat;
	}

} // namespace meii