#include <MEII/EmgRealTimeControl/EmgRealTimeControl.hpp>
#include <random>
#include <algorithm>
#include <ctime>

namespace meii {

	bool is_single_dof(DoF dof) {
		return dof < 4;
	}

	std::vector<std::size_t> gen_rand_class_labels(std::size_t num_labels, std::size_t num_classes) {
	    std::random_device rd_seed; // random seed
	    std::mt19937 rd_gen(rd_seed()); // random unsigned integer generator
	    std::uniform_int_distribution<> class_labels_dist(1, num_classes);
	    std::vector<std::size_t> class_label_list(num_labels , 0);
	    for (int i = 0; i < num_labels; ++i) {
	        class_label_list[i] = (std::size_t)((unsigned)class_labels_dist(rd_gen));
	    }
	    return class_label_list;
	}
	
	std::vector<std::size_t> rand_shuffle_class_labels(std::size_t num_labels_per_class, std::size_t num_classes) {
	    std::vector<std::size_t> class_label_list(num_labels_per_class * num_classes, 0);
	    for (int i = 0; i < num_labels_per_class * num_classes; ++i) {
	        class_label_list[i] = 1 + (i / num_labels_per_class);
	    }
	    std::srand(std::time(0));
	    std::random_shuffle(class_label_list.begin(), class_label_list.end());
	    return class_label_list;
	}

	//Eigen::MatrixXd EmgRealTimeControl::gen_confusion_mat( const std::vector<int>& actual_labels, const std::vector<int>& predicted_labels) const {
	//    size_t N = actual_labels.size();
	//
	//    // assuming the class labels begin at 1
	//    int K = std::max(*std::max_element(actual_labels.begin(), actual_labels.end()), *std::max_element(predicted_labels.begin(), predicted_labels.end()));
	//    Eigen::MatrixXd conf_mat = Eigen::MatrixXd::Zero(K, K);
	//
	//    if (predicted_labels.size() != N) {
	//        print("ERROR: Function gen_confusion_mat received input arguments with different lengths.");
	//        return conf_mat;
	//    }
	//
	//    int min_val = std::min(*std::min_element(actual_labels.begin(), actual_labels.end()), *std::min_element(predicted_labels.begin(), predicted_labels.end()));
	//    if (min_val < 1) {
	//        print("ERROR: Function gen_confusion_mat received input arguments containing class labels less than 1.");
	//        return conf_mat;
	//    }
	//    
	//    for (size_t j = 0; j < K; ++j) {
	//        for (size_t i = 0; i < N; ++i) {
	//            if (actual_labels[i] == j + 1) {
	//                if (predicted_labels[i] == actual_labels[i]) {
	//                    conf_mat(j, j) += 1;
	//                }
	//                else {
	//                    conf_mat(j, predicted_labels[i] - 1) += 1;
	//                }
	//            }
	//        }
	//    }
	//    
	//    return conf_mat;
	//    
	//}

} // namespace meii