#include <MEL/Logging/Log.hpp>
#include <MEII/EmgRealTimeControl/EmgRealTimeControl.hpp>

using namespace mel;
using namespace meii;

int main() {
	init_logger();
	std::vector<double> actual_labels =    { 1, 2, 4, 1, 3, 1, 4, 4, 2, 3, 2, 3 };
	std::vector<double> predicted_labels = { 1, 0, 0, 1, 3, 1, 0, 4, 3, 2, 2, 3 };
	Eigen::MatrixXd conf_mat = gen_confusion_mat(actual_labels, predicted_labels, One);
	std::cout << conf_mat;
    return 0;
}

