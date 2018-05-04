#include "MEL/Utility/System.hpp"
#include "MEL/Utility/Console.hpp"
#include "MEL/Utility/Windows/Keyboard.hpp"
#include <MEL/Core/Clock.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEII/EMG/EmgDataCapture.hpp>
#include <MEL/Utility/RingBuffer.hpp>
#include <MEII/Utility/VirtualInput.hpp>
#include <MEL/Logging/DataLogger.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEII/Utility/Matrix.hpp>
#include <MEII/OpenSim/osim_utility.hpp>
#include <MEL/Communications/Windows/MelShare.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEII/EmgRealTimeControl/EmgRealTimeControl.hpp>

using namespace mel;
using namespace meii;

int main() {

	init_logger();

	std::vector<std::size_t> true_labels = gen_rand_class_labels(20, 4);
	std::cout << true_labels << "\r\n";
	std::vector<std::size_t> pred_labels = rand_shuffle_class_labels(5, 4);
	std::cout << pred_labels << "\r\n";

	Eigen::MatrixXd conf_mat = gen_confusion_mat(true_labels, pred_labels);
	std::cout << conf_mat << "\r\n";

    return 0;
}

