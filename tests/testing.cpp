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

	std::vector<std::size_t> labels = gen_rand_class_labels(20, 4);
	std::cout << labels << "\r\n";
	labels = rand_shuffle_class_labels(5, 4);
	std::cout << labels << "\r\n";

    return 0;
}

