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

using namespace mel;
using namespace meii;

int main() {

	MeiiOsimMotTable meii_log;
	meii_log.push_back_row(std::vector<double>(6, 0.0));
	meii_log.push_back_row(std::vector<double>(6, 1.0));
	meii_log.push_back_row(std::vector<double>(6, 2.0));
	//write_meii_to_osim_mot(meii_log);
	//print(meii_log.get_col_names());
    return 0;
}

