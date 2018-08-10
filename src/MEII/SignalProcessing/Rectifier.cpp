#include <MEII/SignalProcessing/Rectifier.hpp>

using namespace mel;

namespace meii {

	Rectifier::Rectifier(bool full) :
		full_(full)
	{}

    double Rectifier::update(const double x, const Time& current_time) {
        Time unused = current_time; // unused
		if (full_) {
			return mel::abs(x);
		}
		else {
			if (x > 0) {
				return x;
			}
			else {
				return 0;
			}
		}
    }

    void Rectifier::reset() {}

} // namespace meii