#include <MEII/SignalProcessing/Rectifier.hpp>

using namespace mel;

namespace meii {

    Rectifier::Rectifier() {}

    double Rectifier::update(const double x, const Time& current_time) {
        Time unused = current_time; // unused
        return std::abs(x);
    }

    void Rectifier::reset() {}

} // namespace meii