#include <MEII/Control/WayPoint.hpp>
#include <MEL/Logging/Log.hpp>

using namespace mel;

namespace meii {

    WayPoint::WayPoint() :
        path_dim_(0)
    {}

    WayPoint::WayPoint(mel::Time time, std::vector<double> position) :
        time_(time),
        pos_(position),
        path_dim_(position.size())
    {}

    const mel::Time& WayPoint::when() const {
        return time_;
    }

    const std::vector<double>& WayPoint::get_pos() const {
        return pos_;
    }

    const double& WayPoint::operator[](std::size_t index) const {
        return pos_[index];
    }

    double& WayPoint::operator[](std::size_t index) {
        return pos_[index];
    }

    bool WayPoint::empty() const {
        return pos_.empty();
    }

    void WayPoint::set_time(const mel::Time& time) {
        time_ = time;
    }

    void WayPoint::resize(std::size_t path_dim) {
        path_dim_ = path_dim;
        pos_.resize(path_dim);
    }

    void WayPoint::set_pos(const std::vector<double>& pos) {
        pos_ = pos;
        path_dim_ = pos.size();
    }

    std::size_t WayPoint::get_dim() const {
        return path_dim_;
    }

    void WayPoint::clear() {
        path_dim_ = 0;
        time_ = Time::Zero;
        pos_.clear();
    }

} // namespace meii