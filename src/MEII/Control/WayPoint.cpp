#include <MEII/Control/WayPoint.hpp>
#include <Mahi/Util/Logging/Log.hpp>

using namespace mahi::util;

namespace meii {

    WayPoint::WayPoint() :
        path_dim_(0)
    {}

    WayPoint::WayPoint(const Time &time, const std::vector<double> &position) :
        time_(time),
        pos_(position),
        path_dim_(position.size())
    {}

    const Time& WayPoint::when() const {
        return time_;
    }

    const std::vector<double>& WayPoint::get_pos() const {
        return pos_;
    }

	std::vector<double> WayPoint::get_point() const {
		std::vector<double> point(path_dim_ + 1);
		point[0] = time_.as_seconds();
		std::copy(pos_.begin(), pos_.end(), point.begin() + 1);
		return point;
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

    WayPoint WayPoint::set_time(const Time& time) {
        time_ = time;
		return *this;
    }

    void WayPoint::resize(std::size_t path_dim) {
        path_dim_ = path_dim;
        pos_.resize(path_dim);
    }

    WayPoint WayPoint::set_pos(const std::vector<double>& pos) {
        pos_ = pos;
        path_dim_ = pos.size();
		return *this;
    }

    std::size_t WayPoint::get_dim() const {
        return path_dim_;
    }

    void WayPoint::clear() {
        path_dim_ = 0;
        time_ = Time::Zero;
        pos_.clear();
    }

	std::ostream& operator<<(std::ostream& os, const WayPoint& waypoint) {
		return os << waypoint.when() << "\t" << waypoint.get_pos() << std::endl;
	}

} // namespace meii