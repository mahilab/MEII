#include <MEII/Control/Trajectory.hpp>
#include <MEII/MahiExoII/MahiExoII.hpp>
#include <MEL/Logging/Log.hpp>
#include <algorithm>
#include <MEL/Math/Functions.hpp>

using namespace mel;

namespace meii {

    Trajectory::Trajectory() :
        is_empty_(true),
        path_dim_(0),
		interp_method_(Interp::Linear),
		max_diff_({ mel::INF })
    {}

    Trajectory::Trajectory(std::size_t path_dim, const std::vector<WayPoint> &waypoints, Interp interp_method, const std::vector<double> &max_diff) :
        is_empty_(false),
        path_dim_(path_dim),
        waypoints_(waypoints),
        interp_method_(interp_method),
        times_(waypoints_.size()),
        max_diff_(max_diff)
    {
        if (!check_max_diff()) {
            LOG(Warning) << "Clearing Trajectory.";
            clear();
            return;
        }
        if (!check_waypoints()) {
            LOG(Warning) << "Clearing Trajectory.";
            clear();
            return;
        }

        for (std::size_t i = 0; i < waypoints_.size(); ++i) {
            times_[i] = waypoints_[i].when();
        }
    }

    std::vector<double> Trajectory::at_time(Time &instant, Interp interp_method) const {
        if (empty()) {
            LOG(Warning) << "Attempted to access an empty trajectory at a certain time. Returning empty vector.";
            return std::vector<double>();
        }
        if (instant < times_.front()) {
            return waypoints_.front().get_pos();
        }
        else if (instant > times_.back()) {
            return waypoints_.back().get_pos();
        }
        std::size_t after = std::distance(times_.begin(), std::find_if(times_.begin(), times_.end(), [instant](mel::Time t) {return t >= instant; }));
        std::size_t before = after > 0 ? after - 1 : after;
        switch (interp_method) {
        case Interp::Linear:
            return linear_time_interpolate(waypoints_[before], waypoints_[after], instant).get_pos();
            break;
        default:
            LOG(Error) << "Invalid interpolation method used in Trajectory::at_time().";
            return std::vector<double>();
        }
    }

    const WayPoint& Trajectory::operator[](std::size_t index) const {
        return waypoints_[index];
    }

	const WayPoint& Trajectory::front() const {
		return waypoints_.front();
	}

	const WayPoint& Trajectory::back() const {
		return waypoints_.back();
	}

    bool Trajectory::set_waypoints(std::size_t path_dim, const std::vector<WayPoint>& waypoints, Interp interp_method, const std::vector<double> &max_diff) {
        is_empty_ = false;
        path_dim_ = path_dim;
        waypoints_ = waypoints;
        interp_method_ = interp_method;
        max_diff_ = max_diff;
        if (!check_max_diff()) {
            LOG(Warning) << "Clearing Trajectory.";
            clear();
            return false;
        } 
        if (!check_waypoints()) {
            LOG(Warning) << "Clearing Trajectory.";
            clear();
            return false;
        }
        times_.resize(waypoints_.size());
        for (std::size_t i = 0; i < waypoints_.size(); ++i) {
            times_[i] = waypoints_[i].when();
        }
        return true;
    }

    void Trajectory::set_interp_method(Interp interp_method) {
        interp_method_ = interp_method;
    }

    void Trajectory::set_max_diff(std::vector<double> &max_diff) {
        std::vector<double> old_max_diff = max_diff_;
        max_diff_ = max_diff;
        if (!check_max_diff()) {
            LOG(Warning) << "Parameter max_diff not changed.";
            max_diff_ = old_max_diff;
        }
    }

    bool Trajectory::empty() const {
        return is_empty_;
    }

    std::size_t Trajectory::size() const {
        return times_.size();
    }

    std::size_t Trajectory::get_dim() const {
        return path_dim_;
    }

    void Trajectory::clear() {
        is_empty_ = true;
        path_dim_ = 0;
        waypoints_.clear();
        times_.clear();
        max_diff_.clear();
    }

    bool Trajectory::push_back(const WayPoint &waypoint) {        
        if (is_empty_) {
            path_dim_ = waypoint.get_dim();
			is_empty_ = false;
        }
		else {
			if (waypoint.get_dim() != path_dim_) {
				LOG(Warning) << "Input waypoint given to Trajectory contains wrong dimension. Waypoint not added.";
				return false;
			}
			if (waypoint.when() < waypoints_.back().when()) {
				LOG(Warning) << "Input waypoint times must be monotonically increasing or not changing. Waypoint not added.";
				return false;
			}
        }
		waypoints_.push_back(waypoint);
		times_.push_back(waypoint.when());
		return true;
    }

    bool Trajectory::check_max_diff() {
        if (max_diff_.size() != path_dim_) {
            if (max_diff_.size() == 1) {
                max_diff_ = std::vector<double>(path_dim_, max_diff_[0]);
            }
            else {
                LOG(Warning) << "Input max_diff given to Trajectory constructor must either be of size 1 or of size path_dim.";
                return false;
            }
        }
        return true;
    }

    bool Trajectory::check_waypoints() {
        Time t = waypoints_[0].when();
        for (std::size_t i = 0; i < waypoints_.size(); ++i) {
            if (waypoints_[i].get_dim() != path_dim_) {
                LOG(Warning) << "Input waypoints given to Trajectory contains Points of the wrong size.";
                return false;
            }
            if (waypoints_[i].when() < t) {
                LOG(Warning) << "Input waypoint times must be monotonically increasing or not changing.";
                return false;
            }
            t = waypoints_[i].when();
        }

        if (!is_path_smooth()) {
            LOG(Warning) << "Trajectory path does not satisfy required smoothness set by max_diff.";
            return false;
        }
        return true;
    }

    bool Trajectory::is_path_smooth() const {
        if (empty()) {
            LOG(Warning) << "Attempted to access call member function is_path_smooth() on empty trajectory. Returning false.";
            return false;
        }
        bool is_smooth = true;
        for (std::size_t i = 0; i < waypoints_.size() - 1; ++i) {
            if (waypoints_[i + 1].when() != waypoints_[i].when()) {
                for (std::size_t j = 0; j < path_dim_; ++j) {
                    if (std::abs(waypoints_[i + 1][j] - waypoints_[i][j]) / (waypoints_[i + 1].when().as_seconds() - waypoints_[i].when().as_seconds()) > max_diff_[j]) {
                        is_smooth = false;
                    }
                }
            }
        }
        return is_smooth;
    }

    std::vector<WayPoint> Trajectory::linspace_points(const WayPoint& initial, const WayPoint& final, std::size_t n, bool include_initial, bool include_final) {
        if (initial.empty() || final.empty()) {
            LOG(Error) << "Input points given to Trajectory::linear_interpolate() cannot be empty.";
            return { initial };
        }
        if (initial.get_dim() != final.get_dim()) {
            LOG(Error) << "Input points given to Trajectory::linear_interpolate() must be of same size.";
            return { initial };
        }
        if (!include_initial) {
            n++;
        }
        if (!include_final) {
            n++;
        }
        std::vector<double> times = linspace(initial.when().as_seconds(), final.when().as_seconds(), n);
        std::vector<WayPoint> interp_points(n);
        for (std::size_t i = 0; i < n; ++i) {
            interp_points[i] = linear_time_interpolate(initial, final, seconds(times[i]));
        }
        if (!include_initial) {
            interp_points.erase(interp_points.begin());
        }
        if (!include_final) {
            interp_points.pop_back();
        }
        return interp_points;
    }

    WayPoint Trajectory::linear_time_interpolate(const WayPoint& initial, const WayPoint& final, const mel::Time& t) {
        if (initial.empty() || final.empty()) {
            LOG(Error) << "Input points given to Trajectory::linear_time_interpolate() cannot be empty.";
            return { initial };
        }
        if (initial.get_dim() != final.get_dim()) {
            LOG(Error) << "Input points given to Trajectory::linear_time_interpolate() must be of same size.";
            return { initial };
        }
        WayPoint interp_point;
        if (t <= initial.when()) {
            return { initial };
        }
        if (t >= final.when()) {
            return { final };
        }
        interp_point.set_time(t);
        interp_point.resize(initial.get_dim());
        for (std::size_t i = 0; i < initial.get_dim(); ++i) {
            interp_point[i] = initial[i] + (t.as_seconds() - initial.when().as_seconds()) * (final[i] - initial[i]) / (final.when().as_seconds() - initial.when().as_seconds());
        }

        return interp_point;
    }

} // namespace meii