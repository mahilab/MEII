//#include <Control/DiscreteTrajectory.hpp>
//#include <MahiExoII/MahiExoII.hpp>
//#include <MEL/Logging/Log.hpp>
//#include <algorithm>
//#include <MEL/Math/Functions.hpp>
//
//
//using namespace mel;
//
//
//DiscreteTrajectory::DiscreteTrajectory() :
//    Trajectory(),
//    res_(0),
//    interp_method_(Interp::None),
//    time_(std::vector<Time>()),
//    path_(std::vector<std::vector<double>>())
//{}
//
//DiscreteTrajectory::DiscreteTrajectory(std::size_t path_dim, const std::vector<Point>& waypoints, Interp interp_method, const std::vector<double>& resolution) :
//    Trajectory(path_dim, waypoints),
//    interp_method_(interp_method)
//{
//    if (resolution.size() != path_dim_) {
//        if (resolution.size() == 1)
//            res_ = std::vector<double>(path_dim_, resolution[0]);
//        else
//            LOG(Error) << "Input resolution given to Trajectory constructor must either be of size 1 or of size path_dim.";
//    }
//    else {
//        res_ = resolution;
//    }
//    for (std::size_t i = 0; i < path_dim_; ++i) {
//        res_[i] = max(min_res_, res_[i]);
//    }
//    interpolate_waypoints();
//}
//
//std::vector<double> DiscreteTrajectory::at_time(Time instant, Interp interp_method) const {
//    if (empty()) {
//        LOG(Warning) << "Attempted to access an empty trajectory at a certain time. Returning empty vector.";
//        return std::vector<double>();
//    }
//    if (instant < time_.front()) {
//        return path_.front();
//    }
//    else if (instant > time_.back()) {
//        return path_.back();
//    }
//    std::size_t after = std::distance(time_.begin(), std::find_if(time_.begin(), time_.end(), [instant](mel::Time t) {return t > instant; }));
//    std::size_t before = after - 1;
//    std::vector<double> pos(path_dim_);
//    switch (interp_method) {
//    case Interp::None:
//        if (instant - time_[before] > time_[after] - instant)
//            pos = path_[after];
//        else
//            pos = path_[before];
//        break;
//    case Interp::Linear:
//        for (std::size_t i = 0; i < path_dim_; ++i) {
//            pos[i] = path_[before][i] + (instant - time_[before]) * (path_[after][i] - path_[before][i]) / (time_[after] - time_[before]);
//        }
//        break;
//    default:
//        LOG(Error) << "Invalid interpolation method used in Trajectory::at_time().";
//        return std::vector<double>();
//    }
//
//    return pos;
//}
//
//
//void DiscreteTrajectory::set_min_res(double min_res) {
//    min_res_ = min_res;
//    if (!is_empty_) {
//        for (std::size_t i = 0; i < path_dim_; ++i) {
//            res_[i] = max(min_res_, res_[i]);
//        }
//    }
//}
//
//void DiscreteTrajectory::set_waypoints(std::size_t path_dim, const std::vector<Point>& waypoints, Interp interp_method, const std::vector<double>& resolution) {
//
//}
//
//void DiscreteTrajectory::set_interp_method(Interp interp_method) {
//
//}
//
//std::size_t DiscreteTrajectory::size() const {
//    return time_.size();
//}
//
////const std::vector<double>& DiscreteTrajectory::get_pos(std::size_t index) const {
////    return path_[index];
////}
//
////const Trajectory::Point& DiscreteTrajectory::get_point(std::size_t index) {
////    return current_point_ = Point(time_[index], path_[index]);
////}
////
////const std::vector<double>& DiscreteTrajectory::operator[](std::size_t index) const{
////    return path_[index];
////}
//
//const Trajectory::Point& DiscreteTrajectory::operator[](std::size_t index) {
//    return current_point_ = Point(time_[index], path_[index]);
//}
//
//void DiscreteTrajectory::interpolate_waypoints() {
//    if (waypoints_.empty()) {
//        LOG(Error) << "Input waypoints given to Trajectory::interpolate_waypoints() was empty.";
//        return;
//    }
//    for (std::size_t i = 0; i < waypoints_.size(); ++i) {
//        if (waypoints_[i].get_dim() != path_dim_) {
//            LOG(Error) << "Input waypoints given to Trajectory::interpolate_waypoints() contains Points of the wrong size.";
//            return;
//        }
//    }
//    std::vector<Point> interp_points;
//    std::size_t num_interp_points(0);
//    switch (interp_method_) {
//    case Interp::None:
//        time_.resize(waypoints_.size());
//        path_.resize(waypoints_.size());
//        for (std::size_t i = 0; i < waypoints_.size(); ++i) {
//            time_[i] = waypoints_[i].when();
//            path_[i] = waypoints_[i].get_pos();
//        }
//        break;
//    case Interp::Linear:
//        time_ = { waypoints_[0].when() };
//        path_ = { waypoints_[0].get_pos()};      
//        for (std::size_t i = 1; i < waypoints_.size(); ++i) {
//            for (std::size_t j = 0; j < path_dim_; ++j) {
//                num_interp_points = std::max( num_interp_points, (std::size_t)((unsigned)std::ceil( std::abs(waypoints_[i][j] - waypoints_[i - 1][j]) / res_[j] )) + 1);
//            }
//            interp_points = linspace_points(waypoints_[i - 1], waypoints_[i], num_interp_points);
//            time_.pop_back();
//            path_.pop_back();
//            for (std::size_t j = 0; j < interp_points.size(); ++j) {
//                time_.push_back(interp_points[j].when());
//                path_.push_back(interp_points[j].get_pos());
//            }
//        }
//        break;
//    default:
//        LOG(Error) << "Invalid interpolation method used in Trajectory::interpolate_waypoints().";
//        return;
//    }
//}
//
//
//
////std::vector<double> DiscreteTrajectory::interpolate(const std::vector<double>& initial, const std::vector<double>& final, Interp interp_method = Interp::None) const {
////    switch (interp_method) {
////    case Interp::None:
////        if (instant - time_[before] > time_[after] - instant)
////            pos = path_[after];
////        else
////            pos = path_[before];
////        break;
////    case Interp::Linear:
////        for (std::size_t i = 0; i < path_dim_; ++i) {
////            pos[i] = path_[before][i] + (instant - time_[before]) * (path_[after][i] - path_[before][i]) / (time_[after] - time_[before]);
////        }
////        break;
////    default:
////        LOG(Error) << "Invalid interpolation method used in Trajectory::at_time().";
////        return std::vector<double>();
////    }
////}