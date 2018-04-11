//// MIT License
////
//// MEII - MAHI Exo-II Extension of MEL, the MAHI Exoskeleton Library
//// Copyright (c) 2018 Mechatronics and Haptic Interfaces Lab - Rice University
////
//// Permission is hereby granted, free of charge, to any person obtaining a copy
//// of this software and associated documentation files (the "Software"), to deal
//// in the Software without restriction, including without limitation the rights
//// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//// copies of the Software, and to permit persons to whom the Software is
//// furnished to do so, subject to the following conditions:
////
//// The above copyright notice and this permission notice shall be included in
//// all copies or substantial portions of the Software.
////
//// Author(s): Craig McDonald (craig.g.mcdonald@gmail.com)
//
//#ifndef MEII_DISCRETE_TRAJECTORY_HPP
//#define MEII_DISCRETE_TRAJECTORY_HPP
//
//#include <Control/Trajectory.hpp>
//#include <MEL/Utility/Time.hpp>
//#include <vector>
//
//
////==============================================================================
//// CLASS DECLARATION
////==============================================================================
//
//class DiscreteTrajectory : public Trajectory {
//
//public:
//
//    
//
//public:
//
//    /// Constructor
//    DiscreteTrajectory();
//    DiscreteTrajectory(std::size_t path_dim, const std::vector<Point>& waypoints, Interp interp_method = Interp::None, const std::vector<double>& resolution = { 0.0 });
//
//    /// Destructor
//    ~DiscreteTrajectory() {};
//
//    /// Returns a position along the trajectory at the specific instant in time using one of the available interpolation methods
//    virtual std::vector<double> at_time(mel::Time instant, Interp inter_method = Interp::None) const override;
//
//    /// Sets the minimum resolution that the each dimension of the path is allowed to have after interpolation
//    void set_min_res(double min_res);
//
//    /// Sets the waypoints that make the trajectory and performs requested interpolation, essentially reconstructing the trajectory
//    void set_waypoints(std::size_t path_dim, const std::vector<Point>& waypoints, Interp interp_method = Interp::None, const std::vector<double>& resolution = { 0.0 });
//
//    /// Sets the method of interpolation to be used: None = no points added to the path, Linear = new points automatically added between waypoints using linear interpolation at the requested resolution
//    void set_interp_method(Interp interp_method);
//
//    /// Return the length of the trajectory, meaning the number of Points stored
//    std::size_t size() const;
//
//    /// Index-based read access to underlying discrete path
//    //const std::vector<double>& get_pos(std::size_t index) const;
//    //const std::vector<double>& operator[](std::size_t index) const;    
//
//    /// Index-based read access to underlying discrete points
//    //const Point& get_point(std::size_t index);
//    const Trajectory::Point& operator[](std::size_t index);
//
//private:
//
//    void interpolate_waypoints();
//
//    
//
//    //std::vector<double> interpolate(const std::vector<double>& initial, const std::vector<double>& final, Interp interp_method = Interp::None) const;
//
//private:
//
//    std::vector<double> res_; ///< same size as path dimension, giving a resolution to be used by the interpolation method on each dimension fo the path
//    double min_res_ = 0.001; ///< minimum resolution that the each dimension of the path is allowed to have after interpolation
//
//    Interp interp_method_; ///< method of interpolation to be used: None = no points added to the path, Linear = new points automatically added between waypoints using linear interpolation at the requested resolution   
//
//    std::vector<mel::Time> time_; ///< vector of times corresponding to the positions in path
//
//    std::vector<std::vector<double>> path_; ///< vector of positions vectors, each of size path_dim_, that make up the trajectory
//
//    Point current_point_;
//
//
//};
//
//#endif // MEII_DISCRETE_TRAJECTORY_HPP