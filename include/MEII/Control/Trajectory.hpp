// MIT License
//
// MEII - MAHI Exo-II Extension of MEL, the MAHI Exoskeleton Library
// Copyright (c) 2018 Mechatronics and Haptic Interfaces Lab - Rice University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// Author(s): Craig McDonald (craig.g.mcdonald@gmail.com)

#ifndef MEII_TRAJECTORY_HPP
#define MEII_TRAJECTORY_HPP

#include <MEII/Control/WayPoint.hpp>
#include <MEL/Math/Constants.hpp>
#include <MEL/Core/Time.hpp>
#include <vector>

namespace meii {

    //==============================================================================
    // CLASS DECLARATION
    //==============================================================================

    class Trajectory {

    public:
        enum Interp { Linear };

    public:
        /// Returns vector of n time-linearly-interpolated points with equal spacing
        /// in time. Include initial/final set whether or not they will be included in
        /// the output, and count towards n.
        static std::vector<WayPoint> linspace_points(const WayPoint &initial,
            const WayPoint &final, std::size_t n,
            bool include_initial = true,
            bool include_final = true);

        /// Linearly interpolates between the two points based on the given time.
        /// Saturates to initial and final values if time is outside of range.
        static WayPoint linear_time_interpolate(const WayPoint &initial, const WayPoint &final,
            const mel::Time &t);

        /// Constructor
        Trajectory();
        Trajectory(std::size_t path_dim, const std::vector<WayPoint> &waypoints,
            Interp interp_method = Interp::Linear,
            const std::vector<double> &max_diff = { mel::INF });

        /// Destructor
        ~Trajectory() {};

        /// Returns a position along the trajectory at the specific instant in time
        /// using one of the available interpolation methods
        virtual std::vector<double>
            at_time(mel::Time &instant, Interp interp_method = Interp::Linear) const;

        /// Index-based read access to waypoints
        const WayPoint &operator[](std::size_t index) const;

		/// Return the first waypoint
		const WayPoint &front() const; 

		/// Return the last waypoint
		const WayPoint &back() const;

        /// Sets the waypoints that make the trajectory
        bool set_waypoints(std::size_t path_dim, const std::vector<WayPoint> &waypoints,
            Interp interp_method = Interp::Linear,
            const std::vector<double> &max_diff = { mel::INF });

        /// Sets the method of interpolation to be used
        void set_interp_method(Interp interp_method);

        /// Sets the maximum allowable time derivative on the trajectory
        void set_max_diff(std::vector<double> &max_diff);

        /// Returns whether or not the trajectory is empty, having no points
        bool empty() const;

        /// Return the length of the trajectory, meaning the number of Points stored
        std::size_t size() const;

        /// Returns the path dimension
        std::size_t get_dim() const;

        /// Clears the waypoints, setting the trajectory to empty
        void clear();

        /// Adds a single waypoint to the end of the list
        bool push_back(const WayPoint &waypoint);

    private:
        bool check_max_diff();

        bool check_waypoints();

        /// Checks if current trajectory satisfies smoothness criterion based on
        /// max_diff
        bool is_path_smooth() const;

    private:
        bool is_empty_; ///< whether or not this trajectory is empty, having no points

        std::size_t path_dim_; ///< dimensionality of the path

        Interp interp_method_; ///< method of interpolation to be used: None = no
                               ///< points added to the path, Linear = new points
                               ///< automatically added between waypoints using linear
                               ///< interpolation at the requested resolution

        std::vector<WayPoint> waypoints_; ///< vector of underlying waypoints

        std::vector<mel::Time>
            times_; ///< vector of the times associated with the underlying waypoints

        std::vector<double> max_diff_;

        // std::vector<std::vector<double>> paths_;
    };

} // namespace meii

#endif // MEII_TRAJECTORY_HPP