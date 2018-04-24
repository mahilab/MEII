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

#ifndef MEII_WAYPOINT_HPP
#define MEII_WAYPOINT_HPP

#include <MEL/Math/Constants.hpp>
#include <MEL/Core/Time.hpp>
#include <vector>

namespace meii {

    //==============================================================================
    // CLASS DECLARATION
    //==============================================================================

    class WayPoint {

    public:
        /// Constructor
        WayPoint();
        WayPoint(const mel::Time &time, const std::vector<double> &position);

        const mel::Time &when() const;

        const std::vector<double> &get_pos() const;

		std::vector<double> get_point() const;

        /// Read access to position
        const double &operator[](std::size_t index) const;

        /// Write access to position
        double &operator[](std::size_t index);

        bool empty() const;

        WayPoint set_time(const mel::Time &time);

        /// Applies resize() to the position vector, changing its dimension and
        /// clearing the contents
        void resize(std::size_t path_dim);

        /// Overwrites the position vector associated with this point, resizing the
        /// vector to match the size of the input
        WayPoint set_pos(const std::vector<double> &pos);

        /// Returns the path dimension
        std::size_t get_dim() const;

        void clear();

		/// Overload the << stream operator with a WayPoint as the rhs argument
		friend std::ostream& operator<<(std::ostream& os, const WayPoint& waypoint);

    private:
        mel::Time time_;

        std::vector<double> pos_;

        std::size_t path_dim_;
    };

} // namespace meii

#endif // MEII_WAYPOINT_HPP