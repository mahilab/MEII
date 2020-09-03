// MIT License
//
// MEII - MAHI Exo-II Library
// Copyright (c) 2020 Mechatronics and Haptic Interfaces Lab - Rice University
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

#pragma once

#include <Mahi/Util/Math/Constants.hpp>
#include <Mahi/Util/Timing/Time.hpp>
#include <vector>

namespace meii {

    //==============================================================================
    // CLASS DECLARATION
    //==============================================================================

    class WayPoint {

    public:
        /// Constructor
        WayPoint();
        WayPoint(const mahi::util::Time &time, const std::vector<double> &position);

        const mahi::util::Time &when() const;

        const std::vector<double> &get_pos() const;

		std::vector<double> get_point() const;

        /// Read access to position
        const double &operator[](std::size_t index) const;

        /// Write access to position
        double &operator[](std::size_t index);

        bool empty() const;

        WayPoint set_time(const mahi::util::Time &time);

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
        mahi::util::Time time_;

        std::vector<double> pos_;

        std::size_t path_dim_;
    };

} // namespace meii