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

#ifndef MEII_DYNAMIC_MOTION_PRIMITIVE_HPP
#define MEII_DYNAMIC_MOTION_PRIMITIVE_HPP

#include <MEII/Utility/Matrix.hpp>
#include <vector>
#include <MEL/Math/Integrator.hpp>
#include <MEL/Core/Time.hpp>
#include <MEII/Control/Trajectory.hpp>
#include <MEII/Control/WayPoint.hpp>

namespace meii {

    class DynamicMotionPrimitive {

    public:

        /// Constructor
        DynamicMotionPrimitive(mel::Time &sample_period, WayPoint &start, WayPoint &goal, Matrix &K, Matrix &D, double tau, Matrix(*nonlin_function)(const Matrix&, const Matrix&), std::vector<double> &theta, std::vector<double> &goal_tol);

        /// Destructor
        ~DynamicMotionPrimitive() {};

        Trajectory trajectory();

        void update(std::vector<double> theta);

        void clear();

    private:

        bool check_param_dim();

        void step();

    private:

        mel::Time Ts_;
        WayPoint g_;
        WayPoint q_0_;
        Matrix K_;
        Matrix D_;
        double tau_;
        std::vector<double> theta_;
        Matrix(*nonlin_function_)(const Matrix&, const Matrix&);
        double s_;
        std::vector<double> goal_tol_;
        std::size_t path_dim_;
        std::size_t path_size_;

        //std::vector<double> t_;
        
        std::vector<mel::Integrator> integrator_;
        //std::vector<double> q_;
        //std::vector<double> q_dot_;
        //std::vector<double> q_ddot_;
        //std::vector<double> q_prev_;
        //std::vector<double> q_dot_prev_;
        
        Matrix G_;
        Matrix Q_0_;
        Matrix Q_;
        Matrix Q_dot_;

        Trajectory trajectory_;

    };

} // namespace meii


#endif // MEII_DYNAMIC_MOTION_PRIMITIVE_HPP
