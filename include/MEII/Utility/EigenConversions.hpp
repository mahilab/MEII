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

#ifndef MEII_EIGEN_CONVERSIONS_HPP
#define MEII_EIGEN_CONVERSIONS_HPP

#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace meii {

    void eigvec_to_stdvec(const Eigen::VectorXd& eigen_vec, std::vector<double>& std_vec);

    void stdvec_to_eigvec(std::vector<double>& std_vec, Eigen::VectorXd& eigen_vec);

    std::vector<double> copy_eigvec_to_stdvec(const Eigen::VectorXd& eigen_vec);

    Eigen::VectorXd copy_stdvec_to_eigvec(const std::vector<double>& std_vec);

    std::vector<std::vector<double>> copy_eigmat_to_stdvecvec(const Eigen::MatrixXd& eigen_mat);

    Eigen::MatrixXd copy_stdvecvec_to_eigmat(const std::vector<std::vector<double>>& std_vecvec);

    double mat_spectral_norm(const Eigen::MatrixXd& mat);

} // namespace meii

#endif // MEII_EIGEN_CONVERSIONS_HPP