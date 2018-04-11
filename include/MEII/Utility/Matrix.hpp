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

#ifndef MEII_MATRIX_HPP
#define MEII_MATRIX_HPP

#include <vector>

namespace meii {

    class Matrix {

    public:

        /// Constructor
        Matrix();
        Matrix(std::size_t n_rows_n_cols);
        Matrix(std::size_t n_rows, std::size_t n_cols, double value = 0.0);
        Matrix(std::vector<double> vec);

        /// Destructor
        ~Matrix() {};

        /// Read access in 2D
        const double& operator()(std::size_t row_idx, std::size_t col_idx) const;

        /// Read access in 1D
        const double& operator()(std::size_t idx) const;

        /// Write access in 2D
        double& operator()(std::size_t row_idx, std::size_t col_idx);

        /// Write access in 1D
        double& operator()(std::size_t idx);

        /// Returns true if Matrix is empty
        bool empty() const;

        /// Returns the size of the Matrix 
        std::vector<std::size_t> size() const;

        /// Returns the number of rows in the Matrix
        std::size_t rows() const;

        /// Returns the number of columns in the Matrix
        std::size_t cols() const;

        /// Returns the Matrix after applying the transpose operation without complex conjugation
        Matrix transpose() const;

        /// Resizes the Matrix and clears it's values
        void resize(std::size_t n_rows_n_cols);
        void resize(std::size_t n_rows, std::size_t n_cols);

        /// Clears all the values in the Matrix
        void clear();

        /// Overload the += operator with a scalar double as the rhs argument
        Matrix operator+=(double scalar);

        /// Overload the + operator with a scalar double as the rhs argument
        Matrix operator+(double scalar) const;

        /// Overload the -= operator with a scalar double as the rhs argument
        Matrix operator-=(double scalar);

        /// Overload the - operator with a scalar double as the rhs argument
        Matrix operator-(double scalar) const;

        /// Overload the *= operator with a scalar double as the rhs argument
        Matrix operator*=(double scalar);

        /// Overload the * operator with a scalar double as the rhs argument
        Matrix operator*(double scalar) const;

        /// Overload the += operator with a Matrix as the rhs argument
        Matrix operator+=(const Matrix& rhs);

        /// Overload the + operator with a Matrix as the rhs argument
        Matrix operator+(const Matrix& rhs) const;

        /// Overload the -= operator with a Matrix as the rhs argument
        Matrix operator-=(const Matrix& rhs);

        /// Overload the - operator with a Matrix as the rhs argument
        Matrix operator-(const Matrix& rhs) const;

        /// Overload the *= operator with a Matrix as the rhs argument
        Matrix operator*=(const Matrix& rhs);

        /// Overload the * operator with a Matrix as the rhs argument
        Matrix operator*(const Matrix& rhs) const;

        /// Overload the * operator with a std::vector double as the rhs argument
        std::vector<double> operator*(const std::vector<double>& vec) const;

        /// Overload the << stream operator with a Matrix as the rhs argument
        friend std::ostream& operator<<(std::ostream& os, const Matrix& mat);

        //std::vector<T> get_row(std::size_t row_idx) {
        //    std::vector<T> row(n_cols_);
        //    for (std::size_t i = 0; i < n_cols_; ++i) {
        //        row[i] = values_[row_idx * n_cols_ + i];
        //    }
        //    return row;
        //}
    

    private:

        std::size_t n_rows_; ///< number of rows in the matrix representation
        std::size_t n_cols_; ///< number of columns in the matrix representation
        std::size_t n_tot_; ///< total number of elements in the Matrix, equal to n_rows_ * n_cols_
        std::vector<double> values_; ///< underlying container for elements of the Matrix

    };

} // namespace meii


#endif // !MEII_MATRIX_HPP
