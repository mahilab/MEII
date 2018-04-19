#include <MEII/Utility/Matrix.hpp>
#include <MEL/Logging/Log.hpp>

using namespace mel;

namespace meii {

    Matrix::Matrix() :
        n_rows_(0),
        n_cols_(0),
        n_tot_(0),
        values_(std::vector<double>()) 
    {}

    Matrix::Matrix(std::size_t n_rows_n_cols) :
        n_rows_(n_rows_n_cols),
        n_cols_(n_rows_n_cols),
        n_tot_(n_rows_n_cols * n_rows_n_cols),
        values_(n_rows_n_cols * n_rows_n_cols, 0.0)
    {}

    Matrix::Matrix(std::size_t n_rows, std::size_t n_cols, double value) :
        n_rows_(n_rows),
        n_cols_(n_cols),
        n_tot_(n_rows * n_cols),
        values_(n_rows * n_cols, value)
    {}

    Matrix::Matrix(std::vector<double> vec) :
        n_rows_(vec.size()),
        n_cols_(1),
        n_tot_(vec.size()),
        values_(vec)
    {}

    const double& Matrix::operator()(std::size_t row_idx, std::size_t col_idx) const {
        return values_[row_idx * n_cols_ + col_idx];
    }

    const double& Matrix::operator()(std::size_t idx) const {
        return values_[idx];
    }

    double& Matrix::operator()(std::size_t row_idx, std::size_t col_idx) {
        return values_[row_idx * n_cols_ + col_idx];
    }

    double& Matrix::operator()(std::size_t idx) {
        return values_[idx];
    }

    bool Matrix::empty() const {
        return n_tot_ == 0;
    }

    std::vector<std::size_t> Matrix::size() const {
        std::vector<std::size_t> sizes(2);
        sizes[0] = n_rows_;
        sizes[1] = n_cols_;
        return sizes;
    }

    std::size_t Matrix::rows() const {
        return n_rows_;
    }

    std::size_t Matrix::cols() const {
        return n_cols_;
    }

    Matrix Matrix::transpose() const {
        Matrix transp_mat(n_cols_, n_rows_);
        for (std::size_t i = 0; i < n_rows_; ++i) {
            for (std::size_t j = 0; j < n_cols_; ++j) {
                transp_mat(j, i) = this->operator()(i, j);
            }
        }
        return transp_mat;
    }

    void Matrix::resize(std::size_t n_rows_n_cols) {
        n_rows_ = n_rows_n_cols;
        n_cols_ = n_rows_n_cols;
        n_tot_ = n_rows_n_cols * n_rows_n_cols;
        values_.resize(n_rows_n_cols * n_rows_n_cols);
    }

    void Matrix::resize(std::size_t n_rows, std::size_t n_cols) {
        clear();
        n_rows_ = n_rows;
        n_cols_ = n_cols;
        n_tot_ = n_rows * n_cols;
        values_.resize(n_rows * n_cols);
    }

	std::vector<double> Matrix::get_row(std::size_t row_idx) {
		std::vector<double> row(n_cols_);
		for (std::size_t i = 0; i < n_cols_; ++i) {
			row[i] = values_[row_idx * n_cols_ + i];
		}
		return row;
	}

	std::vector<double> Matrix::get_col(std::size_t col_idx) {
		std::vector<double> col(n_rows_);
		for (std::size_t i = 0; i < n_rows_; ++i) {
			col[i] = values_[i * n_cols_ + col_idx];
		}
		return col;
	}

    void Matrix::clear() {
        n_rows_ = 0;
        n_cols_ = 0;
        n_tot_ = 0;
        values_.clear();
    }

    Matrix Matrix::operator+=(double scalar) {
        for (std::size_t i = 0; i < n_tot_; i++) {
            values_[i] += scalar;
        }
        return *this;
    }


    Matrix Matrix::operator+(double scalar) const {
        auto result = *this;
        return result += scalar;
    }


    Matrix Matrix::operator-=(double scalar) {
        for (std::size_t i = 0; i < n_tot_; i++) {
            values_[i] -= scalar;
        }
        return *this;
    }


    Matrix Matrix::operator-(double scalar) const {
        auto result = *this;
        return result -= scalar;
    }


    Matrix Matrix::operator*=(double scalar) {
        for (std::size_t i = 0; i < n_tot_; i++) {
            values_[i] *= scalar;
        }
        return *this;
    }


    Matrix Matrix::operator*(double scalar) const {
        auto result = *this;
        return result *= scalar;
    }


    Matrix Matrix::operator+=(const Matrix& rhs) {
        if (n_rows_ != rhs.n_rows_ || n_cols_ != rhs.n_cols_) {
            LOG(Error) << "Matrix addition can only be performed on matrices with equal dimensions. Returning original LHS matrix.";
            return *this;
        }
        for (std::size_t i = 0; i < n_tot_; i++) {
            values_[i] += rhs.values_[i];
        }
        return *this;
    }


    Matrix Matrix::operator+(const Matrix& rhs) const {
        auto result = *this;
        return result += rhs;
    }


    Matrix Matrix::operator-=(const Matrix& rhs) {
        if (n_rows_ != rhs.n_rows_ || n_cols_ != rhs.n_cols_) {
            LOG(Error) << "Matrix subtraction can only be performed on matrices with equal dimensions. Returning original LHS matrix.";
            return *this;
        }
        for (std::size_t i = 0; i < n_tot_; i++) {
            values_[i] -= rhs.values_[i];
        }
        return *this;
    }


    Matrix Matrix::operator-(const Matrix& rhs) const {
        auto result = *this;
        return result -= rhs;
    }


    Matrix Matrix::operator*=(const Matrix& rhs) {
        if (n_cols_ != rhs.n_rows_) {
            LOG(Error) << "Matrix multiplication can only be performed on matrices with matching inner dimensions. Returning original LHS matrix.";
            return *this;
        }
        Matrix result(n_rows_, rhs.n_cols_, 0.0);
        for (std::size_t i = 0; i < n_rows_; ++i) {
            for (std::size_t j = 0; j < rhs.n_cols_; ++j) {
                for (std::size_t k = 0; k < n_cols_; ++k) {
                    result(i, j) += values_[i * n_cols_ + k] * rhs.values_[k * rhs.n_cols_ + j];
                }
            }
        }
        return (*this) = result;
    }


    Matrix Matrix::operator*(const Matrix& rhs) const {
        auto result = *this;
        return result *= rhs;
    }


    std::vector<double> Matrix::operator*(const std::vector<double>& vec) const {
        if (n_cols_ != vec.size()) {
            LOG(Error) << "Matrix multiplication can only be performed on vectors with matching inner dimensions. Returning original RHS vector.";
            return vec;
        }
        std::vector<double> result(n_rows_, 0.0);
        for (std::size_t i = 0; i < n_rows_; ++i) {
            for (std::size_t j = 0; j < n_cols_; ++j) {
                result[i] += values_[i * n_cols_ + j] * vec[j];
            }
        }
        return result;
    }

    
    std::ostream& operator<<(std::ostream& os, const Matrix& mat) {
        os << "[ ";
        for (std::size_t i = 0; i < mat.n_tot_; ++i) {
            if (i != 0 && i % mat.n_cols_ == 0) {
                os << "\n  ";
            }
            os << mat.values_[i] << " ";
        }
        os << "] " << std::endl;
        return os;
    }
}