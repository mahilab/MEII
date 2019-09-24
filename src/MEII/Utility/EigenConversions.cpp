#include <MEII/Utility/EigenConversions.hpp>
#include <MEL/Logging/Log.hpp>

using namespace mel;

namespace meii {

    void eigvec_to_stdvec(const Eigen::VectorXd& eigen_vec, std::vector<double>& std_vec) {
        std_vec.resize(eigen_vec.size());
        Eigen::VectorXd::Map(&std_vec[0], eigen_vec.size()) = eigen_vec;
    }

    void stdvec_to_eigvec(std::vector<double>& std_vec, Eigen::VectorXd& eigen_vec) {
        eigen_vec = Eigen::Map<Eigen::VectorXd>(std_vec.data(), std_vec.size());
    }

    std::vector<double> copy_eigvec_to_stdvec(const Eigen::VectorXd& eigen_vec) {
        std::vector<double> std_vec(eigen_vec.size());
        for (int i = 0; i < eigen_vec.size(); ++i) {
            std_vec[i] = eigen_vec[i];
        }
        return std_vec;
    }

    Eigen::VectorXd copy_stdvec_to_eigvec(const std::vector<double>& std_vec) {
        Eigen::VectorXd eigen_vec(std_vec.size());
        for (size_t i = 0; i < std_vec.size(); ++i) {
            eigen_vec[i] = std_vec[i];
        }
        return eigen_vec;
    }

    std::vector<std::vector<double>> copy_eigmat_to_stdvecvec(const Eigen::MatrixXd& eigen_mat) {
        std::vector<std::vector<double>> std_vecvec(eigen_mat.rows());
        for (int i = 0; i < eigen_mat.rows(); ++i) {
            std_vecvec[i] = std::vector<double>(eigen_mat.cols());
            for (int j = 0; j < eigen_mat.cols(); ++j) {
                std_vecvec[i][j] = eigen_mat(i, j);
            }
        }
        return std_vecvec;
    }

    Eigen::MatrixXd copy_stdvecvec_to_eigmat(const std::vector<std::vector<double>>& std_vecvec) {
        size_t cols = std_vecvec[0].size();
        Eigen::MatrixXd eigen_mat(std_vecvec.size(), cols);
        for (size_t i = 0; i < std_vecvec.size(); ++i) {
            if (std_vecvec[i].size() == cols) {
                for (size_t j = 0; j < cols; ++j) {
                    eigen_mat(i, j) = std_vecvec[i][j];
                }
            }
            else {
                LOG(Error) << "Input must have same number of cols in each row to be converted into Eigen Matrix type";
            }
        }
        return eigen_mat;
    }

    double mat_spectral_norm(const Eigen::MatrixXd& mat) {
        Eigen::EigenSolver<Eigen::MatrixXd> eigensolver(mat.transpose() * mat, false);
        if (eigensolver.info() != Eigen::Success) {
            print("ERROR: Eigensolver did not converge in mat_spectral_norm");
            return 0;
        }
        Eigen::EigenSolver<Eigen::MatrixXd>::EigenvalueType lambda = eigensolver.eigenvalues();
        std::vector<double> lambda_abs(lambda.size(), 0.0);
        for (int i = 0; i < lambda.size(); ++i) {
            lambda_abs[i] = std::abs(lambda[i]);
        }
        std::vector<double>::iterator lambda_max;
        lambda_max = std::max_element(lambda_abs.begin(), lambda_abs.end());
        return std::sqrt(*lambda_max);
    }

} // namespace meii