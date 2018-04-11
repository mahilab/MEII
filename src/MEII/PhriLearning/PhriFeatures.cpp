#include <MEII/PhriLearning/PhriFeatures.hpp>
#include <MEII/MahiExoII/MahiExoII.hpp>
#include <MEL/Logging/Log.hpp>

using namespace mel;

namespace meii {

    Matrix feature_extraction(const Matrix &q_d) {
        if (q_d.rows() != MahiExoII::N_aj_ || q_d.cols() != 1) {
            LOG(Warning) << "Input vector to q_d phri::feature_extraction() is not of expected size equal to number of MahiExoII anatomical joints. Returning empty vector.";
            return std::vector<double>();
        }

        // distance of Wrist PS joint from horizontal, positive (right arm pronated)
        double ps_horz_pos = 90 * DEG2RAD;
        double phi_0 = (q_d(1) - ps_horz_pos) * (q_d(1) - ps_horz_pos);
        return Matrix(1, 1, phi_0);

    }

    Matrix feature_jacobian(const Matrix &q_d) {
        if (q_d.rows() != MahiExoII::N_aj_, q_d.cols() != 1) {
            LOG(Warning) << "Input vector q_d to phri::feature_jacobian() is not of expected size equal to number of MahiExoII anatomical joints. Returning empty vector.";
            return Matrix();
        }
        Matrix jac(q_d.rows(), 1);
        double ps_horz_pos = 90 * DEG2RAD;
        for (std::size_t i = 0; i < jac.rows(); ++i) {
            if (i == 1) {
                jac(i, 0) = { 2.0 * (q_d(1) - ps_horz_pos) };
            }
            else {
                jac(i, 0) = { 0.0 };
            }
        }
        return jac;
    }

    Matrix feature_gradient(const Matrix &q_d, const Matrix &theta) {
        if (q_d.rows() != MahiExoII::N_aj_ || q_d.cols() != 1) {
            LOG(Warning) << "Input vector q_d to phri::feature_gradient() is not of expected size equal to number of MahiExoII anatomical joints. Returning empty vector.";
            return std::vector<double>();
        }
        if (theta.rows() != 1 || theta.cols() != 1) {
            LOG(Warning) << "Input vector theta to phri::feature_gradient() is not of expected size equal to number of features. Returning empty vector.";
            return std::vector<double>();
        }
        Matrix jac = feature_jacobian(q_d);
        Matrix jac_t = jac.transpose();
        return jac_t * theta;
    }

} // namespace meii