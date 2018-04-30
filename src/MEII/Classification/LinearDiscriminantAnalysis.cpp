#include <MEII/Classification/LinearDiscriminantAnalysis.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEII/Utility/EigenConversions.hpp>
#include <MEL/Logging/Log.hpp>

using namespace mel;

namespace meii {

    double softmax(const std::vector<double>& a, std::size_t k) {
        if (k >= a.size()) {
            print("ERROR: Function softmax received input index k outside of bounds of input vector a.");
            return NAN;
        }
        std::vector<double> b(a.size());
        double c = min(a);
        for (std::size_t i = 0; i < b.size(); ++i) {
            b[i] = a[i] - c;
        }

        for (std::size_t i = 0; i < b.size(); ++i) {
            b[i] = std::exp(b[i]);
        }
        if (std::all_of(b.begin(), b.end(), [](double d) { return std::isfinite(d); }) && sum(b) != 0) {
            return b[k] / sum(b);
        }
        else {
            if (a[k] == max(a)) {
                return 1.0;
            }
            else {
                return 0.0;
            }
        }
    }



    bool bin_linear_discriminant_model(const std::vector<std::vector<double>>& class_0_data, const std::vector<std::vector<double>>& class_1_data, std::vector<double>& w, double& w_0, double max_reg) {

        // data dimension
        std::size_t N_0 = class_0_data.size();
        std::size_t N_1 = class_1_data.size();
        std::size_t D;
        if (N_0 > 0 && N_1 > 0) {
            D = class_0_data[0].size();
            for (std::size_t i = 0; i < N_0; ++i) {
                if (class_0_data[i].size() != D) {
                    LOG(Warning) << "Data given to bin_linear_discriminant_model() contains samples of different sizes. Model was not computed.";
                    return false;
                }
            }
            for (std::size_t i = 0; i < N_1; ++i) {
                if (class_1_data[i].size() != D) {
                    LOG(Warning) << "Data given to bin_linear_discriminant_model() contains samples of different sizes. Model was not computed.";
                    return false;
                }
            }
        }
        else {
            LOG(Warning) << "Some of the data given to bin_linear_discriminant_model() was empty. Model was not computed.";
            return false;
        }

		// compute individual class sample means and covariance matrices
		std::vector<double> class_0_sample_mean;
		std::vector<std::vector<double>> class_0_sample_cov;	
		gauss_mlt_params(class_0_data, class_0_sample_mean, class_0_sample_cov);
		std::vector<double> class_1_sample_mean;
		std::vector<std::vector<double>> class_1_sample_cov;
		gauss_mlt_params(class_1_data, class_1_sample_mean, class_1_sample_cov);
		
		// compute the weighted single covariance matrix
		double n_0 = (double)N_0;
		double n_1 = (double)N_1;
		std::vector<std::vector<double>> sample_cov(D, std::vector<double>(D, 0.0));
		for (std::size_t i = 0; i < D; i++) {
			for (std::size_t j = 0; j < D; j++) {
				sample_cov[i][j] += (n_0 / (n_0 + n_1)) * class_0_sample_cov[i][j] + (n_1 / (n_0 + n_1)) * class_1_sample_cov[i][j];
			}
		}


        //Eigen::MatrixXd sample_cov = Eigen::MatrixXd::Zero(D, D);
        //Eigen::MatrixXd sample_cov_inv(D, D);
        //Eigen::VectorXd w_eig(D);
        //Eigen::VectorXd w_0_eig(1);
        //w = std::vector<double>(D);

        //std::vector<double> class_0_sample_mean;
        //std::vector<std::vector<double>> class_0_sample_cov;
        //gauss_mlt_params(class_0_data, class_0_sample_mean, class_0_sample_cov);
        //Eigen::VectorXd class_0_sample_mean_eig = copy_stdvec_to_eigvec(class_0_sample_mean);
        //Eigen::MatrixXd class_0_sample_cov_eig = copy_stdvecvec_to_eigmat(class_0_sample_cov);

        //std::vector<double> class_1_sample_mean;
        //std::vector<std::vector<double>> class_1_sample_cov;
        //gauss_mlt_params(class_1_data, class_1_sample_mean, class_1_sample_cov);
        //Eigen::VectorXd class_1_sample_mean_eig = copy_stdvec_to_eigvec(class_1_sample_mean);
        //Eigen::MatrixXd class_1_sample_cov_eig = copy_stdvecvec_to_eigmat(class_1_sample_cov);

        

        //sample_cov = (n_0 / (n_0 + n_1)) * class_0_sample_cov_eig + (n_1 / (n_0 + n_1)) * class_1_sample_cov_eig;
        /*if (sample_cov.fullPivLu().isInvertible())
            sample_cov_inv = sample_cov.fullPivLu().inverse();
        else {
            LOG(Error) << "Sample covariance matrix cannot be inverted.";
			std::cout << sample_cov;
            return false;
        }*/


		// convert to Eigen types for computing linear model
		Eigen::VectorXd class_0_sample_mean_eig = copy_stdvec_to_eigvec(class_0_sample_mean);
		Eigen::VectorXd class_1_sample_mean_eig = copy_stdvec_to_eigvec(class_1_sample_mean);
		Eigen::MatrixXd sample_cov_eig = copy_stdvecvec_to_eigmat(sample_cov);
		Eigen::VectorXd w_eig(D);
		Eigen::VectorXd w_0_eig(1);	

		// apply regularization if needed
		double reg_increment = 0.00001;
		std::size_t max_reg_count = (std::size_t)((unsigned)(std::abs(max_reg) / reg_increment));
		std::size_t reg_count = 0;
		if (!sample_cov_eig.fullPivLu().isInvertible()) {
			if (max_reg_count > 0) {
				LOG(Warning) << "Sample covariance matrix cannot be inverted. Applying regularization.";
				while (!sample_cov_eig.fullPivLu().isInvertible() && reg_count < max_reg_count) {
					for (std::size_t i = 0; i < D; ++i) {
						sample_cov_eig(i, i) += reg_increment;
					}
					reg_count++;
				}
			}
			if (reg_count == max_reg_count) {
				LOG(Warning) << "Sample covariance matrix cannot be inverted. Printing covariance matrix and aborting model computation.";
				std::cout << sample_cov_eig << std::endl << std::endl;
				return false;
			}
		}
		Eigen::MatrixXd sample_cov_inv_eig = sample_cov_eig.fullPivLu().inverse();

        // compute linear model
        w_eig = sample_cov_inv_eig * (class_1_sample_mean_eig - class_0_sample_mean_eig);
        w_0_eig = -0.5 * class_1_sample_mean_eig.transpose() * sample_cov_inv_eig * class_1_sample_mean_eig + 0.5 * class_0_sample_mean_eig.transpose() * sample_cov_inv_eig * class_0_sample_mean_eig;
		
		// convert back from eigen types
		w = copy_eigvec_to_stdvec(w_eig);
        w_0 = w_0_eig[0];

        return true;
    }

    bool multi_linear_discriminant_model(const std::vector<std::vector<std::vector<double>>>& all_class_data, std::vector<std::vector<double>>& w, std::vector<double>& w_0, double r_min) {

        // data dimensions
        if (all_class_data.empty()) {
            LOG(Warning) << "Training data given to multi_linear_discriminant_model() is empty. Model computation aborted.";
            return false;
        }

        std::size_t K = all_class_data.size(); // number of classes
        std::vector<std::size_t> N(K); // number of observations per class
        std::size_t N_total = 0; // total number of observations
        std::vector<double> r(K); // observation number ratios
        for (std::size_t k = 0; k < K; ++k) {
            N[k] = all_class_data[k].size();
            N_total += N[k];
            if (N[k] == 0) {
                LOG(Warning) << "Training data given to multi_linear_discriminant_model() is empty for at least one class. Model computation aborted.";
                return false;
            }
        }
        std::size_t D = all_class_data[0][0].size(); // size of observation samples
        for (std::size_t k = 0; k < K; ++k) {
            r[k] = (double)N[k] / (double)N_total;
            for (std::size_t n = 0; n < N[k]; ++n) {
                if (all_class_data[k][n].size() != D) {
                    LOG(Error) << "Training data given to multi_linear_discriminant_model contains observations of different sizes. Model computation aborted.";
                    return false;
                }
            }
        }

        // compute individual class sample means and covariance matrices, as well as weighted single covariance matrix
        std::vector<std::vector<double>> sample_means(K);
        std::vector<std::vector<std::vector<double>>> sample_covs(K);
        std::vector<std::vector<double>> sample_cov(D, std::vector<double>(D, 0.0));
        for (std::size_t k = 0; k < K; ++k) {
            gauss_mlt_params(all_class_data[k], sample_means[k], sample_covs[k]);
            for (std::size_t i = 0; i < D; i++) {
                for (std::size_t j = 0; j < D; j++) {
                    sample_cov[i][j] += r[k] * sample_covs[k][i][j];
                }
            }
        }

        // convert to Eigen types for computing matrix inversions, then convert back
        Eigen::VectorXd sample_mean_eig;
        Eigen::MatrixXd sample_cov_eig;
        Eigen::VectorXd w_row_eig;
        Eigen::VectorXd w_0_eig(K);
        //Eigen::VectorXd w_0_intermediate_eig;
        sample_cov_eig = copy_stdvecvec_to_eigmat(sample_cov);
		double reg_increment = -20;
        //std::size_t max_reg_count = (std::size_t)((unsigned)(std::abs(max_reg) / reg_increment));
		std::size_t max_reg_count = 300;
        std::size_t reg_count = 0;
    //    if (!sample_cov_eig.fullPivLu().isInvertible()) {
    //        if (max_reg_count > 0) {
    //            LOG(Warning) << "Sample covariance matrix cannot be inverted. Applying regularization.";
    //            while (!sample_cov_eig.fullPivLu().isInvertible() && reg_count < max_reg_count) {
    //                for (std::size_t i = 0; i < D; ++i) {
    //                    sample_cov_eig(i, i) += reg_increment;
    //                }
    //                reg_count++;
    //            }
				//LOG(Warning) << "Regularization coefficient used was " << reg_count * reg_increment;
    //        }
    //        if (reg_count == max_reg_count) {
    //            LOG(Warning) << "Sample covariance matrix cannot be inverted. Printing covariance matrix and aborting model computation.";
    //            std::cout << sample_cov_eig << std::endl << std::endl;
    //            return false;
    //        }
    //    }
		if (sample_cov_eig.fullPivLu().rcond() < r_min) {
			if (max_reg_count > 0) {
				LOG(Warning) << "Sample covariance matrix is ill-conditioned. Applying regularization.";
				while (sample_cov_eig.fullPivLu().rcond() < r_min && reg_count < max_reg_count) {
					for (std::size_t i = 0; i < D; ++i) {
						sample_cov_eig(i, i) += std::exp(reg_increment);					
					}
					reg_increment += 0.1;
					reg_count++;
				}
			}
			if (reg_count == max_reg_count) {
				LOG(Warning) << "Sample covariance matrix cannot be inverted. Printing covariance matrix and aborting model computation.";
				std::cout << sample_cov_eig << std::endl << std::endl;
				return false;
			}
		}
        w.resize(K);
        for (std::size_t k = 0; k < K; ++k) {
            sample_mean_eig = copy_stdvec_to_eigvec(sample_means[k]);
            w_row_eig = sample_cov_eig.fullPivLu().solve(sample_mean_eig);
            //w_0_intermediate_eig = sample_cov_eig.fullPivLu().solve(sample_mean_eig);
            //w_0_eig[k] = -0.5 * sample_mean_eig.dot(w_0_intermediate_eig);
			w_0_eig[k] = -0.5 * sample_mean_eig.dot(w_row_eig);
            w[k] = copy_eigvec_to_stdvec(w_row_eig);
        }
        w_0 = copy_eigvec_to_stdvec(w_0_eig);

        return true;
    }

} // namespace meii