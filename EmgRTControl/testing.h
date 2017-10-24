#pragma once
#include "mel_types.h"
#include <Eigen\Dense>
#include <Eigen\StdVector>
#include <boost/circular_buffer.hpp>

using namespace mel;

void ml_covariance_estimate(const std::vector<double_vec>& sample_data, const Eigen::VectorXd& sample_data_mean, Eigen::MatrixXd& cov_mat);

template <typename T> bool read_csv(std::string filename, std::string directory, std::vector<std::vector<T>>& output);
template <typename T> bool write_csv(std::string filename, std::string directory, const std::vector<std::vector<T>>& input);

double softmax(const Eigen::VectorXd& a, int k);
