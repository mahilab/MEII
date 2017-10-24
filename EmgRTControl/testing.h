#pragma once
#include "mel_types.h"
#include <Eigen\Dense>
#include <Eigen\StdVector>
#include <boost/circular_buffer.hpp>

using namespace mel;


template <typename T> bool read_csv(std::string filename, std::string directory, std::vector<std::vector<T>>& output);
template <typename T> bool write_csv(std::string filename, std::string directory, const std::vector<std::vector<T>>& input);


