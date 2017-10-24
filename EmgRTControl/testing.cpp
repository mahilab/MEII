#include "testing.h"
#include "mel_util.h"
#include <iostream>
#include <fstream>
#include <string>
#include <boost/filesystem.hpp>

using namespace mel;

template <typename T>
bool read_csv(std::string filename, std::string directory, std::vector<std::vector<T>>& output) {
    output.clear();
    std::string full_filename = directory + "\\" + filename + ".csv";
    std::ifstream input(full_filename);
    input.precision(12);
    if (input.is_open()) {
        std::string csv_line;
        while (std::getline(input, csv_line)) {
            std::istringstream csv_stream(csv_line);
            std::vector<T> row;
            std::string number;
            T data;
            while (std::getline(csv_stream, number, ',')) {
                std::istringstream number_stream(number);
                number_stream >> data;
                row.push_back(data);
            }
            output.push_back(row);
        }
        return true;
    }
    else {
        util::print("ERROR: File not found.");
        return false;
    }
}
template bool read_csv<int>(std::string filename, std::string directory, std::vector<std::vector<int>>& output);
template bool read_csv<double>(std::string filename, std::string directory, std::vector<std::vector<double>>& output);

template <typename T>
bool write_csv(std::string filename, std::string directory, const std::vector<std::vector<T>>& input) {
    std::string full_filename = directory + "\\" + filename + ".csv";
    boost::filesystem::path dir(directory.c_str());
    boost::filesystem::create_directories(dir);
    std::ofstream output(full_filename);
    output.precision(12);
    if (output.is_open()) {
        for (int row = 0; row < input.size(); ++row) {
            for (int col = 0; col + 1 < input[row].size(); ++col) {
                output << input[row][col] << ",";
            }
            output << input[row][input[row].size() - 1] << std::endl;
        }
        output.close();
        return true;
    }
    else {
        util::print("ERROR: File not found.");
        return false;
    }
}
template bool write_csv<int>(std::string filename, std::string directory, const std::vector<std::vector<int>>& input);
template bool write_csv<double>(std::string filename, std::string directory, const std::vector<std::vector<double>>& input);


double softmax(const Eigen::VectorXd& a, int k) {
    if (k < 0 || k > a.size()) {
        util::print("ERROR: Function softmax received input index k outside of bounds of input vector a.");
        return NAN;
    }
    Eigen::VectorXd b(a.size());
    for (int i = 0; i < a.size(); ++i) {
        b(i) = std::exp(a(i));
    }
    
    if (b.allFinite()) {
        return b(k) / b.sum();
    }
    else {
        if (std::isinf(b(k))) {
            return 1.0;
        }
        else {
            return 0.0;
        }
    }
}

