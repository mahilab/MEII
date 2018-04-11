//#include <MEII/Utility/LoggingUtil.hpp>
//#include <MEL/Logging/DataLogger.hpp>
//
//using namespace mel;
//
//namespace meii {
//
//    bool log_vector(const std::vector<std::vector<double>> &data, const std::string &filename, const std::string &directory = ".", bool timestamp = true) {
//        if 
//        DataLogger datalog(WriterType::Buffered, false);
//        for (std::size_t i = 0; i < data.size(); ++i) {
//            datalog.buffer(data[i]);
//        }
//        datalog.save_data("ex_mes_array_captured_tkeo_datalog.csv", ".", false);
//        datalog.wait_for_save();
//    }
//
//    bool log_matrix(const Matrix &data, const std::string &filename, const std::string &directory = ".", bool timestamp = true) {
//
//    }
//
//}