#include <MEII/OpenSim/osim_utility.hpp>
#include <MEL/Utility/System.hpp>
#include <MEL/Core/Timestamp.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Logging/DataLogger.hpp>

using namespace mel;

namespace meii {	

	bool write_to_osim_mot(const mel::Table &data, const std::string &filename, const std::string& directory, bool in_degrees, bool timestamp) {
		std::string new_filename = filename.empty() ? (data.name().empty() ? "osim_motion_file" : data.name()) : filename;
		std::string filename_no_ext;
		std::string file_ext;
		split_file_name(new_filename.c_str(), filename_no_ext, file_ext);
		if (file_ext.empty()) {
			file_ext = "mot";
		}
		std::string full_filename;
		if (timestamp) {
			Timestamp stamp;
			full_filename = directory + get_path_slash() + filename_no_ext + "_" + stamp.yyyy_mm_dd_hh_mm_ss() + "." + file_ext;
		}
		else {
			full_filename = directory + get_path_slash() + filename_no_ext + "." + file_ext;
		}
		LOG(Verbose) << "Saving data to " << full_filename;
		create_directory(directory);
		_unlink(full_filename.c_str());
		File file;
		file.open(full_filename.c_str());
		std::ostringstream oss;
		oss << std::setprecision(16);
		oss << make_osim_mot_header(data, in_degrees);
		if (!data.empty()) {
			for (std::size_t i = 0; i < data.col_count() - 1; i++) {
				oss << data.get_col_name(i) << "\t";
			}
			oss << data.get_col_name(data.col_count() - 1) << "\r\n";
			oss << std::endl;
			for (std::size_t i = 0; i < data.row_count(); i++) {
				for (size_t j = 0; j < data.col_count() - 1; ++j) {
					oss << data(i, j) << "\t";
				}
				oss << data(i, data.col_count() - 1) << "\r\n";
			}
		}
		file.write(oss.str());
		file.close();
		return true;
	}

	bool write_meii_to_osim_mot(MeiiOsimMotTable data, const std::string &filename, const std::string& directory, bool timestamp) {
		for (std::size_t i = 0; i < 5; i++) {
			data.insert_col(data.coordinate_list_[i], std::vector<double>(data.row_count(), data.coordinate_defaults_[i]), i + 1);
		}
		for (std::size_t i = 6; i < 7; i++) {
			data.insert_col(data.coordinate_list_[i], std::vector<double>(data.row_count(), data.coordinate_defaults_[i]), i + 1);
		}
		for (std::size_t i = 8; i < 11; i++) {
			data.insert_col(data.coordinate_list_[i], std::vector<double>(data.row_count(), data.coordinate_defaults_[i]), i + 1);
		}
		for (std::size_t i = 14; i < 18; i++) {
			data.insert_col(data.coordinate_list_[i], std::vector<double>(data.row_count(), data.coordinate_defaults_[i]), i + 1);
		}
		bool in_degrees = true;
		write_to_osim_mot(data, filename, directory, in_degrees, timestamp);
		return true;
	}

	std::string make_osim_mot_header(const Table &table, bool in_degrees) {
		std::vector<std::string> in_degrees_vals = {"no", "yes"};
		std::string in_degrees_str;
		if (in_degrees) {
			in_degrees_str = in_degrees_vals[1];
		}
		else {
			in_degrees_str = in_degrees_vals[0];
		}
		std::ostringstream oss;
		oss << "Results" << "\r\n";
		oss << "version" << "=" << "1" << "\r\n";
		oss << "nRows" << "=" << table.row_count() << "\r\n";
		oss << "nColumns" << "=" << table.col_count() << "\r\n";
		oss << "inDegrees" << "=" << in_degrees_str << "\r\n";
		oss << "endheader" << "\r\n";
		return oss.str();
	}

	bool write_to_osim_sto(const mel::Table &data, const std::string &filename, const std::string& directory, bool in_degrees, bool timestamp) {
		std::string new_filename = filename.empty() ? (data.name().empty() ? "osim_controls_file" : data.name()) : filename;
		std::string filename_no_ext;
		std::string file_ext;
		split_file_name(new_filename.c_str(), filename_no_ext, file_ext);
		if (file_ext.empty()) {
			file_ext = "sto";
		}
		std::string full_filename;
		if (timestamp) {
			Timestamp stamp;
			full_filename = directory + get_path_slash() + filename_no_ext + "_" + stamp.yyyy_mm_dd_hh_mm_ss() + "." + file_ext;
		}
		else {
			full_filename = directory + get_path_slash() + filename_no_ext + "." + file_ext;
		}
		LOG(Verbose) << "Saving data to " << full_filename;
		create_directory(directory);
		_unlink(full_filename.c_str());
		File file;
		file.open(full_filename.c_str());
		std::ostringstream oss;
		oss << std::setprecision(16);
		oss << make_osim_sto_header(data, in_degrees);
		if (!data.empty()) {
			for (std::size_t i = 0; i < data.col_count() - 1; i++) {
				oss << data.get_col_name(i) << "\t";
			}
			oss << data.get_col_name(data.col_count() - 1) << "\r\n";
			oss << "\r\n";
			for (std::size_t i = 0; i < data.row_count(); i++) {
				for (size_t j = 0; j < data.col_count() - 1; ++j) {
					oss << data(i, j) << "\t";
				}
				oss << data(i, data.col_count() - 1) << "\r\n";
			}
		}
		file.write(oss.str());
		file.close();
		return true;
	}

	bool write_meii_to_osim_sto(MeiiOsimStoTable data, const std::string &filename, const std::string& directory, bool timestamp) {
		bool in_degrees = false;
		write_to_osim_sto(data, filename, directory, in_degrees, timestamp);
		return true;
	}

	std::string make_osim_sto_header(const Table &table, bool in_degrees) {
		std::vector<std::string> in_degrees_vals = { "no", "yes" };
		std::string in_degrees_str;
		if (in_degrees) {
			in_degrees_str = in_degrees_vals[1];
		}
		else {
			in_degrees_str = in_degrees_vals[0];
		}
		std::ostringstream oss;
		oss << "controls" << "\r\n";
		oss << "version" << "=" << "1" << "\r\n";
		oss << "nRows" << "=" << table.row_count() << "\r\n";
		oss << "nColumns" << "=" << table.col_count() << "\r\n";
		oss << "inDegrees" << "=" << in_degrees_str << "\r\n";
		oss << "endheader" << "\r\n";
		return oss.str();
	}


	MeiiOsimMotTable::MeiiOsimMotTable(const std::string &name) :
		Table(name),
		coordinate_list_({ "exo_base_rail_rotation_y","exo_base_rail_translation_x","exo_base_rail_translation_z","exo_base_block_translation_y",
			"exo_upper_arm_rotation_x","exo_forearm_rotation_z","exo_counterweight_translation_x","exo_wrist_base_rotation_x",
			"exo_wrist_slider_1_rotation_z","exo_wrist_slider_2_rotation_z","exo_wrist_slider_3_rotation_z",
			"exo_wrist_rail_1_translation_x","exo_wrist_rail_2_translation_x","exo_wrist_rail_3_translation_x",
			"exo_wrist_ring_rotation_x","exo_wrist_ring_rotation_y","exo_wrist_ring_rotation_z",
			"exo_handle_translation_x" }),
		coordinate_defaults_({ 0.0, -0.178, 0.22, 0.29153, 0.0, 0.0, 0.0, 0.0, -23.298353937957266, -23.298353937957266, -23.298353937957266, 0.1305, 0.1305, 0.1305, -0.381533904858006, -0.164299800232324, 23.297806897499196, 0.04445 })
	{
		set_col_names({ "time", coordinate_list_[5], coordinate_list_[7], coordinate_list_[11], coordinate_list_[12], coordinate_list_[13] });
		for (std::size_t i = 0; i < coordinate_list_.size(); ++i) {
			coordinate_map_.insert(std::pair<std::string, std::size_t>(coordinate_list_[i], i));
		}
	}

	bool MeiiOsimMotTable::set_coordinate_defaults(const std::string &coordinate_name, double default_value) {
		if (coordinate_map_.find(coordinate_name) == coordinate_map_.end()) {
			LOG(Warning) << "Could not find MEII coordinate name given to set coordinate default value. Value not set.";
			return false;
		}
		std::size_t index = std::distance(coordinate_map_.begin(), coordinate_map_.find(coordinate_name));
		coordinate_defaults_[index] = default_value;
		LOG(Info) << "Coordinate " << coordinate_list_[index] << " default value set to " << coordinate_defaults_[index];
		return true;
	}

	MeiiOsimStoTable::MeiiOsimStoTable(const std::string &name) :
		Table(name),
		coordinate_actuator_list_({ 
			"exo_forearm_rotation_z_actuator", 
			"exo_wrist_base_rotation_x_actuator",
			"exo_wrist_rail_1_translation_x_actuator",
			"exo_wrist_rail_2_translation_x_actuator",
			"exo_wrist_rail_3_translation_x_actuator"})
	{
		set_col_names({ "time", coordinate_actuator_list_[0], coordinate_actuator_list_[1], coordinate_actuator_list_[2], coordinate_actuator_list_[3], coordinate_actuator_list_[4] });
	}

}