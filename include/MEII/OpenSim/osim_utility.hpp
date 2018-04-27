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

#ifndef MEII_OSIM_UTILITY_HPP
#define MEII_OSIM_UTILITY_HPP

#include <MEL/Logging/Table.hpp>
#include <map>

namespace meii {

	class MeiiOsimMotTable : public mel::Table {
	public:
		/// Constructor
		MeiiOsimMotTable(const std::string &name = "MeiiOsimMot");

		bool set_coordinate_defaults(const std::string &coordinate_name, double default_value);

		friend bool write_meii_to_osim_mot(MeiiOsimMotTable data, const std::string &filename, const std::string& directory, bool timestamp);

	private:

		std::vector<std::string> coordinate_list_; ///< vector of coordinate names in the MEII OpenSim model
		std::vector<double> coordinate_defaults_; ///< vector of default values used for the coordinates in the MEII OpenSim model
		std::map<std::string, std::size_t> coordinate_map_;

	};

	class MeiiOsimStoTable : public mel::Table {
	public:
		/// Constructor
		MeiiOsimStoTable(const std::string &name = "MeiiOsimSto");

		friend bool write_meii_to_osim_sto(MeiiOsimStoTable data, const std::string &filename, const std::string& directory, bool timestamp);

	private:

		std::vector<std::string> coordinate_actuator_list_;

	};

	bool write_to_osim_mot(const mel::Table &data, const std::string &filename = "", const std::string& directory = ".", bool in_degrees = true, bool timestamp = true);

	bool write_meii_to_osim_mot(MeiiOsimMotTable data, const std::string &filename = "", const std::string& directory = ".", bool timestamp = true);

	std::string make_osim_mot_header(const mel::Table &table, bool in_degrees = true);

	bool write_to_osim_sto(const mel::Table &data, const std::string &filename = "", const std::string& directory = ".", bool in_degrees = true, bool timestamp = true);

	bool write_meii_to_osim_sto(MeiiOsimStoTable data, const std::string &filename = "", const std::string& directory = ".", bool timestamp = true);

	std::string make_osim_sto_header(const mel::Table &table, bool in_degrees = false);

} // namespace meii

#endif() // MEII_OSIM_UTILITY_HPP