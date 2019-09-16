#include <MEII/Utility/logging_util.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Core/Console.hpp>

using namespace mel;

namespace meii {

	MeiiTable::MeiiTable(const std::string &name) :
		Table(name)
	{
		push_back_col("time");
		for (std::size_t i = 0; i < robot_joint_str.size(); ++i) {
			push_back_col(robot_joint_str[i]);
		}
		for (std::size_t i = 0; i < robot_joint_str.size(); ++i) {
			push_back_col(robot_joint_str[i] + "_Vel");
		}
		for (std::size_t i = 0; i < robot_joint_str.size(); ++i) {
			push_back_col(robot_joint_str[i] + "_Trq");
		}
	}

	EmgTable::EmgTable(const std::string &name, std::vector<mel::uint32> emg_channel_numbers, bool raw, bool dm, bool env, bool tkeo_env) :
		Table(name)
	{
		push_back_col("time");
		if (raw) {
			for (std::size_t i = 0; i < emg_channel_numbers.size(); ++i) {
				push_back_col("MES_RAW_" + stringify(emg_channel_numbers[i]));
			}
		}
		if (dm) {
			for (std::size_t i = 0; i < emg_channel_numbers.size(); ++i) {
				push_back_col("MES_DM_" + stringify(emg_channel_numbers[i]));
			}
		}
		if (env) {
			for (std::size_t i = 0; i < emg_channel_numbers.size(); ++i) {
				push_back_col("MES_ENV_" + stringify(emg_channel_numbers[i]));
			}
		}
		if (tkeo_env) {
			for (std::size_t i = 0; i < emg_channel_numbers.size(); ++i) {
				push_back_col("MES_TKEO_ENV_" + stringify(emg_channel_numbers[i]));
			}
		}
	}

}