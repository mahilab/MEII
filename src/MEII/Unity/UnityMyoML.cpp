#include <MEII/Unity/UnityMyoML.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEL/Logging/Log.hpp>

using namespace mel;

namespace meii {

	UnityMyoML::UnityMyoML() :
		game_(game_name_, game_path_),
		arm_(Left),
		dof_(ElbowFE),
		num_classes_((std::size_t)2),
		target_count_((std::size_t)8),
		targets_(target_count_, 0.0),
		arrows_(target_count_, 0.0),
		target_label_(-1),
		viz_target_num_(-1),
		arrow_scale_(0.0),
		effort_min_(0.0),
		effort_max_(1.0),
		ms_targets_("targets"),
		ms_center_("center"),
		ms_arrows_("arrows"),
		ms_ring_("ring"),
		ms_text_("text")
	{
		viz_target_mapping_ = { { { 3, 7 },{ 1, 5 },{ 1, 5 },{ 3, 7 },{ 1, 5, 3, 2, 4, 7, 8, 6 },{ 3, 7, 1, 2, 8, 5, 4, 6 } },{ { 3, 7 },{ 5, 1 },{ 5, 1 },{ 3, 7 },{ 5, 1, 3, 4, 2, 7, 6, 8 },{ 3, 7, 5, 4, 6, 1, 2, 8 } } };
	}


	void UnityMyoML::launch() {
		game_.launch();
	}

	void UnityMyoML::set_experiment_conditions(Arm arm, DoF dof) {

		arm_ = arm;
		dof_ = dof;

		if (is_single_dof(dof_)) {
			num_classes_ = 2;
		}
		else {
			num_classes_ = 8;
		}

		targets_ = std::vector<double>(target_count_, 1.0);
		ms_center_.write_data({ 1.0 });
		ms_targets_.write_data(targets_);
		ms_arrows_.write_data(arrows_);
		ms_text_.write_message(dof_str_[dof]);
	}

	void UnityMyoML::set_target(std::size_t target_label) {
		if (viz_target_num_ > 0) {
			targets_[viz_target_num_ - 1] = 1.0;			
		}
		target_label_ = target_label;
		if (target_label_ == 0) {
			viz_target_num_ = 0;
			ms_center_.write_data({ 2.0 });
		}
		else {
			viz_target_num_ = compute_viz_target_num(target_label_);
			targets_[viz_target_num_ - 1] = 2.0;
			ms_center_.write_data({ 1.0 });
		}
		ms_targets_.write_data(targets_);
	}


	std::size_t UnityMyoML::get_target_label() const {
		return target_label_;
	}

	int UnityMyoML::get_viz_target_num() const {
		return viz_target_num_;
	}

	int UnityMyoML::compute_viz_target_num(std::size_t target_label) const {
		if (target_label == 0) {
			return 0;
		}
		if (target_label < 0 || target_label > num_classes_) {
			return -1;
		}
		return viz_target_mapping_[arm_][dof_][target_label - 1];
	}

} // namespace meii