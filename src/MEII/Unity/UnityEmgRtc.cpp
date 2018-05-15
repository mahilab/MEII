#include <MEII/Unity/UnityEmgRtc.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEL/Logging/Log.hpp>

using namespace mel;

namespace meii {

    UnityEmgRtc::UnityEmgRtc() :
		game_(game_name_, game_path_),
        arm_(Left),
        dof_(ElbowFE),
        num_classes_(2),
        phase_(Calibration),
		target_count_(8),
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
        viz_target_mapping_ = { { {3, 7}, {1, 5}, {1, 5}, {3, 7}, {2, 4, 8, 6}, {2, 4, 8, 6} }, { { 3, 7 },{ 5, 1 },{ 5, 1 },{ 3, 7 },{ 4, 2, 6, 8 },{ 4, 2, 6, 8 } } };
    }


    void UnityEmgRtc::launch() {
        game_.launch();
    }

    void UnityEmgRtc::set_experiment_conditions(Arm arm, DoF dof, Phase phase) {

		arm_ = arm;
		dof_ = dof;
		phase_ = phase;

		if (is_single_dof(dof_)) {
			num_classes_ = 2;
		}
		else {
			num_classes_ = 4;
		}

		targets_ = std::vector<double>(target_count_, 0.0);
		switch (dof) {
		case ElbowFE:
			targets_[2] = 1.0;
			targets_[6] = 1.0;
			break;
		case WristPS:
			targets_[0] = 1.0;
			targets_[4] = 1.0;
			break;
		case WristFE:
			targets_[0] = 1.0;
			targets_[4] = 1.0;
			break;
		case WristRU:
			targets_[2] = 1.0;
			targets_[6] = 1.0;
			break;
		case ElbowFE_WristPS:
			targets_[1] = 1.0;
			targets_[3] = 1.0;
			targets_[5] = 1.0;
			targets_[7] = 1.0;
			break;
		case WristFE_WristRU:
			targets_[1] = 1.0;
			targets_[3] = 1.0;
			targets_[5] = 1.0;
			targets_[7] = 1.0;
			break;
		}
		ms_center_.write_data({ 1.0 });
		ms_targets_.write_data(targets_);
		ms_text_.write_message(dof_str_[dof]);
    }

    void UnityEmgRtc::set_target(int target_label) {
		if (viz_target_num_ > 0) {
			targets_[viz_target_num_ - 1] = 1.0;
			arrows_[viz_target_num_ - 1] = 0.0;
		}
		target_label_ = target_label;
		viz_target_num_ = compute_viz_target_num(target_label_);
		if (viz_target_num_ > 0) {
			targets_[viz_target_num_ - 1] = 2.0;
			arrows_[viz_target_num_ - 1] = arrow_scale_;
		}
		ms_targets_.write_data(targets_);
		ms_arrows_.write_data(arrows_);
    }

	void UnityEmgRtc::set_center(bool center_glow) {
		if (center_glow) {
			ms_center_.write_data({ 3.0 });
		}
		else {
			ms_center_.write_data({ 1.0 });
		}
	}

    void UnityEmgRtc::set_effort(double effort) {
        arrow_scale_ = (saturate(effort, effort_min_, effort_max_) - effort_min_) / (effort_max_ - effort_min_);
		arrows_[viz_target_num_ - 1] = arrow_scale_;
        ms_arrows_.write_data(arrows_);
    }

    void UnityEmgRtc::set_effort_range(double effort_min, double effort_max) {
        effort_min_ = effort_min;
        effort_max_ = effort_max;
    }

	void UnityEmgRtc::set_ring(bool visible) {
		if (visible) {
			ms_ring_.write_data({ 1.0 });
		}
		else {
			ms_ring_.write_data({ 0.0 });
		}
	}

	int UnityEmgRtc::get_target_label() const {
        return target_label_;
    }

	int UnityEmgRtc::get_viz_target_num() const {
        return viz_target_num_;
    }

	int UnityEmgRtc::compute_viz_target_num(int target_label) const {
		if (target_label == 0) {
			return 0;
		}
		if (target_label < 0 || target_label > num_classes_) {
			return -1;
		}
        return viz_target_mapping_[arm_][dof_][target_label - 1];
    }

} // namespace meii