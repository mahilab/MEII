#include <MEII/Unity/UnityEmgRtc.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEL/Logging/Log.hpp>

using namespace mel;

namespace meii {

    UnityEmgRtc::UnityEmgRtc() :
        variables_set_(false),
        hand_num_(0),
        dof_(0),
        num_classes_(0),
        condition_(0),
        target_label_(-1),
        viz_target_num_(-1),
        scene_num_(1, 0.0),
        scene_int_(0),
        effort_min_(0.0),
        effort_max_(1.0),
        ms_scene_("melshare_scene"),
        ms_target_("melshare_target"),
		ms_center_("melshare_center"),
        ms_effort_("melshare_effort")
    {
        viz_target_mapping_ = { { {3, 7}, {1, 5}, {1, 5}, {3, 7}, {2, 4, 8, 6}, {2, 4, 8, 6} }, { { 3, 7 },{ 5, 1 },{ 5, 1 },{ 3, 7 },{ 4, 2, 6, 8 },{ 4, 2, 6, 8 } } };
    }


    void UnityEmgRtc::launch() {
        game.launch();
    }

    void UnityEmgRtc::set_experiment_conditions(std::size_t hand, std::size_t& dof, std::size_t& num_classes, std::size_t& condition, bool& menu) {
        scene_num_ = ms_scene_.read_data();
        if (scene_num_.empty())
            return;
        scene_int_ = (int)scene_num_[0];
        if (scene_int_ == 0) {
            menu = true;
            hand_num_ = hand;
        }
        else if (scene_int_ > 0) {
            menu = false;
            hand_num_ = hand;
            dof_ = (std::size_t)((unsigned)((scene_int_ - 1) / 4));
            condition_ = (std::size_t)((unsigned)((scene_int_ - 1) % 4));
            if (dof_ < 4) {
                num_classes_ = 2;
            }
            else {
                num_classes_ = 4;
            }
            dof = dof_;
            condition = condition_;
            num_classes = num_classes_;
        }
        variables_set_ = true;
    }

    void UnityEmgRtc::set_target(int target_label) {
        if (!variables_set_)
            return;
        target_label_ = target_label;
        viz_target_num_ = compute_viz_target_num(target_label_);
        ms_target_.write_data({ (double)viz_target_num_ });
    }

	void UnityEmgRtc::set_center(bool center_glow) {
		if (center_glow) {
			ms_center_.write_data({ 1.0 });
		}
		else {
			ms_center_.write_data({ 0.0 });
		}
	}

    void UnityEmgRtc::set_effort(double effort) {
        effort = (saturate(effort, effort_min_, effort_max_) - effort_min_) / (effort_max_ - effort_min_);
        ms_effort_.write_data({ effort });
    }

    void UnityEmgRtc::set_effort_range(double effort_min, double effort_max) {
        effort_min_ = effort_min;
        effort_max_ = effort_max;
    }

	int UnityEmgRtc::get_target_label() const {
        return target_label_;
    }

	int UnityEmgRtc::get_viz_target_num() const {
        return viz_target_num_;
    }

	int UnityEmgRtc::compute_viz_target_num(int target_label) const {
        if (target_label == 0)
            return 0;
        if (target_label < 0 || target_label > num_classes_)
            return -1;
        if (hand_num_ < 0 || hand_num_ > 1)
            return -1;
        if (dof_ < 0 || dof_ > 5)
            return -1;
        return viz_target_mapping_[hand_num_][dof_][target_label - 1];
    }

} // namespace meii