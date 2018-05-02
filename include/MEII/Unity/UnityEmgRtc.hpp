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

#ifndef MEII_UNITY_EMG_RTC_HPP
#define MEII_UNITY_EMG_RTC_HPP

#include <MEII/EmgRealTimeControl/EmgRealTimeControl.hpp>
#include <MEL/Utility/Windows/ExternalApp.hpp>
#include <MEL/Communications/Windows/MelShare.hpp>
#include <vector>

namespace meii {

    //==============================================================================
    // CLASS DECLARATION
    //==============================================================================

    class UnityEmgRtc {

    public:

        /// Constructor
        UnityEmgRtc();

        /// Launches the Unity game associated with the EmgRealTimeControl experiment.
        void launch();

        /// Read the scene number associated with the game and set the experimental conditions
        void set_experiment_conditions(Arm arm, DoF dof, Phase phase);

        /// Target labels correspond to class labels, but with 0 being the center target.
        void set_target(int target_label);

		/// Turns on the center target color and glow when given true, and turns off the center target when given false
		void set_center(bool center_glow);

        /// Set the effort level, which corresponds to the size of the arrow during certain conditions
        void set_effort(double effort);

        /// Set the range of effort values to be expected, so that the visualization may scale the arrow appropriately
        void set_effort_range(double effort_min, double effort_max);

		/// Turns on the ring when true
		void set_ring(bool visible);

        /// Return the last value set as the target label
		int get_target_label() const;

        /// Return the last value set as the visual target number
		int get_viz_target_num() const;

    private:

        /// Convert the target numbers based on an classification into the visualization's target numbering system
		int compute_viz_target_num(int target_label) const;

    private:

		std::string game_name_ = "emg_real_time_ctrl";
		std::string game_path_ = "C:\\Git\\MEII\\unity\\builds\\planar_targets.exe";
		mel::ExternalApp game_; ///< executable generated by Unity

	
		Arm arm_;
		DoF dof_;
		std::size_t num_classes_; ///< number of classes based on the dof, 2 for single, 4 for multi
		Phase phase_;

		std::size_t target_count_;
		std::vector<double> targets_;
		std::vector<double> arrows_;
		int target_label_;
		int viz_target_num_;
		double arrow_scale_;
        std::vector<std::vector<std::vector<int>>> viz_target_mapping_;
        double effort_min_;
        double effort_max_;
       
        mel::MelShare ms_targets_;
		mel::MelShare ms_center_;
        mel::MelShare ms_arrows_;
		mel::MelShare ms_ring_;
    };

} // namespace meii

#endif // MEII_UNITY_EMG_RTC_HPP