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

#ifndef MEII_MES_ARRAY_HPP
#define MEII_MES_ARRAY_HPP

#include <MEII/EMG/MyoelectricSignal.hpp>
#include <MEL/Daq/Input.hpp>

namespace meii {

    //==============================================================================
    // CLASS DECLARATION
    //==============================================================================

    class MesArray {

    public:

        /// Constructor
        MesArray(std::vector<mel::AnalogInput::Channel> ai_channels = std::vector<mel::AnalogInput::Channel>(), std::size_t buffer_capacity = 200,
            std::size_t hp_filter_order = 4, double hp_filter_cutoff = 0.05,
            std::size_t lp_filter_order = 4, double lp_filter_cutoff = 0.01,
            std::size_t tkeo_lp_filter_order = 4, double tkeo_lp_filter_cutoff = 0.01);


        /// Update the voltage readings from the associated analog input channels on the DAQ, and apply all signal processing and update associated signals
        void update();

        /// Update and push to the buffers
        void update_and_buffer();

        /// Clear the buffers
        void clear_buffer();

        /// Resize the buffers to a new capacity
        void resize_buffer(std::size_t capacity);

        /// Get the current capacity of the MES buffers
        std::size_t get_buffer_capacity() const;

        /// Get the current size of the MES buffers
        std::size_t get_buffer_size() const;

        /// Check to see if the buffers are full
        bool is_buffer_full();

        /// Reset all signal processing used on each MES
        void reset_signal_processing();

        /// Return the number of MES in the array
        std::size_t size() const;

        /// Get raw MES
        const std::vector<double>& get_raw() const;

        /// Get demeaned MES
        const std::vector<double>& get_demean() const;

        /// Get processed MES
        const std::vector<double>& get_envelope() const;

        /// Get processed MES using TKEO
        const std::vector<double>& get_tkeo_envelope() const;

		/// Get mean of processed MES using TKEO
		const std::vector<double>& get_tkeo_envelope_mean() const;

        /// Get the last window_size elements pushed to the MES raw buffers
        std::vector<std::vector<double>> get_raw_buffer_data(std::size_t window_size) const;

        /// Get the last window_size elements pushed to the MES demean buffers
        std::vector<std::vector<double>> get_dm_buffer_data(std::size_t window_size) const;

        /// Get the last window_size elements pushed to the MES envelope buffers
        std::vector<std::vector<double>> get_env_buffer_data(std::size_t window_size) const;

        /// Get the last window_size elements pushed to the MES TKEO envelope buffers
        std::vector<std::vector<double>> get_tkeo_env_buffer_data(std::size_t window_size) const;

		/// Get the last window_size elements pushed to the MES TKEO envelope mean buffer
		std::vector<std::vector<double>> get_tkeo_env_mean_buffer_data(std::size_t window_size) const;

        /// Get the last window_size elements pushed to the MES raw buffer specified by mes_index
        std::vector<double> get_single_raw_buffer_data(std::size_t mes_index, std::size_t window_size) const;

        /// Get the last window_size elements pushed to the MES demean buffer specified by mes_index
        std::vector<double> get_single_dm_buffer_data(std::size_t mes_index, std::size_t window_size) const;

        /// Get the last window_size elements pushed to the MES envelope buffer specified by mes_index
        std::vector<double> get_single_env_buffer_data(std::size_t mes_index, std::size_t window_size) const;

        /// Get the last window_size elements pushed to the MES TKEO envelope buffer specified by mes_index
        std::vector<double> get_single_tkeo_env_buffer_data(std::size_t mes_index, std::size_t window_size) const;

    private:

        /// Put most recently updated signal in the buffer
        void push_buffer();

		/// Calculate the means of certain vector signals
		void calculate_means();

    private:

        std::vector<mel::uint32> channel_numbers_; ///< numbers associated by the DAQ with each of the analog input channels
        std::size_t mes_count_; ///< length of the MES array

        std::vector<MyoelectricSignal> mes_; ///< vector of MES

        std::vector<double> raw_; ///< vector of raw MES [V]
        std::vector<double> demean_; ///< vector of MES after high-pass filtering to remove mean and motion artifacts
        std::vector<double> envelope_; ///< vector of MES evnelope from rectification and low-pass filtering
        std::vector<double> tkeo_envelope_; ///< vector of MES envelope from TKEO, rectification, and low-pass filtering
		std::vector<double> tkeo_envelope_mean_; /// < mean of the vector of MES envelope from TKEO, rectification, and low-pass filtering

        std::size_t buffer_capacity_; ///< capacity of all of the signal buffers (should always be the same for all MES)   

        mel::RingBuffer<std::vector<double>> raw_buffer_; ///< signal buffer to hold time history of raw MES vectors
        mel::RingBuffer<std::vector<double>> dm_buffer_; ///< signal buffer to hold time history of demean MES vectors
        mel::RingBuffer<std::vector<double>> env_buffer_; ///< signal buffer to hold time history of MES envelope vectors
        mel::RingBuffer<std::vector<double>> tkeo_env_buffer_; ///< signal buffer to hold time history of MES TKEO envelope vectors   
		mel::RingBuffer<std::vector<double>> tkeo_env_mean_buffer_; ///< signal buffer to hold time history of MES TKEO envelope vector means 

    };

} // namespace meii

#endif // MEII_MES_ARRAY_HPP
