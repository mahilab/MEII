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

#ifndef MEII_MYOELECTRIC_SIGNAL_HPP
#define MEII_MYOELECTRIC_SIGNAL_HPP

#include <MEL/Core/Device.hpp>
#include <MEL/Daq/Input.hpp>
#include <MEL/Math/Butterworth.hpp>
#include <MEII/SignalProcessing/Rectifier.hpp>
#include <MEII/SignalProcessing/TeagerKaiserEnergyOperator.hpp>
#include <MEL/Utility/RingBuffer.hpp>
#include <vector>

namespace meii {

    //==============================================================================
    // CLASS DECLARATION
    //==============================================================================

    class MyoelectricSignal : public mel::Device {

    public:

        /// Constructor
        MyoelectricSignal(mel::AnalogInput::Channel ai_channel, std::size_t buffer_capacity = 200,
            std::size_t hp_filter_order = 4, double hp_filter_cutoff = 0.05,
            std::size_t lp_filter_order = 4, double lp_filter_cutoff = 0.01,
            std::size_t tkeo_lp_filter_order = 4, double tkeo_lp_filter_cutoff = 0.01);

		/// Override on_enable from device class
		bool on_enable() override;

		/// Override on_enable from device class
		bool on_disable() override;

        /// Update the voltage reading from the associated analog input channel on the DAQ, and apply all signal processing and update associated signals
        void update();

        /// Update and push to the buffer
        void update_and_buffer();

        /// Clear the buffer
        void clear_buffer();

        /// Resize the buffer to a new capacity
        void resize_buffer(std::size_t capacity);

        /// Get the current capacity of the MES buffer
        std::size_t get_buffer_capacity() const;

        /// Get the current size of the MES buffer
        std::size_t get_buffer_size() const;

        /// Check to see if the buffer is full
        bool is_buffer_full();

        /// Reset all signal processing used on MES
        void reset_signal_processing();

        /// Get raw MES
        double get_raw() const;

        /// Get demeaned MES
        double get_demean() const;

        /// Get processed MES
        double get_envelope() const;

        /// Get processed MES using TKEO
        double get_tkeo_envelope() const;

        /// Get the last window_size elements pushed to the MES raw buffer
        std::vector<double> get_raw_buffer_data(std::size_t window_size) const;

        /// Get the last window_size elements pushed to the MES demean buffer
        std::vector<double> get_dm_buffer_data(std::size_t window_size) const;

        /// Get the last window_size elements pushed to the MES envelope buffer
        std::vector<double> get_env_buffer_data(std::size_t window_size) const;

        /// Get the last window_size elements pushed to the MES TKEO envelope buffer
        std::vector<double> get_tkeo_env_buffer_data(std::size_t window_size) const;


    private:

        /// Put most recently updated signal in the buffer
        void push_buffer();

    private:

        mel::AnalogInput::Channel ai_channel_; ///< analog input of the associated DAQ

        double raw_; ///< raw MES [V]
        double demean_; ///< MES after high-pass filtering to remove mean and motion artifacts
        double envelope_; ///< MES evnelope from rectification and low-pass filtering
        double tkeo_envelope_; ///< MES envelope from TKEO, rectification, and low-pass filtering

        std::size_t buffer_capacity_; ///< capacity of MES buffer
        mel::RingBuffer<double> raw_buffer_; ///< signal buffer to hold time history of raw MES
        mel::RingBuffer<double> dm_buffer_; ///< signal buffer to hold time history of demeaned MES
        mel::RingBuffer<double> env_buffer_; ///< signal buffer to hold time history of MES envelope
        mel::RingBuffer<double> tkeo_env_buffer_; ///< signal buffer to hold time history of MES TKEO envelope

        mel::Filter hp_filter_; ///< high-pass filter for first phase of standard MES processing
        Rectifier full_rect_; ///< full-wave rectifier for second phase of standard MES processing
        mel::Filter lp_filter_; ///< low-pass filter for third phase of standard MES processing
		Rectifier half_rect_; ///< half-wave rectifier for fourth and final phase of standard MES processing
        TeagerKaiserEnergyOperator tkeo_; ///< teager-kaiser energy operator for second phase of TKEO MES processing
        Rectifier tkeo_full_rect_; ///< full-wave rectifier for third phase of standard MES processing
        mel::Filter tkeo_lp_filter_; ///< low-pass filter for fourth and final phase of TKEO MES processing
		Rectifier tkeo_half_rect_; ///< half-wave rectifier for fifth and final phase of TKEO MES processing

    };

} // namespace meii

#endif // MEII_MYOELECTRIC_SIGNAL_HPP
