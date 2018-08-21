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

#ifndef MEII_VIRTUAL_INPUT_HPP
#define MEII_VIRTUAL_INPUT_HPP

#include <MEL/Daq/Input.hpp>
#include <MEL/Utility/RingBuffer.hpp>
#include <chrono>
#include <random>

namespace meii {

	class VirtualInput : public mel::Input<mel::Voltage> {

	public:

		VirtualInput(const std::string& name, std::vector<mel::uint32> channel_numbers);

		~VirtualInput() override;
		
		bool on_enable() override;
		bool on_disable() override;
		bool update() override;
		bool update_channel(mel::uint32 channel_number) override;

	private:

		mel::uint64 seed_;
		std::default_random_engine generator_;
		std::normal_distribution<double> distribution_;

	};

} // namespace meii

#endif  // MEII_VIRTUAL_INPUT_HPP
