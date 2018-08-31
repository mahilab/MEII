// MIT License
//
// MEL - Mechatronics Engine & Library
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
// Author(s): Nathan Dunkelberger (nbd2@rice.edu)

#include <MEII/Daq/Crio.hpp>
#include <MEL/Utility/System.hpp>
#include <MEL/Logging/Log.hpp>
#include "Detail/NiFpga.h"


namespace meii {

//==============================================================================
// STATIC VARIABLES
//==============================================================================

//==============================================================================
// CLASS DEFINITIONS
//==============================================================================

Crio::Crio(const std::string& name) :
    DaqBase(name),
    Crio9401(*this, std::vector<mel::uint32> {0,1,2,3,4,5,6,7}),
{
	NiFpga_Session session;
}


bool Crio::on_open() {
    NiFpga_Status status = NiFpga_Initialize();
    if (NiFpga_IsNotError(status))
    {
        return true;
    }
    return false;
}

bool Crio::on_close(){
    /* Close the session */
	NiFpga_MergeStatus(&status, NiFpga_Close(session, 0));

	/* Finalize must be called before exiting program and after closing FPGA session */
	NiFpga_MergeStatus(&status, NiFpga_Finalize());

	/* Return an error code if there is an error */
	if(NiFpga_IsError(status))
	{
		printf("Error! Exiting program. LabVIEW error code: %d\n", status);
	}
	return 0;
}

bool Crio::on_enable(){
    return true;
}

bool Crio::on_disable(){
    return true;
}

bool Crio::update_input() {
    return true;// return (A.update_input() && B.update_input() && C.update_input());
}

bool Crio::update_output() {
    return true;// return (A.update_output() && B.update_output() && C.update_output());
}

}