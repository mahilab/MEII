// MIT License
//
// Copyright (c) 2020 Mechatronics and Haptic Interfaces Lab - Rice University
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
// Author(s): Evan Pezent (epezent@rice.edu)

#include <Mahi/Daq.hpp>
#include <Mahi/Util.hpp>

using namespace mahi::daq;
using namespace mahi::util;

int main(int argc, char const *argv[])
{
    MahiLogger->set_max_severity(Verbose);

    // For this example, connect AO0 to AI0 and DIO0 to DIO1, 
    // PWM0 to DIO2, and connect an encoder to channel 0.
    QPid qpid;
    if (!qpid.is_open())
        return 1;

    /// Print the DAQ info
    // print_info(qpid);
    qpid.DO.set_channels({0,1,2,3,4,5,6,7});

    for (size_t i = 0; i < 8; i++){
        qpid.DO.enable_values[i] = TTL_HIGH;
        qpid.DO.disable_values[i] = TTL_HIGH;
        // qpid.DO.expire_values[i] = TTL_HIGH;
    }
    
    

    qpid.AO.enable_values[1] = 0.0;
    qpid.AO.disable_values[1] = 0.0;
    
    // Set the units on one of our encoders
    qpid.encoder.units[6] = 360.0 / 2048 * 0.42 / 4.5;
    qpid.encoder.units[2] = 360.0 / 2048 * 0.0662864;
    qpid.encoder.units[3] = 2.0*PI / 2048 * 0.23*INCH2METER;
    qpid.encoder.units[4] = 2.0*PI / 2048 * 0.23*INCH2METER;
    qpid.encoder.units[5] = 2.0*PI / 2048 * 0.23*INCH2METER;
    // Zero the encoder
    qpid.encoder.zero(6);
    qpid.encoder.zero(2);
    qpid.encoder.zero(3);
    qpid.encoder.zero(4);
    qpid.encoder.zero(5);



    /// Enable, this will set enable values on AO and DO, which we can read on AI[0] and DI[0]
    qpid.enable();
    sleep(1_ms);

    prompt("Press ENTER to start I/O loop.");

    bool dig_out = TTL_HIGH;
    /// General I/O loop
    for (int i = 0; i < 500; ++i) {
        
        if(!(i%50)) dig_out = !dig_out;
        /// Synced read, reads all DAQ inputs
        qpid.read_all();
        print("encoder[0]: {:+.2f} deg, {} counts", qpid.encoder.positions[6], qpid.encoder[6]);
        print("encoder[2]: {:+.2f} deg.", qpid.encoder.positions[2]);
        print("encoder[3]: {:+.4f} deg.", qpid.encoder.positions[3]);
        print("encoder[4]: {:+.4f} deg.", qpid.encoder.positions[4]);
        print("encoder[5]: {:+.4f} deg.\n", qpid.encoder.positions[5]);
        qpid.AO[1] = 5.0;
        qpid.AO[2] = 6.0;
        qpid.AO[3] = 7.0;
        qpid.AO[4] = 8.0;
        qpid.AO[5] = 9.0;

        qpid.DO[1] = dig_out;
        qpid.DO[2] = dig_out;
        qpid.DO[3] = dig_out;
        qpid.DO[4] = dig_out;
        qpid.DO[5] = dig_out;

        qpid.write_all();
        sleep(50_ms);
    }
    qpid.disable();
    qpid.close();

    return 0;
}
