/*
 * Generated with the FPGA Interface C API Generator 17.0.0
 * for NI-RIO 17.0.0 or later.
 */

#ifndef __NiFpga_quadrature_h__
#define __NiFpga_quadrature_h__

#ifndef NiFpga_Version
   #define NiFpga_Version 1700
#endif

#include "NiFpga.h"

/**
 * The filename of the FPGA bitfile.
 *
 * This is a #define to allow for string literal concatenation. For example:
 *
 *    static const char* const Bitfile = "C:\\" NiFpga_quadrature_Bitfile;
 */
#define NiFpga_quadrature_Bitfile "NiFpga_quadrature.lvbitx"

/**
 * The signature of the FPGA bitfile.
 */
static const char* const NiFpga_quadrature_Signature = "800D56442D6B8CA806E1B231FD114D87";

typedef enum
{
   NiFpga_quadrature_IndicatorBool_A = 0x18016,
   NiFpga_quadrature_IndicatorBool_B = 0x1800E,
   NiFpga_quadrature_IndicatorBool_Index = 0x18012,
} NiFpga_quadrature_IndicatorBool;

typedef enum
{
   NiFpga_quadrature_IndicatorI16_velocity2 = 0x1800A,
} NiFpga_quadrature_IndicatorI16;

typedef enum
{
   NiFpga_quadrature_IndicatorI32_WaveformChart = 0x18018,
   NiFpga_quadrature_IndicatorI32_encoderposition = 0x18004,
   NiFpga_quadrature_IndicatorI32_index_save = 0x18028,
} NiFpga_quadrature_IndicatorI32;

typedef enum
{
   NiFpga_quadrature_IndicatorU64_Tick = 0x18000,
} NiFpga_quadrature_IndicatorU64;

typedef enum
{
   NiFpga_quadrature_ControlBool_direction = 0x1802E,
   NiFpga_quadrature_ControlBool_disable = 0x18022,
   NiFpga_quadrature_ControlBool_reset = 0x18026,
} NiFpga_quadrature_ControlBool;

typedef enum
{
   NiFpga_quadrature_ControlI32_reset_value = 0x1801C,
} NiFpga_quadrature_ControlI32;

#endif
