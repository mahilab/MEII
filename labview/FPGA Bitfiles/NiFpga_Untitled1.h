/*
 * Generated with the FPGA Interface C API Generator 17.0.0
 * for NI-RIO 17.0.0 or later.
 */

#ifndef __NiFpga_Untitled1_h__
#define __NiFpga_Untitled1_h__

#ifndef NiFpga_Version
   #define NiFpga_Version 1700
#endif

#include "NiFpga.h"

/**
 * The filename of the FPGA bitfile.
 *
 * This is a #define to allow for string literal concatenation. For example:
 *
 *    static const char* const Bitfile = "C:\\" NiFpga_Untitled1_Bitfile;
 */
#define NiFpga_Untitled1_Bitfile "NiFpga_Untitled1.lvbitx"

/**
 * The signature of the FPGA bitfile.
 */
static const char* const NiFpga_Untitled1_Signature = "CEC79C52B29294B7FCB982D632F9A543";

typedef enum
{
   NiFpga_Untitled1_IndicatorBool_Crio9401DI_0 = 0x1803E,
   NiFpga_Untitled1_IndicatorBool_Crio9401DI_1 = 0x18006,
   NiFpga_Untitled1_IndicatorBool_Crio9401DI_2 = 0x1800E,
   NiFpga_Untitled1_IndicatorBool_Crio9401DI_3 = 0x18016,
   NiFpga_Untitled1_IndicatorBool_Crio9401DI_4 = 0x1801E,
   NiFpga_Untitled1_IndicatorBool_Crio9401DI_5 = 0x18026,
   NiFpga_Untitled1_IndicatorBool_Crio9401DI_6 = 0x1802E,
   NiFpga_Untitled1_IndicatorBool_Crio9401DI_7 = 0x18036,
} NiFpga_Untitled1_IndicatorBool;

typedef enum
{
   NiFpga_Untitled1_ControlBool_Crio9401D0_0 = 0x1803A,
   NiFpga_Untitled1_ControlBool_Crio9401D0_1 = 0x1800A,
   NiFpga_Untitled1_ControlBool_Crio9401D0_2 = 0x18012,
   NiFpga_Untitled1_ControlBool_Crio9401D0_3 = 0x1801A,
   NiFpga_Untitled1_ControlBool_Crio9401D0_4 = 0x18022,
   NiFpga_Untitled1_ControlBool_Crio9401D0_5 = 0x1802A,
   NiFpga_Untitled1_ControlBool_Crio9401D0_6 = 0x18032,
   NiFpga_Untitled1_ControlBool_Crio9401D0_7 = 0x18002,
   NiFpga_Untitled1_ControlBool_Stop = 0x18046,
} NiFpga_Untitled1_ControlBool;

typedef enum
{
   NiFpga_Untitled1_ControlArrayI16_IO_Enable = 0x18040,
} NiFpga_Untitled1_ControlArrayI16;

typedef enum
{
   NiFpga_Untitled1_ControlArrayI16Size_IO_Enable = 8,
} NiFpga_Untitled1_ControlArrayI16Size;

#endif
