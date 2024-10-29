/*****************************************************************************
* linker command file for C6748 test code.
*
* © Copyright 2009, Logic Product Development Company. All Rights Reserved.
******************************************************************************/

//-l rts67plus.lib
//-l ..\..\..\..\bsl\lib\evmc6748_bsl.lib

-stack           0x00000800
-heap            0x00000800

MEMORY
{
   dsp_l2_ram:      ORIGIN = 0x11800000  LENGTH = 0x00040000
   shared_ram:      ORIGIN = 0x80000000  LENGTH = 0x00020000
   external_ram:    ORIGIN = 0xC0000000  LENGTH = 0x08000000
   arm_local_ram:   ORIGIN = 0xFFFF0000  LENGTH = 0x00002000
}

SECTIONS
{
   .text       > shared_ram
   .const      > shared_ram
   .bss        > shared_ram
   .far        > shared_ram
   .switch     > shared_ram
   .stack      > shared_ram
   .data       > shared_ram
   .cinit      > shared_ram
   .sysmem     > shared_ram
   .cio        > shared_ram
}

