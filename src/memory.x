/* 
Partial linker script with memory definitions for TM4C129
This portion of the linker script is INCLUDE'd in the more complete "link.x" (line 23 of link.x)
which is constructed by cortex-m-rt's build script and referenced by the "-T link.x" 
compiler directive in .cargo/config .

Note there is an error in the datasheet which is corrected here (datasheet has wrong end address for SRAM)
See https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/arm-based-microcontrollers/f/arm-based-microcontrollers-forum/584243/tm4c1294ncpdt-error-in-address-range-of-sram-in-manual

Linker configuration is in .cargo/config (specifies use of link.x script & any other compiler or linker options)
build.rs adds this file to the rustc OUT_DIR so that it can be found by link.x
*/

MEMORY
{
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 1M
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 256K
}
