# Embedded_System_Concepts
 Learn an embedded system concepts like startup code, linker script, RTOS, ..etc
 
## StartUp code
piece of code run before my main function to initialize the Vector table and 
ram sections .data, .bss, stack, heap, and any other code need to run before the
main like speed up the processor by configuring the PLL or jumping to the 
bootloader to update the code or choose from two apps existing in the flash or 
any option application required before the main app run.

Strat-up code for most processors written in assemblly, because SP not set up 
yet and .data not copy from ROM to RAM, and .bss isn't cleared ye. but ARM 
Cortex-M designed to reduce the need for low-level assembly.


### Example:
in IAR ide:
-- open project option -> Debugger == uncheck "Run to main".
    Now try to start debugging the code not starting from the main but starting 
    from __iar_program_start. 
    Note that all CPU registers are 0 except the Stack pointer (SP) and Program 
    counter (PC).
    then code branch (BL) to ?main
    in function called __Cmain (?main):
        the code branch to __low_level_init function after the controller 
        completes executing some code. it branches to the main and starts 
        executing my code

    why PC and SP are not 0 like other CPU registers?
        because ARM Cortex-M4 hardwire to load address 0x00 into SP and load 
        address 0x04 into PC at power up. this info is mentioned in the data 
        sheet in the vector table section.
        star debugging and check "Disassembly" address 0x00 and 0x04 - address 
        0x04 value is odd because the LSB of any value loaded to the PC must be 
        1 because this bit indicates the Thumb mode of the processor which is 
        supported by Cortex-M -


-- open project option --> linker --> list == check "Generate linker map file"
    now check the generated map file and you will find .text, .bss, CSTACK, and 
    .rodata sections in "PLACEMENT SUMMARY" and in "MODULE SUMMARY" you will 
    find each module ro code, ro data, and rw data size.