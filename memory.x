/* filepath: memory.x */
MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    RAM   : ORIGIN = 0x20000000, LENGTH = 520K
}

/* Fictitious region that represents the memory available to the stack */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);