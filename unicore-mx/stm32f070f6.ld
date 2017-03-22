/* Linker script for STM32F070F6, 32k flash, 6k RAM. */

/* Define memory regions. */
MEMORY
{
	rom (rx) : ORIGIN = 0x08000000, LENGTH = 32K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 6K
}

/* Include the common ld script. */
INCLUDE libucmx_stm32f0.ld
